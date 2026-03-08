#!/usr/bin/env python3

import subprocess
import tempfile
import time
import csv
import os
import json
import signal
import argparse
import sys
import threading
import collections

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Patch
from matplotlib.collections import LineCollection
from Log import *

# ControlMode enum (matches firmware ControlMode)
class ControlMode:
    PulseInjection = 0
    Alignment      = 1
    Drag           = 2
    ClosedLoop     = 3
    Stalled        = 4
    Stopped        = 5
    Fault          = 6

# ===================== MOTOR CONSTANTS =====================
MCPWM_FREQ   = 20_000_000
MCPWM_PERIOD = 1000
MOTOR_LOOP_FREQ = MCPWM_FREQ / MCPWM_PERIOD  # 20,000 Hz
ALARM_US = 1_000_000 / MOTOR_LOOP_FREQ        # 50 µs

POLE_PAIRS = 6
RPM_COEFF  = 60.0 * (1_000_000 / ALARM_US) / POLE_PAIRS

avg_window_ticks = int(0.02 * (1_000_000 / ALARM_US))  # 400 ticks = 20ms

# ===================== ARGUMENTS =====================
parser = argparse.ArgumentParser(description="ESP32 trace capture/view tool")

parser.add_argument("-d",   "--duration", type=int, default=5)
parser.add_argument("-o",   "--output", default="trace.csv")
parser.add_argument("--input", help="Open existing trace (bin or csv)")
parser.add_argument("--port", default="/dev/ttyACM0",
                    help="Serial port for USB-CDC capture")
parser.add_argument("--ocd", default=None,
                    help="Path to openocd binary (enables reset-halt before capture)")
parser.add_argument("--gdb", default=None, metavar="ELF",
                    help="Path to ELF file; writes a VSCode launch config and waits for you to attach GDB before capturing (does not require --ocd)")
parser.add_argument("-f",   default="board/esp32s3-builtin.cfg",
                    help="OpenOCD board cfg (used with --ocd)")

args = parser.parse_args()

TRACE_DURATION = args.duration
CSV_OUTPUT     = args.output
INPUT_FILE     = args.input

# ===================== GLOBAL ARRAYS =====================
samples = {}   # key = sample index

timestamps=[]
valley_offsets=[]
phase_vals=[]
neutral_vals=[]
hiZ=[]
phase=[]
mode=[]
sampleCount=[]
duty=[]
ticksToNext=[]

# trace_events: list of (timestamp_ticks, event_idx, event_name)
trace_events = []
event_names  = {}   # idx → name, populated from HEADER frame

# ===================== PACKET CONSTANTS =====================
MAGIC = b'\xA5\xA5\xA5\xA5'
PAYLOAD_SIZE = 11
PACKET_SIZE = 4 + PAYLOAD_SIZE + 1

# ===================== CRC8 DALLAS =====================
from Utilities import crc8_dallas

# ===================== LEGACY DECODER (old framing format) =====================
def decode_stream(data):

    i = 0
    valid = 0
    bad_crc = 0
    duplicates = 0

    while i <= len(data) - PACKET_SIZE:
        # 1️⃣ Magic check
        if data[i:i+4] != MAGIC:
            i += 1
            continue

        # 2️⃣ Enough room for full packet?
        if i + PACKET_SIZE > len(data):
            break

        # 3️⃣ Fixed-spacing structure check
        next_magic_pos = i + PACKET_SIZE
        if next_magic_pos + 4 <= len(data):
            if data[next_magic_pos:next_magic_pos+4] != MAGIC:
                i += 1
                continue

        # 4️⃣ Extract payload + CRC
        payload = data[i+4:i+4 + PAYLOAD_SIZE]
        crc_rx = data[i+4 + PAYLOAD_SIZE]

        # 5️⃣ CRC check
        if crc8_dallas(payload) != crc_rx:
            bad_crc += 1
            i += 1
            continue

        # 6️⃣ Parse payload (now safe)
        val = int.from_bytes(payload, "little")

        ts16    = (val >> 0)  & 0xFFFF
        adc_ph  = (val >> 16) & 0xFFF
        adc_neutral = (val >> 28) & 0xFFF
        hiz     = (val >> 40) & 0x3
        ph      = (val >> 42) & 0x7
        cm      = (val >> 45) & 0x7
        sc24    = (val >> 48) & 0xFFFFFF
        duty8   = (val >> 72) & 0xFF
        ticks   = (val >> 80) & 0xFF

        # 7️⃣ Basic payload sanity
        if adc_ph > 4095 or adc_neutral > 4095 or hiz > 2 or ph > 5:
            i += 1
            continue

        # 8️⃣ Deduplicate
        if sc24 not in samples:
            samples[sc24] = (ts16, adc_ph, adc_neutral, hiz, ph, cm, duty8, ticks)
        else:
            duplicates += 1

        valid += 1
        i += PACKET_SIZE

    log_header(Level.INFO, "TRACE DECODE")
    log_stat("Valid packets:", valid)
    log_stat("Bad CRC:", bad_crc, Level.ERROR if bad_crc > 100 else Level.WARN if bad_crc > 0 else Level.INFO)
    log_stat("Duplicates:", duplicates, Level.WARN if duplicates > 0 else Level.INFO)
    log_stat("Unique samples:", len(samples))
    log_footer(Level.INFO)

    if not samples:
        log(Level.ERROR, "No valid samples")
        sys.exit(1)

    # sort by sample index
    ordered = sorted(samples.items())

    # timestamp wrap reconstruction
    global_ts = 0
    last_ts = None

    for sc, pkt in ordered:
        ts16, adc_ph, adc_neutral, hiz, ph, cm, duty8, ticks = pkt

        if last_ts is not None and ts16 < last_ts:
            global_ts += 65536
        last_ts = ts16

        timestamps.append(global_ts + ts16)
        phase_vals.append(adc_ph)
        neutral_vals.append(adc_neutral)
        hiZ.append(hiz)
        phase.append(ph)
        mode.append(cm)
        sampleCount.append(sc)
        duty.append(duty8)
        ticksToNext.append(ticks)
        valley_offsets.append(0)  # not present in legacy CSV format

# ===================== BINARY TRACE DECODER =====================
TRACE_SAMPLE_BYTES = 7

def decode_binary_trace(data, sample_bytes=None):
    """Decode raw binary trace data into the global sample arrays."""
    sample_size = sample_bytes if sample_bytes is not None else TRACE_SAMPLE_BYTES
    n_samples = len(data) // sample_size
    remainder = len(data) % sample_size

    if remainder != 0:
        log(Level.WARN, f"{remainder} trailing bytes ignored (not a multiple of {sample_size})")

    log_header(Level.INFO, "BINARY TRACE DECODE")
    log_stat("Raw bytes:", len(data))
    log_stat("Sample size:", f"{sample_size} bytes")
    log_stat("Samples:", n_samples)

    global_ts = 0
    sample_index = 0

    for i in range(n_samples):
        offset = i * sample_size
        raw = data[offset:offset + sample_size]

        # byte 0: timestampDelta
        ts_delta = raw[0]

        # bytes 1-3: phaseValue:12, neutralValue:12
        mid = int.from_bytes(raw[1:4], 'little')
        phase_val   = mid & 0xFFF
        neutral_val = (mid >> 12) & 0xFFF

        # byte 4: phase:3, controlMode:3, padding:2
        b4 = raw[4]
        motor_step   = b4 & 0x7
        control_mode = (b4 >> 3) & 0x7

        # byte 5: dutyCycle
        duty_val = raw[5]

        # byte 6: ticksToNextStep; bytes 7-8: valleyOffsetUs (int16, optional)
        ticks_val     = raw[6] if sample_size > 6 else 0
        valley_offset = int.from_bytes(raw[7:9], "little", signed=True) if sample_size >= 9 else 0

        # Skip zero samples (uninitialised buffer padding)
        if ts_delta == 0 and phase_val == 0 and neutral_val == 0 and duty_val == 0:
            continue

        global_ts += ts_delta

        timestamps.append(float(global_ts))
        phase_vals.append(phase_val)
        neutral_vals.append(neutral_val)
        hiZ.append(motor_step % 3)
        phase.append(motor_step)
        mode.append(control_mode)
        sampleCount.append(sample_index)
        duty.append(duty_val)
        ticksToNext.append(ticks_val)
        valley_offsets.append(valley_offset)

        sample_index += 1

    decode_level = Level.ERROR if sample_index == 0 else Level.WARN if sample_index < n_samples * 0.95 else Level.INFO
    log_stat("Decoded:", sample_index, decode_level)
    log_footer(Level.INFO)

    if sample_index == 0:
        log(Level.ERROR, "No valid samples decoded")
        sys.exit(1)

# ===================== LOAD MODE =====================
if INPUT_FILE:

    if INPUT_FILE.lower().endswith(".csv"):
        log(Level.INFO, "Loading CSV")
        with open(INPUT_FILE) as f:
            reader = csv.DictReader(f)
            for row in reader:
                timestamps.append(float(row["timestamp"]))
                phase_vals.append(float(row["phaseValue"]))
                neutral_vals.append(float(row["neutralValue"]))
                hiZ.append(int(row["highImpedencePhase"]))
                phase.append(int(row["phase"]))
                mode.append(int(row["controlMode"]))
                sampleCount.append(int(row["sampleCount"]))
                duty.append(int(row.get("duty", 0)))
                ticksToNext.append(int(row.get("ticks", 0)))
                valley_offsets.append(int(row.get("valleyOffset", 0)))

    elif INPUT_FILE.lower().endswith(".ezdc"):
        log(Level.INFO, "Loading EZDC")
        from CdcCapture import CdcCapture
        with open(INPUT_FILE, "rb") as f:
            raw_stream = f.read()
        raw_data, raw_events, hdr = CdcCapture.parse_stream(raw_stream)
        if hdr:
            event_names.update(hdr.get("event_names", {}))
        for ts, idx in raw_events:
            name = event_names.get(idx, f"Event{idx}")
            trace_events.append((float(ts), idx, name))
        log(Level.INFO, f"Raw bytes: {len(raw_stream)}")
        decode_binary_trace(raw_data, sample_bytes=hdr.get("sample_bytes") if hdr else None)

    else:
        log(Level.INFO, "Loading binary trace")
        with open(INPUT_FILE,"rb") as f:
            data=f.read()
        log(Level.INFO, f"Raw bytes: {len(data)}")
        decode_binary_trace(data)

# ===================== CAPTURE MODE (CDC) =====================
else:
    from CdcCapture import CdcCapture

    cap = CdcCapture(
        port=args.port,
        openocd=args.ocd,
        board_cfg=args.f,
        gdb_elf=args.gdb,
    )
    try:
        raw_data, raw_events, hdr, raw_stream = cap.capture(duration=TRACE_DURATION)
    except KeyboardInterrupt:
        log(Level.WARN, "Interrupted")
        raw_data   = bytes(cap._samples) if cap._samples else b""
        raw_events = list(cap._events)
        hdr        = cap._header
        raw_stream = bytes(cap._raw_stream)
    finally:
        cap.close()

    if hdr:
        event_names.update(hdr.get("event_names", {}))
    for ts, idx in raw_events:
        name = event_names.get(idx, f"Event{idx}")
        trace_events.append((float(ts), idx, name))

    # Save raw wire stream as .ezdc
    ezdc_path = os.path.splitext(CSV_OUTPUT)[0] + ".ezdc"
    with open(ezdc_path, "wb") as f:
        f.write(raw_stream)
    log(Level.INFO, f"EZDC saved: {ezdc_path}")

    log(Level.INFO, "Decoding trace data...")
    decode_binary_trace(raw_data, sample_bytes=hdr.get("sample_bytes") if hdr else None)

# ===================== SAVE CSV =====================
if timestamps:
    with open(CSV_OUTPUT,"w",newline="") as csvfile:
        writer=csv.writer(csvfile)
        writer.writerow(["timestamp","phaseValue","neutralValue","highImpedencePhase","phase","controlMode","sampleCount","duty","ticks","valleyOffset"])
        for row in zip(timestamps,phase_vals,neutral_vals,hiZ,phase,mode,sampleCount,duty,ticksToNext,valley_offsets):
            writer.writerow(row)
    log(Level.INFO, f"CSV saved: {CSV_OUTPUT}")

# ===================== LOSS ANALYSIS =====================
if not sampleCount:
    log(Level.ERROR, "No data")
    sys.exit(0)

sc = np.array(sampleCount, dtype=np.uint32)

expected = sc[-1] - sc[0] + 1
received = len(sc)
lost = expected - received

log_header(Level.INFO, "SAMPLE LOSS STATS")
log_stat("Received samples:", received)
log_stat("Expected from counter:", expected)
lost_level = Level.ERROR if lost > expected * 0.10 else Level.WARN if lost > 0 else Level.INFO
log_stat("Total missed samples:", lost, lost_level)
if expected > 0:
    loss_pct = (lost / expected) * 100
    loss_level = Level.ERROR if loss_pct > 10.0 else Level.WARN if loss_pct > 0.1 else Level.INFO
    log_stat("Loss percentage:", f"{loss_pct:.3f}%", loss_level)
log_footer(Level.INFO)

# ===================== CONVERT =====================
t=np.array(timestamps)
pv=np.array(phase_vals)
neutral=np.array(neutral_vals)
ph=np.array(phase)
cm=np.array(mode)
hiz=np.array(hiZ)
sc = np.array(sampleCount, dtype=np.uint32)
duty_arr = np.array(duty, dtype=float)
duty_pct = duty_arr * (100.0 / 255.0)
ticks_arr = np.array(ticksToNext)

t = t - t[0]

# ===================== ZERO CROSS DETECTION =====================
zc_times = []
comm_predict_times = []

TICK_US = 100   # motor loop tick period in microseconds

for i in range(1, len(t)):
    if cm[i] != 3:
        continue

    # zero-cross just detected
    if ticks_arr[i] > 0 and ticks_arr[i-1] == 0:
        zc_times.append(t[i])

    # predicted commutation moment
    if ticks_arr[i] > 0 and ticks_arr[i] < 255:
        comm_time = t[i] + ticks_arr[i] * TICK_US
        comm_predict_times.append(comm_time)

# ===================== STEP DURATION TRACKING =====================
step_times = []
step_durations = []

last_step_time = None
last_phase = ph[0]

for i in range(1, len(t)):
    if cm[i] != 3:  # closed-loop only
        last_phase = ph[i]
        continue

    if ph[i] != last_phase:
        now = t[i]

        if last_step_time is not None:
            dur = now - last_step_time
            step_durations.append(dur)
            step_times.append(now)

        last_step_time = now

    last_phase = ph[i]

# convert to ticks (same units firmware uses)
step_durations_ticks = np.array(step_durations) / ALARM_US

# ===================== RPM CALC (FIRMWARE ACCURATE) =====================
rpm_times = []
rpm_values = []

avg_window_ticks  = 20_000   # same as kSpeedAveragingDuration
durations         = []
last_step_time    = None
current_step_ticks = None
saturated_count   = None
saturated_reliable = True

for i in range(len(t)):
    if cm[i] not in (ControlMode.Drag, ControlMode.ClosedLoop):
        continue

    tick_val  = int(ticks_arr[i])
    prev_tick = int(ticks_arr[i-1]) if i > 0 else 0
    sc_gap    = int(sc[i] - sc[i-1]) if i > 0 else 1

    if i > 0 and tick_val > 0 and prev_tick == 0:
        if current_step_ticks is not None:
            durations.insert(0, current_step_ticks)
        current_step_ticks = None
        last_step_time     = t[i]

        if tick_val < 255:
            current_step_ticks  = float(tick_val)
            saturated_count     = None
            saturated_reliable  = True
        else:
            saturated_count    = 255
            saturated_reliable = True

    elif saturated_count is not None and tick_val > 0:
        if sc_gap > 255:
            saturated_reliable = False

        if tick_val == 255 and saturated_reliable:
            saturated_count += sc_gap
        elif tick_val < 255:
            if saturated_reliable:
                current_step_ticks = float(saturated_count + tick_val - 255)
            saturated_count    = None
            saturated_reliable = True

    elif i > 0 and ph[i] != ph[i-1] and current_step_ticks is None and saturated_count is None:
        now = t[i]
        if last_step_time is not None:
            durations.insert(0, (now - last_step_time) / ALARM_US)
        last_step_time = now

    if not durations:
        continue

    kept        = []
    total_ticks = 0
    for d in durations:
        kept.append(d)
        total_ticks += d
        if total_ticks >= avg_window_ticks and len(kept) > 1:
            break
    durations = kept

    if not durations:
        continue

    avg = sum(durations) / len(durations)
    if avg <= 0:
        continue

    if last_step_time is not None:
        time_in_step_ticks = (t[i] - last_step_time) / ALARM_US
        if time_in_step_ticks > avg:
            avg = (total_ticks + time_in_step_ticks) / (len(kept) + 1)

    rpm = RPM_COEFF / avg
    rpm_times.append(t[i])
    rpm_values.append(rpm)

# ===================== BUILD CONTINUOUS RPM TRACE =====================
rpm_full = np.full_like(t, np.nan, dtype=float)

if len(rpm_times):
    idx = 0
    current_rpm = rpm_values[0]
    for i in range(len(t)):
        while idx + 1 < len(rpm_times) and t[i] >= rpm_times[idx + 1]:
            idx += 1
            current_rpm = rpm_values[idx]
        rpm_full[i] = current_rpm

rpm_arr = np.array(rpm_values)
if len(rpm_arr):
    RPM_YMIN = 0
    RPM_YMAX = float(np.nanmax(rpm_arr)) * 1.2
else:
    RPM_YMIN = 0
    RPM_YMAX = 1000

log_sub(Level.DEBUG, f"RPM values:  {len(rpm_values)}")
log_sub(Level.DEBUG, f"RPM range:   {RPM_YMIN:.0f} - {RPM_YMAX:.0f}")
log_sub(Level.DEBUG, f"RPM non-nan: {np.count_nonzero(~np.isnan(rpm_full))}")

# ===================== BUILD PER-PHASE ARRAYS =====================
u_vals = np.full_like(pv, np.nan, dtype=float)
v_vals = np.full_like(pv, np.nan, dtype=float)
w_vals = np.full_like(pv, np.nan, dtype=float)

for i in range(len(pv)):
    if hiz[i] == 0:
        u_vals[i] = pv[i]
    elif hiz[i] == 1:
        v_vals[i] = pv[i]
    elif hiz[i] == 2:
        w_vals[i] = pv[i]

# ===================== BREAK BETWEEN FLOATING PHASE CHANGES =====================
def break_on_phase_change(time, values, hiz):
    t_out=[]
    v_out=[]
    last = hiz[0]

    for ti,vi,p in zip(time,values,hiz):
        if p != last:
            t_out.append(np.nan)
            v_out.append(np.nan)
        t_out.append(ti)
        v_out.append(vi)
        last = p

    return np.array(t_out), np.array(v_out)

t_u, u_plot = break_on_phase_change(t, u_vals, hiz)
t_v, v_plot = break_on_phase_change(t, v_vals, hiz)
t_w, w_plot = break_on_phase_change(t, w_vals, hiz)

t_u_full = t_u.copy()
u_full = u_plot.copy()

t_v_full = t_v.copy()
v_full = v_plot.copy()

t_w_full = t_w.copy()
w_full = w_plot.copy()

t_neutral_full = t.copy()
neutral_full = neutral.copy()

t_duty_full = t.copy()
duty_full = duty_pct.copy()

# ===================== LOSS MARKERS =====================
loss_times=[]
for i in range(1,len(sc)):
    diff = sc[i] - sc[i-1]
    if diff > 1:
        loss_times.append(t[i])

# ===================== PHASE ERROR ANALYSIS =====================
ZC_EXPECT_UP = {0, 2, 4}
BEMF_MIN_AMPLITUDE = 30
BLANKING_FRACTION = 0.25

phase_error_times = []
phase_error_values = []
phase_error_rpms = []

analysis_mask = (cm == 2) | (cm == 3)
t_a = t[analysis_mask]
ph_a = ph[analysis_mask]
pv_a = pv[analysis_mask]
nv_a = neutral[analysis_mask]
cm_a = cm[analysis_mask]

step_changes_a = np.where(np.diff(ph_a) != 0)[0]

log_header(Level.INFO, "PHASE ERROR ANALYSIS")
log_stat("Steps to analyse:", len(step_changes_a))

skipped_amplitude = 0
skipped_no_zc = 0
skipped_multi_zc = 0
analysed = 0

for si in range(len(step_changes_a) - 1):
    step_start_idx = step_changes_a[si] + 1
    step_end_idx   = step_changes_a[si + 1]

    if step_end_idx <= step_start_idx + 2:
        continue

    step_ph = ph_a[step_start_idx]
    step_t  = t_a[step_start_idx:step_end_idx + 1]
    step_pv = pv_a[step_start_idx:step_end_idx + 1].astype(float)
    step_nv = nv_a[step_start_idx:step_end_idx + 1].astype(float)

    step_start_t  = step_t[0]
    step_end_t    = step_t[-1]
    step_duration = step_end_t - step_start_t

    if step_duration <= 0:
        continue

    step_midpoint_t = step_start_t + step_duration * 0.5

    diff = step_pv - step_nv

    amplitude = np.max(np.abs(diff))
    if amplitude < BEMF_MIN_AMPLITUDE:
        skipped_amplitude += 1
        continue

    blanking_t = step_start_t + step_duration * BLANKING_FRACTION
    valid_mask = step_t > blanking_t
    if not np.any(valid_mask):
        continue

    valid_diff = diff[valid_mask]
    valid_t    = step_t[valid_mask]

    expect_up = step_ph in ZC_EXPECT_UP

    signs = np.sign(valid_diff)
    sign_changes = np.where(np.diff(signs) != 0)[0]

    correct_zc = []
    for sc_idx in sign_changes:
        going_up = valid_diff[sc_idx] < valid_diff[sc_idx + 1]
        if going_up == expect_up:
            correct_zc.append(sc_idx)

    if len(correct_zc) == 0:
        skipped_no_zc += 1
        continue

    if len(correct_zc) > 1:
        skipped_multi_zc += 1

    zc_idx = correct_zc[0]

    t0 = valid_t[zc_idx]
    t1 = valid_t[zc_idx + 1]
    d0 = valid_diff[zc_idx]
    d1 = valid_diff[zc_idx + 1]

    if (d1 - d0) == 0:
        continue

    zc_t = t0 + (t1 - t0) * (-d0 / (d1 - d0))

    error_fraction = (zc_t - step_midpoint_t) / step_duration
    error_fraction = max(-0.5, min(0.5, error_fraction))

    step_rpm = 60_000_000.0 / (step_duration * 6)

    phase_error_times.append(step_midpoint_t)
    phase_error_values.append(error_fraction)
    phase_error_rpms.append(step_rpm)
    analysed += 1

phase_error_times  = np.array(phase_error_times)
phase_error_values = np.array(phase_error_values)
phase_error_rpms   = np.array(phase_error_rpms)

log_stat("Analysed:", analysed)
total_steps = max(len(step_changes_a), 1)
log_stat("Skipped (low BEMF):", skipped_amplitude, Level.WARN if skipped_amplitude > total_steps * 0.5 else Level.INFO)
log_stat("Skipped (no ZC):", skipped_no_zc, Level.WARN if skipped_no_zc > total_steps * 0.2 else Level.INFO)
log_stat("Skipped (multi ZC):", skipped_multi_zc, Level.WARN if skipped_multi_zc > total_steps * 0.1 else Level.INFO)
if len(phase_error_values):
    log_stat("Error range:", f"{phase_error_values.min():.3f} to {phase_error_values.max():.3f}")
    mean_err = phase_error_values.mean()
    log_stat("Mean error:", f"{mean_err:.3f}", Level.ERROR if abs(mean_err) > 0.3 else Level.WARN if abs(mean_err) > 0.15 else Level.INFO)
log_footer(Level.INFO)


def decimate_visible(x, y, xmin, xmax, max_points=4000):
    if len(x) == 0:
        return x, y

    data_min = x[0]
    data_max = x[-1]

    if xmin < data_min:
        xmin = data_min
    if xmax > data_max:
        xmax = data_max

    if xmax <= xmin:
        xmin = data_min
        xmax = data_max

    i0 = np.searchsorted(x, xmin, side="left")
    i1 = np.searchsorted(x, xmax, side="right")

    if i1 <= i0:
        i0 = max(0, min(len(x)-1, i0))
        i1 = min(len(x), i0 + 1)

    xs = x[i0:i1]
    ys = y[i0:i1]

    if len(xs) <= max_points:
        return xs, ys

    step = max(1, len(xs)//max_points)
    return xs[::step], ys[::step]

# ===================== PLOT =====================
from matplotlib.widgets import Button
from matplotlib.collections import LineCollection
from matplotlib.ticker import FuncFormatter

plt.style.use("dark_background")

# ---------- LAYOUT CONSTANTS & PERSISTED CONFIG ----------
EXTRA_AX_HEIGHT_PX = 160
TOP_MARGIN_PX      = 10
BOTTOM_MARGIN_PX   = 55
GAP_PX             = 50

_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".motortrace_config.json")

def _load_config():
    try:
        with open(_CONFIG_PATH) as f:
            return json.load(f)
    except Exception:
        return {}

def _save_config(cfg):
    try:
        with open(_CONFIG_PATH, "w") as f:
            json.dump(cfg, f)
    except Exception:
        pass

_config            = _load_config()
_main_ax_height_px = _config.get("main_ax_height_px", 600)

# ---------- FIGURE SETUP ----------
_initial_fig_h_in = (_main_ax_height_px + TOP_MARGIN_PX + BOTTOM_MARGIN_PX) / 100
fig, (ax, ax_err, ax_valley) = plt.subplots(
    3, 1,
    figsize=(18, _initial_fig_h_in),
    gridspec_kw={'height_ratios': [3, 1, 1], 'hspace': 0.08}
)
ax_err.set_visible(False)
ax_valley.set_visible(False)
fig.patch.set_facecolor("black")
ax.set_facecolor("black")
ax_err.set_facecolor("black")
ax_valley.set_facecolor("black")

UI_LEFT_PX = 200
Y_AXIS_PX  = 80
UI_GAP_PX  = 10

phase_err_visible   = False
adc_latency_visible = False

_resizing   = False  # guard against recursive resize callbacks
_our_resize = False  # set when we trigger the resize, so on_resize ignores it

def _resize_window(w_px, h_px):
    """Resize the figure canvas without repositioning the window."""
    global _our_resize
    _our_resize = True
    fig.set_size_inches(w_px / fig.dpi, h_px / fig.dpi, forward=True)

def _total_fig_h_px():
    """Compute required figure height from the current panel visibility state."""
    n_extra = sum(1 for vis in [phase_err_visible, adc_latency_visible] if vis)
    return (TOP_MARGIN_PX
            + _main_ax_height_px
            + GAP_PX * n_extra
            + EXTRA_AX_HEIGHT_PX * n_extra
            + BOTTOM_MARGIN_PX)

def update_plot_layout(resize_window=False, update_main_height=True):
    """
    Extra panels have a fixed pixel height; the main axis fills the rest.
    If resize_window=True (called when toggling a panel), the window is
    resized to preserve _main_ax_height_px.
    If resize_window=False and update_main_height=True (manual window resize),
    _main_ax_height_px is updated from the current window size.
    If update_main_height=False (our own programmatic resize), _main_ax_height_px
    is left unchanged to prevent drift.
    """
    global _resizing, _main_ax_height_px
    if _resizing:
        return
    _resizing = True
    try:
        fig_w_px = fig.get_figwidth() * fig.dpi

        if resize_window:
            new_h_px = _total_fig_h_px()
            _resize_window(fig_w_px, new_h_px)
            fig_h_px = new_h_px
        else:
            fig_h_px = fig.get_figheight() * fig.dpi
            if update_main_height:
                n_extra  = sum(1 for vis in [phase_err_visible, adc_latency_visible] if vis)
                _main_ax_height_px = max(
                    fig_h_px - TOP_MARGIN_PX - BOTTOM_MARGIN_PX
                    - n_extra * (EXTRA_AX_HEIGHT_PX + GAP_PX),
                    50
                )
                _save_config({"main_ax_height_px": _main_ax_height_px})

        extra_panels = [
            (ax_err,    phase_err_visible),
            (ax_valley, adc_latency_visible),
        ]

        left  = (UI_LEFT_PX + UI_GAP_PX + Y_AXIS_PX) / fig_w_px
        right = 0.995

        # Stack from top downward
        cursor_px = fig_h_px - TOP_MARGIN_PX

        ax.set_position([left,
                         (cursor_px - _main_ax_height_px) / fig_h_px,
                         right - left,
                         _main_ax_height_px / fig_h_px])
        cursor_px -= _main_ax_height_px + GAP_PX

        for a, vis in extra_panels:
            if vis:
                a.set_position([left,
                                (cursor_px - EXTRA_AX_HEIGHT_PX) / fig_h_px,
                                right - left,
                                EXTRA_AX_HEIGHT_PX / fig_h_px])
                cursor_px -= EXTRA_AX_HEIGHT_PX + GAP_PX
            else:
                a.set_position([-10, -10, 0.001, 0.001])
    finally:
        _resizing = False

update_plot_layout()

for a in (ax, ax_err):
    a.tick_params(axis='x', pad=2)
    a.tick_params(axis='y', pad=2)
    a.xaxis.labelpad = 4
    a.yaxis.labelpad = 4
    a.grid(True, which="major", linewidth=0.6, alpha=0.12)
    a.minorticks_on()
    a.grid(True, which="minor", linewidth=0.4, alpha=0.05)

# ===================== INITIAL LINES =====================
xmin, xmax = t[0], t[-1]

tu, uu = decimate_visible(t_u_full, u_full, xmin, xmax)
tv, vv = decimate_visible(t_v_full, v_full, xmin, xmax)
tw, ww = decimate_visible(t_w_full, w_full, xmin, xmax)
tn, vn = decimate_visible(t_neutral_full, neutral_full, xmin, xmax)

line_u, = ax.plot(tu, uu, color="#cc4444", label="Phase U", linewidth=1.2)
line_v, = ax.plot(tv, vv, color="#44cc44", label="Phase V", linewidth=1.2)
line_w, = ax.plot(tw, ww, color="#4488ff", label="Phase W", linewidth=1.2)
line_n, = ax.plot(tn, vn, color="#888888", label="Vbus/2", linewidth=2)

# ===================== DUTY AXIS =====================
ax2 = ax.twinx()
ax2.set_facecolor("none")
DUTY_YMIN = 0
DUTY_YMAX = 100
ax2.set_ylim(DUTY_YMIN, DUTY_YMAX)
ax2.set_autoscale_on(False)
ax2.set_navigate(False)

td, dd = decimate_visible(t_duty_full, duty_full, xmin, xmax)
line_duty, = ax2.plot(td, dd, color="#ffaa00", linewidth=1.4, alpha=0.9, label="Duty %")
ax2.grid(False)
ax2.spines["right"].set_alpha(0.3)
ax2.set_ylabel("Duty %", color="#ffaa00")
ax2.tick_params(axis='y', colors="#ffaa00")

# ===================== RPM AXIS =====================
ax3 = ax.twinx()
ax3.spines.right.set_position(("outward", 60))
ax3.set_facecolor("none")
ax3.set_zorder(10)
ax3.patch.set_visible(False)

tr, rr = decimate_visible(t, rpm_full, xmin, xmax)
line_rpm, = ax3.plot(tr, rr, color="#b39ddb", linewidth=1.4, alpha=0.85, label="RPM")
ax3.set_ylabel("RPM", color="#b39ddb")
ax3.tick_params(axis='y', colors="#b39ddb")
ax3.set_ylim(RPM_YMIN, RPM_YMAX)
ax3.set_autoscale_on(False)
ax3.set_navigate(False)

# ===================== PHASE ERROR PLOT =====================
phase_error_scatter = None
phase_error_trend   = None

TREND_WINDOW = 20

def _rolling_average(times, values, window):
    if len(values) < 2:
        return times.copy(), np.full_like(values, np.nan)
    out = np.full_like(values, np.nan, dtype=float)
    for i in range(window - 1, len(values)):
        out[i] = values[i - window + 1 : i + 1].mean()
    return times.copy(), out

if len(phase_error_times) >= TREND_WINDOW:
    _trend_t_full, _trend_v_full = _rolling_average(
        phase_error_times, phase_error_values, TREND_WINDOW
    )
else:
    _trend_t_full = phase_error_times.copy()
    _trend_v_full = np.full_like(phase_error_values, np.nan)

def render_phase_error():
    global phase_error_scatter, phase_error_trend
    if phase_error_scatter:
        phase_error_scatter.remove()
        phase_error_scatter = None
    if phase_error_trend:
        phase_error_trend.remove()
        phase_error_trend = None

    if len(phase_error_times) == 0:
        return

    xmin_e, xmax_e = ax_err.get_xlim()
    mask = (phase_error_times >= xmin_e) & (phase_error_times <= xmax_e)
    if not np.any(mask):
        return

    te = phase_error_times[mask]
    ve = phase_error_values[mask]
    re = phase_error_rpms[mask]

    colors = np.where(ve > 0, '#ff4444', '#4488ff')

    max_rpm = re.max() if re.max() > 0 else 1
    sizes = 10 + 40 * (re / max_rpm)

    phase_error_scatter = ax_err.scatter(
        te, ve,
        c=colors,
        s=sizes,
        alpha=0.7,
        linewidths=0,
        zorder=5
    )

    trend_mask = (
        (_trend_t_full >= xmin_e) &
        (_trend_t_full <= xmax_e) &
        (~np.isnan(_trend_v_full))
    )
    if np.any(trend_mask):
        phase_error_trend, = ax_err.plot(
            _trend_t_full[trend_mask],
            _trend_v_full[trend_mask],
            color="#ffdd00",
            linewidth=2.0,
            alpha=0.9,
            zorder=6,
            label=f"Trend ({TREND_WINDOW}-step avg)"
        )

ax_err.axhline(0, color="#ffffff", linewidth=0.8, alpha=0.4, linestyle="--")
ax_err.set_ylim(-0.55, 0.55)
ax_err.set_ylabel("Phase Error\n(fraction of step)", color="#aaaaaa", fontsize=8)

# ===================== VALLEY OFFSET PLOT =====================
_valley_t  = np.array(timestamps) if timestamps else np.array([])
_valley_v  = np.array(valley_offsets) if valley_offsets else np.array([])

if len(_valley_v) > 0:
    ax_valley.scatter(_valley_t, _valley_v, s=1, c="#00e5ff", alpha=0.4, linewidths=0)
ax_valley.axhline(0, color="#ffffff", linewidth=0.8, alpha=0.4, linestyle="--")
ax_valley.set_ylabel("Valley offset\n(µs)", color="#aaaaaa", fontsize=8)
ax_valley.tick_params(axis="y", colors="#aaaaaa")
ax_valley.tick_params(axis="x", colors="#aaaaaa")
ax_valley.set_xlabel("Time (µs)", color="#aaaaaa", fontsize=8)
ax_valley.set_xlim(ax.get_xlim())
ax_err.tick_params(axis='y', colors="#aaaaaa")
ax_err.set_yticks([-0.5, -0.25, 0, 0.25, 0.5])
ax_err.set_yticklabels(["-0.5\n(leading)", "-0.25", "0\n(perfect)", "+0.25", "+0.5\n(lagging)"], fontsize=7)

ax_err.axhspan( 0.4,  0.55, color="#ff2222", alpha=0.1)
ax_err.axhspan(-0.55,-0.4,  color="#2222ff", alpha=0.1)

render_phase_error()

# ===================== LOSS MARKERS =====================
loss_segments = [[(lt,0),(lt,1)] for lt in loss_times]
loss_collection = LineCollection(
    loss_segments, colors="#aa3333", linewidths=1, alpha=0.15,
    transform=ax.get_xaxis_transform(), visible=False
)
ax.add_collection(loss_collection)
loss_visible = False

# ===================== ZC MARKERS =====================
zc_segments = [[(zt,0),(zt,1)] for zt in zc_times]
zc_collection = LineCollection(
    zc_segments, colors="#00ffff", linewidths=1.2, alpha=0.8,
    transform=ax.get_xaxis_transform(), label="Zero Cross"
)
ax.add_collection(zc_collection)

# ===================== COMMUTATION PREDICTION =====================
comm_segments = [[(ct,0),(ct,1)] for ct in comm_predict_times]
comm_collection = LineCollection(
    comm_segments, colors="#ff00ff", linewidths=1, alpha=0.35,
    linestyle="dashed", transform=ax.get_xaxis_transform(), visible=False
)
ax.add_collection(comm_collection)
comm_visible = False

# ===================== EVENT MARKERS =====================
# Distinct colours cycled per event type
_EVENT_PALETTE = [
    "#ff6e40",  # orange-red
    "#40c4ff",  # light blue
    "#b9f6ca",  # mint green
    "#ea80fc",  # purple
    "#ffff00",  # yellow
    "#ff4081",  # pink
    "#64ffda",  # teal
    "#ccff90",  # lime
]

def _event_colour(idx: int) -> str:
    return _EVENT_PALETTE[idx % len(_EVENT_PALETTE)]

# Build one LineCollection per event type so each gets its own colour/label.
# We also add matching collections to ax_err so events appear there too.
_event_collections_main = {}   # event_idx → LineCollection on ax
_event_collections_err  = {}   # event_idx → LineCollection on ax_err

def _rebuild_event_collections():
    """(Re)build event marker collections from trace_events.
    Called once after data is loaded, and again if event_names arrives late."""
    # Remove any existing collections
    for col in _event_collections_main.values():
        col.remove()
    for col in _event_collections_err.values():
        col.remove()
    _event_collections_main.clear()
    _event_collections_err.clear()

    if not trace_events:
        return

    # Group timestamps by event index
    by_idx = {}
    for ts, idx, name in trace_events:
        by_idx.setdefault(idx, []).append(ts)

    for idx, times in sorted(by_idx.items()):
        colour   = _event_colour(idx)
        name     = event_names.get(idx, f"Event{idx}")
        segs     = [[(t, 0), (t, 1)] for t in times]
        col_main = LineCollection(
            segs, colors=colour, linewidths=1.2, alpha=0.7,
            linestyle="--", transform=ax.get_xaxis_transform(),
            label=name, zorder=12
        )
        col_err = LineCollection(
            segs, colors=colour, linewidths=1.2, alpha=0.7,
            linestyle="--", transform=ax_err.get_xaxis_transform(),
            zorder=12
        )
        ax.add_collection(col_main)
        ax_err.add_collection(col_err)
        _event_collections_main[idx] = col_main
        _event_collections_err[idx]  = col_err

_rebuild_event_collections()

BUTTON_WIDTH_PX  = 200
BUTTON_HEIGHT_PX = 32
BUTTON_MARGIN_PX = 8
_button_axes     = []  # ordered list of all button axes

def _button_position(index):
    """Compute the [x, y, w, h] figure-fraction position for button at index."""
    fig_w_px   = round(fig.get_figwidth() * fig.dpi)
    fig_h_px   = round(fig.get_figheight() * fig.dpi)
    spacing_px = 6
    x = BUTTON_MARGIN_PX
    y = fig_h_px - (BUTTON_HEIGHT_PX + BUTTON_MARGIN_PX) - index * (BUTTON_HEIGHT_PX + spacing_px)
    return [x / fig_w_px, y / fig_h_px, BUTTON_WIDTH_PX / fig_w_px, BUTTON_HEIGHT_PX / fig_h_px]

def _reposition_button(index):
    try:
        _button_axes[index].set_position(_button_position(index))
    except IndexError:
        pass

def update_button_bar_layout():
    for i in range(len(_button_axes)):
        _reposition_button(i)

def create_fixed_button(fig):
    index = len(_button_axes)
    ax_btn = fig.add_axes([0, 0, 10, 10])
    _button_axes.append(ax_btn)
    original_draw = ax_btn.draw
    def _draw(renderer):
        _reposition_button(index)
        original_draw(renderer)
    ax_btn.draw = _draw
    return ax_btn

def style_button(btn):
    """Apply dark-theme styling to a matplotlib Button."""
    btn.color      = "#333333"
    btn.hovercolor = "#555555"
    btn.label.set_color("white")
    btn.label.set_verticalalignment("center")
    btn.label.set_position((0.5, 0.5))
    btn.ax.set_facecolor("#333333")

button_ax = create_fixed_button(fig)
btn = Button(button_ax, "Show Loss")
btn.label.set_fontsize(9)
style_button(btn)

def toggle_loss(event):
    global loss_visible
    loss_visible = not loss_visible
    loss_collection.set_visible(loss_visible)
    btn.label.set_text("Hide Loss" if loss_visible else "Show Loss")
    fig.canvas.draw_idle()

btn.on_clicked(toggle_loss)

button2_ax = create_fixed_button(fig)
btn_comm = Button(button2_ax, "Show Comm")
btn_comm.label.set_fontsize(9)
style_button(btn_comm)

def toggle_comm(event):
    global comm_visible
    comm_visible = not comm_visible
    comm_collection.set_visible(comm_visible)
    btn_comm.label.set_text("Hide Comm" if comm_visible else "Show Comm")
    fig.canvas.draw_idle()

btn_comm.on_clicked(toggle_comm)

button3_ax = create_fixed_button(fig)
btn_phase_err = Button(button3_ax, "Show Phase Error")
btn_phase_err.label.set_fontsize(9)
style_button(btn_phase_err)

def toggle_phase_err(event):
    global phase_err_visible
    phase_err_visible = not phase_err_visible
    ax_err.set_visible(phase_err_visible)
    btn_phase_err.label.set_text("Hide Phase Error" if phase_err_visible else "Show Phase Error")
    update_plot_layout(resize_window=True)
    fig.canvas.draw_idle()

btn_phase_err.on_clicked(toggle_phase_err)

button4_ax = create_fixed_button(fig)
btn_adc_latency = Button(button4_ax, "Show ADC Latency")
btn_adc_latency.label.set_fontsize(9)
style_button(btn_adc_latency)

def toggle_adc_latency(event):
    global adc_latency_visible
    adc_latency_visible = not adc_latency_visible
    ax_valley.set_visible(adc_latency_visible)
    btn_adc_latency.label.set_text("Hide ADC Latency" if adc_latency_visible else "Show ADC Latency")
    update_plot_layout(resize_window=True)
    fig.canvas.draw_idle()

btn_adc_latency.on_clicked(toggle_adc_latency)

update_button_bar_layout()

# ===================== GENERIC DYNAMIC BAND RENDERER =====================
def render_band(values, colors, y0, y1, alpha=0.3, zorder=0, artist_ref=None, target_ax=None):
    if target_ax is None:
        target_ax = ax
    xmin, xmax = target_ax.get_xlim()
    span = xmax - xmin
    if span <= 0:
        return artist_ref

    band_res = int(fig.get_figwidth() * fig.dpi)
    band_res = max(800, min(4000, band_res))

    band_img = np.zeros((1, band_res, 3))

    i0 = np.searchsorted(t, xmin, side="left")
    i1 = np.searchsorted(t, xmax, side="right")
    if i1 <= i0:
        return artist_ref

    seg_start_t = t[i0]
    current = values[i0]

    def fill_segment(ts, te, val):
        if te <= ts:
            return
        x0 = int((ts - xmin) / span * band_res)
        x1 = int((te - xmin) / span * band_res)
        x0 = max(0, min(band_res-1, x0))
        x1 = max(0, min(band_res,   x1))
        band_img[0, x0:x1] = colors.get(val, (0.2,0.2,0.2))

    for i in range(i0+1, i1):
        if values[i] != current:
            fill_segment(seg_start_t, t[i], current)
            seg_start_t = t[i]
            current = values[i]

    fill_segment(seg_start_t, t[i1-1], current)

    if artist_ref:
        artist_ref.remove()

    new_artist = target_ax.imshow(
        band_img, aspect='auto',
        extent=[xmin, xmax, y0, y1],
        origin='lower', interpolation='nearest',
        alpha=alpha, zorder=zorder
    )

    return new_artist

# ===================== DYNAMIC MOTOR PHASE BACKGROUND =====================
phase_colors = {
    0:(0.40,0.15,0.15),
    1:(0.45,0.30,0.15),
    2:(0.45,0.45,0.15),
    3:(0.15,0.45,0.15),
    4:(0.15,0.20,0.45),
    5:(0.35,0.15,0.45),
}
phase_artist = None

def render_phase():
    global phase_artist
    ymin, ymax = ax.get_ylim()
    phase_artist = render_band(ph, phase_colors, ymin, ymax, alpha=0.35, zorder=-10, artist_ref=phase_artist)

# ===================== CONTROL MODE BAND =====================
mode_colors = {
    0:(0.2,0.2,0.2),
    1:(0.13,0.33,0.53),
    2:(0.53,0.33,0.13),
    3:(0.13,0.53,0.33),
    4:(0.53,0.13,0.53),
    5:(0.53,0.53,0.13),
    6:(0.13,0.53,0.53),
    7:(0.4,0.4,0.4),
}
mode_artist     = None
mode_artist_err = None

def render_mode():
    global mode_artist, mode_artist_err
    ymin, ymax = ax.get_ylim()
    h   = (ymax - ymin) * 0.05
    gap = (ymax - ymin) * 0.01
    mode_artist = render_band(
        cm, mode_colors, ymin + h + gap, ymin + 2*h + gap,
        alpha=0.9, zorder=2, artist_ref=mode_artist
    )
    eymin, eymax = ax_err.get_ylim()
    eh   = (eymax - eymin) * 0.06
    egap = (eymax - eymin) * 0.01
    mode_artist_err = render_band(
        cm, mode_colors, eymin + egap, eymin + eh + egap,
        alpha=0.9, zorder=2, artist_ref=mode_artist_err, target_ax=ax_err
    )

# ===================== LEGENDS =====================
wave_legend = ax.legend(
    handles=[line_u, line_v, line_w, line_n, line_duty, line_rpm],
    loc="upper right", bbox_to_anchor=(1.0, 1.0), fontsize=9, framealpha=0.4
)
ax.add_artist(wave_legend)

from matplotlib.lines import Line2D

# ===================== AXIS BEHAVIOUR =====================
ax.set_xlim(t[0], t[-1])
ax_err.set_xlim(t[0], t[-1])

ymin = np.nanmin(pv)
ymax = np.nanmax(pv)
margin = (ymax - ymin) * 0.08
ymin -= margin
ymax += margin

locked_ymin = None
locked_ymax = None
is_dragging    = False
drag_start_y   = None
drag_start_ylim = None

def set_locked_y(ymin, ymax):
    global locked_ymin, locked_ymax
    locked_ymin = ymin
    locked_ymax = ymax
    ax.set_ylim(ymin, ymax)

def enforce_locked_y(event_ax):
    if locked_ymin is None:
        return
    if event_ax.get_ylim() != (locked_ymin, locked_ymax):
        event_ax.set_ylim(locked_ymin, locked_ymax)

def enforce_locked_duty(event_ax):
    if event_ax.get_ylim() != (DUTY_YMIN, DUTY_YMAX):
        event_ax.set_ylim(DUTY_YMIN, DUTY_YMAX)

def enforce_locked_rpm(event_ax):
    if event_ax.get_ylim() != (RPM_YMIN, RPM_YMAX):
        event_ax.set_ylim(RPM_YMIN, RPM_YMAX)

set_locked_y(ymin, ymax)
ax.callbacks.connect("ylim_changed", enforce_locked_y)
ax2.callbacks.connect("ylim_changed", enforce_locked_duty)
ax3.callbacks.connect("ylim_changed", enforce_locked_rpm)

def on_scroll(event):
    if event.inaxes not in (ax, ax_err):
        return

    base_scale = 1.2
    scale = 1/base_scale if event.button == "up" else base_scale

    if event.key == "alt" and event.inaxes == ax:
        ymin, ymax = ax.get_ylim()
        y = event.ydata
        if y is None:
            return
        height = (ymax - ymin) * scale
        rel = (ymax - y) / (ymax - ymin)
        set_locked_y(y - height * (1 - rel), y + height * rel)
        fig.canvas.draw_idle()
        return

    xmin, xmax = ax.get_xlim()
    x = event.xdata
    if x is None:
        x = (xmin + xmax) / 2

    width = (xmax - xmin) * scale
    rel = (xmax - x) / (xmax - xmin)
    new_xmin = x - width * (1 - rel)
    new_xmax = x + width * rel

    ax.set_xlim(new_xmin, new_xmax)
    ax_err.set_xlim(new_xmin, new_xmax)
    fig.canvas.draw_idle()

fig.canvas.mpl_connect("scroll_event", on_scroll)

def on_release(event):
    global is_dragging
    is_dragging = False


# ===================== TRACE SELECTION & CURSOR MARKER =====================
_selectable_traces = [
    (line_u,    t,  u_vals,   "Phase U",  ax),
    (line_v,    t,  v_vals,   "Phase V",  ax),
    (line_w,    t,  w_vals,   "Phase W",  ax),
    (line_n,    t,  neutral,  "Vbus/2",   ax),
    (line_duty, t,  duty_pct, "Duty %",   ax2),
    (line_rpm,  t,  rpm_full, "RPM",      ax3),
]

_selected_trace = None
_marker_dot     = None
_marker_label   = None
_marker_vline   = None

_original_lw = {line_u: 1.2, line_v: 1.2, line_w: 1.2,
                line_n: 2.0, line_duty: 1.4, line_rpm: 1.4}

def _clear_marker():
    global _marker_dot, _marker_label, _marker_vline
    if _marker_dot:
        _marker_dot.remove()
        _marker_dot = None
    if _marker_label:
        _marker_label.remove()
        _marker_label = None
    if _marker_vline:
        _marker_vline.remove()
        _marker_vline = None

def _deselect_trace():
    global _selected_trace
    if _selected_trace is not None:
        line, _, _, _, _ = _selectable_traces[_selected_trace]
        line.set_linewidth(_original_lw[line])
        line.set_alpha(0.9 if line in (line_duty, line_rpm) else 1.0)
    _selected_trace = None
    _clear_marker()

def _select_trace(idx):
    global _selected_trace
    _deselect_trace()
    _selected_trace = idx
    line, _, _, _, _ = _selectable_traces[idx]
    line.set_linewidth(3.0)
    line.set_alpha(1.0)

def _hit_test_trace(event):
    if event.xdata is None or event.ydata is None:
        return None
    best_idx  = None
    best_dist = float("inf")
    for i, (line, raw_t, raw_y, label, trace_ax) in enumerate(_selectable_traces):
        valid = ~np.isnan(raw_y)
        if not np.any(valid):
            continue
        vt = raw_t[valid]
        vy = raw_y[valid]
        idx = np.searchsorted(vt, event.xdata, side="left")
        idx = max(0, min(len(vt) - 1, idx))
        try:
            x_disp,  y_disp  = trace_ax.transData.transform((vt[idx], vy[idx]))
            xc_disp, yc_disp = trace_ax.transData.transform((event.xdata, event.ydata))
        except Exception:
            continue
        dist = np.hypot(x_disp - xc_disp, y_disp - yc_disp)
        if dist < best_dist:
            best_dist = dist
            best_idx  = i
    return best_idx if best_dist < 20 else None

def _update_marker(x_us):
    global _marker_dot, _marker_label, _marker_vline
    if _selected_trace is None:
        return
    _clear_marker()
    line, raw_t, raw_y, label, trace_ax = _selectable_traces[_selected_trace]
    valid = ~np.isnan(raw_y)
    if not np.any(valid):
        return
    vt = raw_t[valid]
    vy = raw_y[valid]
    idx = np.searchsorted(vt, x_us, side="left")
    idx = max(0, min(len(vt) - 1, idx))
    if 0 < idx < len(vt):
        t0, t1 = vt[idx-1], vt[idx]
        y0, y1 = vy[idx-1], vy[idx]
        frac   = (x_us - t0) / (t1 - t0) if t1 != t0 else 0
        t_val  = x_us
        y_val  = y0 + frac * (y1 - y0)
    else:
        t_val, y_val = vt[idx], vy[idx]
    xmin, xmax = ax.get_xlim()
    span = xmax - xmin
    if span >= 5_000_000:
        t_str = f"{t_val/1e6:.4f}s"
    elif span >= 5_000:
        t_str = f"{t_val/1e3:.2f}ms"
    else:
        t_str = f"{t_val:.1f}µs"
    if label == "Duty %":
        y_str = f"{y_val:.1f}%"
    elif label == "RPM":
        y_str = f"{y_val:.0f} RPM"
    else:
        y_str = f"{y_val:.1f}"
    dot_color = line.get_color()
    _marker_dot = trace_ax.scatter(
        [t_val], [y_val], color=dot_color, s=60, zorder=20,
        linewidths=1.5, edgecolors="white"
    )
    xmin_ax, xmax_ax = ax.get_xlim()
    ha       = "left" if t_val < xmin_ax + (xmax_ax - xmin_ax) * 0.75 else "right"
    x_offset = 8 if ha == "left" else -8
    _marker_label = trace_ax.annotate(
        f"{label}\n{t_str}\n{y_str}",
        xy=(t_val, y_val),
        xytext=(x_offset, 10),
        textcoords="offset points",
        fontsize=8,
        color="white",
        ha=ha,
        va="bottom",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="#222222",
                  edgecolor=dot_color, alpha=0.85),
        zorder=21
    )
    _marker_vline = ax.axvline(
        t_val, color=dot_color, linewidth=0.8,
        alpha=0.5, linestyle=":", zorder=15
    )
    fig.canvas.draw_idle()

def on_press(event):
    global is_dragging, drag_start_y, drag_start_ylim

    if fig.canvas.toolbar is not None and fig.canvas.toolbar.mode != "":
        return

    if event.inaxes == ax and event.button == 1 and event.key == "alt":
        is_dragging     = True
        drag_start_y    = event.ydata
        drag_start_ylim = ax.get_ylim()
        return

    if event.button == 1 and event.key != "alt" and event.inaxes in (ax, ax2, ax3):
        hit = _hit_test_trace(event)
        if hit is not None:
            _select_trace(hit)
            if event.xdata is not None:
                _update_marker(event.xdata)
        else:
            _deselect_trace()
        fig.canvas.draw_idle()

def on_motion_with_marker(event):
    global drag_start_y, drag_start_ylim
    if is_dragging and event.inaxes == ax:
        if drag_start_y is None or event.ydata is None:
            return
        dy = event.ydata - drag_start_y
        ymin0, ymax0 = drag_start_ylim
        set_locked_y(ymin0 - dy, ymax0 - dy)
        fig.canvas.draw_idle()
        return
    if _selected_trace is not None and event.inaxes in (ax, ax2, ax3):
        if event.xdata is not None:
            _update_marker(event.xdata)

fig.canvas.mpl_connect("button_press_event",   on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("motion_notify_event",  on_motion_with_marker)

def on_resize(event):
    global _our_resize
    if _our_resize:
        _our_resize = False  # consume the flag — don't update _main_ax_height_px
        update_button_bar_layout()
        update_plot_layout(resize_window=False, update_main_height=False)
        return
    update_button_bar_layout()
    update_plot_layout(resize_window=False, update_main_height=True)

fig.canvas.mpl_connect("resize_event", on_resize)

# ===================== SMART TIME AXIS =====================
def update_time_axis(target_ax):
    xmin, xmax = target_ax.get_xlim()
    span = xmax - xmin

    if span >= 5_000_000:
        scale = 1e6; unit = "s"
    elif span >= 5_000:
        scale = 1e3; unit = "ms"
    else:
        scale = 1.0; unit = "µs"

    offset    = round(xmin / scale, 3 if unit == "s" else 1 if unit == "ms" else 0)
    offset_us = offset * scale

    def fmt(x, pos):
        return f"{(x - offset_us) / scale:.3g}"

    target_ax.xaxis.set_major_formatter(FuncFormatter(fmt))

    if offset != 0:
        target_ax.set_xlabel(f"Time ({unit} + {offset:g}{unit})")
    else:
        target_ax.set_xlabel(f"Time ({unit})")

update_time_axis(ax)
update_time_axis(ax_err)

def sync_xlim_to_err(event_ax):
    new_xlim = event_ax.get_xlim()
    if ax_err.get_xlim() != new_xlim:
        ax_err.set_xlim(new_xlim)
    update_time_axis(ax_err)
    render_phase_error()

def sync_xlim_to_main(event_ax):
    new_xlim = event_ax.get_xlim()
    if ax.get_xlim() != new_xlim:
        ax.set_xlim(new_xlim)
    update_time_axis(ax)

ax.callbacks.connect("xlim_changed", sync_xlim_to_err)
ax_err.callbacks.connect("xlim_changed", sync_xlim_to_main)
ax.callbacks.connect("xlim_changed", update_time_axis)

def render_all_bands(event=None):
    render_phase()
    render_mode()

def update_visible(event=None):
    ax_valley.set_xlim(ax.get_xlim())
    xmin, xmax = ax.get_xlim()

    t_vis, _ = decimate_visible(t, t, xmin, xmax)
    if len(t_vis) == 0:
        return

    i0 = np.searchsorted(t, t_vis[0],  side="left")
    i1 = np.searchsorted(t, t_vis[-1], side="right")

    tv = t[i0:i1]
    uu = u_vals[i0:i1]
    vv = v_vals[i0:i1]
    ww = w_vals[i0:i1]
    vn = neutral[i0:i1]
    dd = duty_pct[i0:i1]
    rr = rpm_full[i0:i1]

    line_u.set_data(tv, uu)
    line_v.set_data(tv, vv)
    line_w.set_data(tv, ww)
    line_n.set_data(tv, vn)
    line_duty.set_data(tv, dd)
    line_rpm.set_data(tv, rr)

    render_all_bands()
    render_phase_error()

ax.callbacks.connect("xlim_changed", update_visible)
ax_valley.callbacks.connect("xlim_changed", lambda e: ax_valley.set_xlim(ax.get_xlim()) if ax_valley.get_xlim() != ax.get_xlim() else None)

render_all_bands()
render_phase_error()


render_all_bands()
render_phase_error()

def _on_close(event):
    try:
        import re
        cfg = _load_config()
        cfg["main_ax_height_px"] = _main_ax_height_px
        m = re.search(r'([+-]\d+[+-]\d+)$', fig.canvas.manager.window.wm_geometry())
        if m:
            cfg["win_pos"] = m.group(1)
        _save_config(cfg)
    except Exception:
        _save_config({"main_ax_height_px": _main_ax_height_px})

fig.canvas.mpl_connect("close_event", _on_close)

try:
    _win_pos = _config.get("win_pos")
    if _win_pos:
        fig.canvas.manager.window.geometry(_win_pos)
except Exception:
    pass

# --- Dynamic event toggle buttons (created after data loads) ---
_event_visible = {}
_event_buttons = {}

_rebuild_event_collections()

for _evt_idx in sorted(_event_collections_main.keys()):
    _evt_name = event_names.get(_evt_idx, f"Event{_evt_idx}")
    _event_visible[_evt_idx] = True
    _ax_btn = create_fixed_button(fig)
    _btn_e = Button(_ax_btn, f"Hide {_evt_name}")
    _btn_e.label.set_fontsize(9)
    style_button(_btn_e)
    _event_buttons[_evt_idx] = _btn_e

    def _make_toggle(i, b, n):
        def toggle(event):
            _event_visible[i] = not _event_visible[i]
            vis = _event_visible[i]
            _event_collections_main[i].set_visible(vis)
            if i in _event_collections_err:
                _event_collections_err[i].set_visible(vis)
            b.label.set_text(f"Hide {n}" if vis else f"Show {n}")
            fig.canvas.draw_idle()
        return toggle

    _btn_e.on_clicked(_make_toggle(_evt_idx, _btn_e, _evt_name))

update_button_bar_layout()

plt.show()
