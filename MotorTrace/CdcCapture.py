"""
cdc_capture.py — USB-CDC trace capture with OpenOCD reset control

The run loop continuously reads from the serial port. Trace frames are
collected into a sample buffer; response frames are dispatched to queued
continuation handlers in FIFO order. This means we never stop consuming
data, so the firmware never blocks on a full USB TX buffer.

Usage from TraceADC.py:

    from cdc_capture import CdcCapture
    cap = CdcCapture(port="/dev/ttyACM0", openocd="openocd",
                     board_cfg="board/esp32s3-builtin.cfg")
    raw_data = cap.capture(duration=12)
    cap.close()
    decode_binary_trace(raw_data)

Or standalone:

    python cdc_capture.py --port /dev/ttyACM0 --duration 5 --output trace.bin
"""

import os
import sys
import time
import struct
import signal
import socket
import argparse
import subprocess
import threading
import collections

import serial

from Log import *

# ======================== Protocol Constants ========================

TRACE_MAGIC     = bytes([0xA5, 0x5A])
COMMAND_MAGIC   = bytes([0xC0, 0xC0])
RESPONSE_MAGIC  = bytes([0xC1, 0xC1])
LOG_MAGIC       = bytes([0xD0, 0xD0])

TRACE_FRAME_SIZE    = 10  # 2 magic + 7 sample + 1 crc  (default; updated from header)
COMMAND_FRAME_SIZE  = 8   # 2 magic + 1 cmd + 4 param + 1 crc
RESPONSE_FRAME_SIZE = 4   # 2 magic + 1 resp + 1 crc
LOG_HEADER_SIZE     = 4   # 2 magic + 2 length
LOG_MAX_PAYLOAD     = 256

HEADER_MAGIC       = bytes([0xE2, 0xDC])
EVENT_MAGIC        = bytes([0xE1, 0xE1])
EVENT_FRAME_SIZE   = 8       # 2 magic + 4 ts + 1 idx + 1 crc

SAMPLE_SIZE = 7

# DebugCommand enum
CMD_BEGIN_TRACE = 0x00
CMD_END_TRACE   = 0x01
CMD_RUN_MOTOR   = 0x02
CMD_STOP_MOTOR  = 0x03

# CommandResponse enum
RESP_OKAY          = 0x00
RESP_INVALID_CMD   = 0x01
RESP_INVALID_STATE = 0x02

RESPONSE_NAMES = {
    RESP_OKAY:          "Okay",
    RESP_INVALID_CMD:   "InvalidCommand",
    RESP_INVALID_STATE: "InvalidState",
}

# ======================== CRC8 Dallas ========================

from Utilities import crc8_dallas


# ======================== OpenOCD Helper ========================

class OpenOCDController:
    """Manages an OpenOCD process and TCL connection for target reset control."""

    TCL_PORT = 6666
    TERMINATOR = b'\x1a'

    def __init__(self, openocd_bin="openocd", board_cfg="board/esp32s3-builtin.cfg"):
        self._proc = None
        self._sock = None
        self._log_buffer = collections.deque(maxlen=500)
        self._log_thread_out = None
        self._log_thread_err = None
        self._log_running = False
        self._openocd_bin = openocd_bin
        self._board_cfg = board_cfg

    def _start_log_reader(self):
        self._log_running = True

        def reader(pipe):
            while self._log_running:
                line = pipe.readline()
                if not line:
                    break
                self._log_buffer.append(line.rstrip())

        self._log_thread_out = threading.Thread(target=reader, args=(self._proc.stdout,), daemon=True)
        self._log_thread_err = threading.Thread(target=reader, args=(self._proc.stderr,), daemon=True)
        self._log_thread_out.start()
        self._log_thread_err.start()

    def dump_log(self):
        log_header(Level.INFO, "OPENOCD LOG")
        for line in list(self._log_buffer):
            log_sub(Level.DEBUG, line)
        log_footer(Level.INFO)

    def start(self):
        """Launch OpenOCD and connect to its TCL interface."""

        existing = subprocess.run(["pgrep", "-a", "openocd"], capture_output=True, text=True)
        if existing.stdout.strip():
            log(Level.WARN, "Existing OpenOCD process(es) detected:")
            for line in existing.stdout.strip().splitlines():
                log_sub(Level.WARN, line)
            log(Level.ERROR, "Kill them first (pkill openocd) or they will conflict.")
            sys.exit(1)

        log(Level.INFO, "Starting OpenOCD...")
        self._proc = subprocess.Popen(
            [self._openocd_bin, "-f", self._board_cfg, "-c", "init"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._start_log_reader()

        log(Level.INFO, "Waiting for OpenOCD to initialise...")
        time.sleep(2.0)

        if self._proc.poll() is not None:
            self.dump_log()
            raise RuntimeError("OpenOCD exited during startup")

        log(Level.INFO, "Connecting to OpenOCD TCL interface...")
        self._sock = None
        for attempt in range(10):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect(("localhost", self.TCL_PORT))
                s.settimeout(10.0)
                self._sock = s
                s.settimeout(0.3)
                try:
                    while True:
                        chunk = s.recv(4096)
                        if not chunk:
                            break
                except socket.timeout:
                    pass
                s.settimeout(10.0)
                self._tcl_send("echo ping")
                log(Level.INFO, "TCL connected")
                break
            except Exception as e:
                log(Level.WARN, f"TCL not ready yet ({e}), retrying...")
                self._sock = None
                time.sleep(1.0)

        if self._sock is None:
            self.dump_log()
            raise RuntimeError("Could not connect to OpenOCD TCL (port 6666) after 10 attempts")

    def _tcl_send(self, cmd):
        self._sock.sendall((cmd + "\x1a").encode())
        response = b""
        while True:
            chunk = self._sock.recv(4096)
            response += chunk
            if self.TERMINATOR in response:
                break
        return response.replace(self.TERMINATOR, b"").decode(errors="replace").strip()

    def reset_halt(self):
        log(Level.INFO, "Resetting and halting target...")
        self._tcl_send("reset halt")
        log(Level.INFO, "Target halted")

    def resume(self):
        log(Level.INFO, "Resuming target...")
        self._tcl_send("resume")
        log(Level.INFO, "Target resumed")

    def stop(self):
        log(Level.INFO, "Stopping OpenOCD...")
        self._log_running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        if self._proc and self._proc.poll() is None:
            self._proc.send_signal(signal.SIGINT)
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                log(Level.ERROR, "OpenOCD did not exit cleanly — killing")
                self._proc.kill()
        log(Level.INFO, "OpenOCD terminated")


# ======================== Port Watcher ========================

def wait_for_port(port, timeout=15.0, poll_interval=0.3):
    log(Level.INFO, f"Waiting for {port} to appear...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(port):
            try:
                s = serial.Serial(port, baudrate=115200, timeout=0.5)
                s.close()
                log(Level.INFO, f"{port} is ready")
                return True
            except (serial.SerialException, OSError):
                pass
        time.sleep(poll_interval)
    raise TimeoutError(f"{port} did not appear within {timeout}s")


# ======================== Terminal launcher ========================

def _is_vscode():
    """True when running inside a VSCode integrated terminal."""
    return os.environ.get("TERM_PROGRAM") == "vscode"


def _launch_gdb_vscode(gdb_elf):
    """
    Write a temporary GDB attach launch configuration into .vscode/launch.json
    (preserving any existing configurations), then open the Run & Debug panel.
    Returns True on success.
    """
    import json

    # Walk up from CWD to find the workspace root (directory containing .vscode/)
    # Fall back to CWD if not found.
    search = os.path.abspath(os.getcwd())
    workspace_root = search
    for _ in range(8):
        if os.path.isdir(os.path.join(search, ".vscode")):
            workspace_root = search
            break
        parent = os.path.dirname(search)
        if parent == search:
            break
        search = parent

    vscode_dir  = os.path.join(workspace_root, ".vscode")
    launch_path = os.path.join(vscode_dir, "launch.json")
    os.makedirs(vscode_dir, exist_ok=True)

    # Load existing launch.json if present
    if os.path.exists(launch_path):
        with open(launch_path) as f:
            try:
                launch = json.load(f)
            except json.JSONDecodeError:
                launch = {}
    else:
        launch = {}

    launch.setdefault("version", "0.2.0")
    launch.setdefault("configurations", [])

    CONFIG_NAME = "MotorTrace — Attach GDB"

    # Remove any previous auto-generated entry so we don't accumulate duplicates
    launch["configurations"] = [
        c for c in launch["configurations"] if c.get("name") != CONFIG_NAME
    ]

    gdb_elf_abs = os.path.abspath(gdb_elf)

    launch["configurations"].insert(0, {
        "name":    CONFIG_NAME,
        "type":    "gdbtarget",
        "request": "attach",
        "openOcdLaunchCommands": [
            "set ESP_RTOS none",
            "set ESP_GDBSTUB_KEEPALIVE_TIMEOUT 0",
        ],
    })

    with open(launch_path, "w") as f:
        json.dump(launch, f, indent=4)

    log(Level.INFO, f"Written debug config to {launch_path}")
    log(Level.INFO, f"Open Run & Debug (Ctrl+Shift+D) and select '{CONFIG_NAME}'.")
    log(Level.INFO, "VSCode will launch OpenOCD, reset the target, and attach GDB.")
    log(Level.INFO, "Set any breakpoints, then click Continue (F5).")

    return True


def _launch_in_terminal(cmd, title="GDB", gdb_elf=None):
    """
    Launch a GDB session interactively.
    In VSCode: writes a launch.json and opens Run & Debug.
    Otherwise: opens a new terminal window with the GDB command.
    """
    if _is_vscode() and gdb_elf:
        return _launch_gdb_vscode(gdb_elf)

    # Fallback — try common terminal emulators
    candidates = [
        ("gnome-terminal", ["gnome-terminal", "--title={title}", "--", "bash", "-c", "{cmd}; exec bash"]),
        ("xfce4-terminal", ["xfce4-terminal", "--title={title}", "-e", "bash -c '{cmd}; exec bash'"]),
        ("konsole",        ["konsole", "--new-tab", "-p", "tabtitle={title}", "-e", "bash", "-c", "{cmd}; exec bash"]),
        ("xterm",          ["xterm", "-title", "{title}", "-e", "bash", "-c", "{cmd}; exec bash"]),
        ("x-terminal-emulator", ["x-terminal-emulator", "-e", "bash", "-c", "{cmd}; exec bash"]),
    ]

    for binary, template in candidates:
        if subprocess.run(["which", binary], capture_output=True).returncode != 0:
            continue
        args = [a.replace("{title}", title).replace("{cmd}", cmd) for a in template]
        try:
            proc = subprocess.Popen(args)
            log(Level.INFO, f"Launched GDB via {binary}")
            return proc
        except Exception as e:
            log(Level.WARN, f"{binary} failed: {e}")

    return None


# ======================== CdcCapture ========================

class CdcCapture:
    """
    CDC trace capture with a unified run loop.

    All serial reads go through _pump(), which:
      - Collects trace frames into self._samples
      - Dispatches response frames to queued continuations
    """

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=1.0,
                 openocd=None, board_cfg="board/esp32s3-builtin.cfg",
                 gdb_elf=None):
        self._port = port
        self._ocd  = None

        if gdb_elf:
            # VSCode GDB mode — let the gdbtarget extension own OpenOCD entirely.
            # We just write the launch config and wait for the user to attach and
            # resume, then connect to the serial port as normal.
            if not _launch_gdb_vscode(gdb_elf):
                log(Level.ERROR, "Failed to write VSCode launch config.")
                sys.exit(1)
            input("\n      Press Enter here once GDB has attached and resumed the target... ")
            print()
            wait_for_port(port)
            time.sleep(0.5)

        elif openocd:
            ocd = OpenOCDController(openocd_bin=openocd, board_cfg=board_cfg)
            ocd.start()
            ocd.reset_halt()
            ocd.resume()
            ocd.stop()
            wait_for_port(port)
            time.sleep(0.5)

        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        log(Level.INFO, f"CDC connected on {port}")

        # Run loop state
        self._buf               = b""
        self._samples           = bytearray()
        self._events            = []
        self._bad_crc           = 0
        self._header            = None
        self._raw_stream        = bytearray()
        self._response_handlers = collections.deque()

    # -------------------- Header parsing --------------------

    @staticmethod
    def _parse_header_payload(payload: bytes) -> dict:
        """Parse the HEADER frame payload into a dict.
        Format: version(1) sample_bytes(1) pwm_freq(4) event_count(1)
                [idx(1) name(null-terminated)] * event_count
        """
        if len(payload) < 7:
            return {}
        p = 0
        version      = payload[p]; p += 1
        sample_bytes = payload[p]; p += 1
        pwm_freq     = int.from_bytes(payload[p:p+4], "little"); p += 4
        event_count  = payload[p]; p += 1
        event_names  = {}
        for _ in range(event_count):
            if p >= len(payload): break
            idx  = payload[p]; p += 1
            end  = payload.index(0, p)
            name = payload[p:end].decode("utf-8", errors="replace")
            event_names[idx] = name
            p = end + 1
        return {
            "version":      version,
            "sample_bytes": sample_bytes,
            "pwm_freq_hz":  pwm_freq,
            "event_names":  event_names,
        }

    # -------------------- Frame building --------------------

    def _build_command(self, cmd, param=0):
        payload = struct.pack("<BI", cmd, param)
        crc = crc8_dallas(payload)
        return COMMAND_MAGIC + payload + bytes([crc])

    # -------------------- Core run loop --------------------

    def _pump(self):
        """
        Read available data from serial and process all complete frames.
        Trace frames → self._samples
        Response frames → queued handler
        Log frames → printed to stdout
        """
        waiting = self.ser.in_waiting
        chunk = self.ser.read(max(256, waiting))
        if chunk:
            self._buf        += chunk
            self._raw_stream += chunk

        self._parse_buf()

    def _parse_buf(self):
        """Process all complete frames currently in self._buf."""
        buf = bytes(self._buf)
        pos = 0
        n   = len(buf)

        while pos < n - 1:
            b0, b1 = buf[pos], buf[pos + 1]

            # --- Trace frame ---
            if b0 == TRACE_MAGIC[0] and b1 == TRACE_MAGIC[1]:
                if n - pos < self._trace_frame_size:
                    break
                sample_data = buf[pos + 2 : pos + 2 + self._sample_size]
                crc_rx      = buf[pos + 2 + self._sample_size]
                if crc8_dallas(sample_data) == crc_rx:
                    self._samples.extend(sample_data)
                else:
                    self._bad_crc += 1
                pos += self._trace_frame_size
                continue

            # --- Response frame ---
            if b0 == RESPONSE_MAGIC[0] and b1 == RESPONSE_MAGIC[1]:
                if n - pos < RESPONSE_FRAME_SIZE:
                    break
                resp_byte = buf[pos + 2]
                crc_rx    = buf[pos + 3]
                if crc8_dallas(bytes([resp_byte])) == crc_rx:
                    pos += RESPONSE_FRAME_SIZE
                    if self._response_handlers:
                        handler = self._response_handlers.popleft()
                        handler(resp_byte)
                    elif not getattr(self, '_suppress_unexpected_responses', False):
                        log(Level.WARN, f"Unexpected response: {RESPONSE_NAMES.get(resp_byte, hex(resp_byte))}")
                else:
                    pos += 2
                continue

            # --- Log frame: [D0 D0] text[\0] CRC8 ---
            if b0 == LOG_MAGIC[0] and b1 == LOG_MAGIC[1]:
                null_pos = buf.find(0, pos + 2)
                if null_pos == -1:
                    if n - pos > LOG_MAX_PAYLOAD + 3:
                        pos += 2
                    break
                if n <= null_pos + 1:
                    break
                crc_data = buf[pos + 2 : null_pos + 1]
                crc_rx   = buf[null_pos + 1]
                if crc8_dallas(crc_data) == crc_rx:
                    log_str = buf[pos + 2 : null_pos].decode("utf-8", errors="replace").rstrip()
                    if log_str:
                        log_firmware(log_str)
                else:
                    self._bad_crc += 1
                pos = null_pos + 2
                continue

            # --- Header frame: [E2 DC] ---
            if b0 == HEADER_MAGIC[0] and b1 == HEADER_MAGIC[1]:
                MIN_PAYLOAD = 7
                if n - pos < 2 + MIN_PAYLOAD + 1:
                    break
                p = pos + 2
                if n < p + MIN_PAYLOAD:
                    break
                event_count = buf[p + 6]
                p += 7
                ok = True
                for _ in range(event_count):
                    if n < p + 2:
                        ok = False; break
                    p += 1
                    null_pos = buf.find(0, p)
                    if null_pos == -1:
                        ok = False; break
                    p = null_pos + 1
                if not ok or n < p + 1:
                    break
                crc_data = bytes(buf[pos + 2 : p])
                crc_rx   = buf[p]
                if crc8_dallas(crc_data) == crc_rx:
                    self._header = CdcCapture._parse_header_payload(crc_data)
                    sb = self._header.get("sample_bytes")
                    if sb:
                        self._sample_size      = sb
                        self._trace_frame_size = 2 + sb + 1
                    log(Level.INFO,
                        f"Protocol v{self._header.get('version','?')}, "
                        f"{self._header.get('sample_bytes','?')} bytes/sample, "
                        f"PWM {self._header.get('pwm_freq_hz','?')} Hz, "
                        f"{len(self._header.get('event_names', {}))} event type(s)")
                else:
                    self._bad_crc += 1
                pos = p + 1
                continue

            # --- Event frame: [E1 E1] ---
            if b0 == EVENT_MAGIC[0] and b1 == EVENT_MAGIC[1]:
                if n - pos < EVENT_FRAME_SIZE:
                    break
                payload = bytes(buf[pos + 2 : pos + 7])
                crc_rx  = buf[pos + 7]
                if crc8_dallas(payload) == crc_rx:
                    ts_us = int.from_bytes(payload[0:4], "little")
                    idx   = payload[4]
                    self._events.append((ts_us, idx))
                else:
                    self._bad_crc += 1
                pos += EVENT_FRAME_SIZE
                continue

            # --- Out of sync — scan forward to next magic ---
            next_magic = n
            for magic in (TRACE_MAGIC, RESPONSE_MAGIC, LOG_MAGIC, HEADER_MAGIC, EVENT_MAGIC):
                idx = buf.find(magic, pos + 1)
                if idx != -1:
                    next_magic = min(next_magic, idx)
            if next_magic == n:
                pos = n - 1
                break
            pos = next_magic

        self._buf = buf[pos:]

    def send_command(self, cmd, param=0, on_response=None):
        """
        Send a command frame. The on_response callback will be called
        with the response byte when the run loop encounters it.
        """
        if on_response is not None:
            self._response_handlers.append(on_response)

        frame = self._build_command(cmd, param)
        self.ser.write(frame)
        self.ser.flush()

    def send_command_sync(self, cmd, param=0, timeout=5.0):
        """
        Send a command and block until the response arrives,
        while still pumping trace data. Returns the response byte.
        """
        result = [None]

        def handler(resp):
            result[0] = resp

        self.send_command(cmd, param, on_response=handler)

        deadline = time.time() + timeout
        while result[0] is None and time.time() < deadline:
            self._pump()

        if result[0] is None:
            raise TimeoutError(f"No response to command 0x{cmd:02x}")

        return result[0]

    # -------------------- High-level API --------------------

    def capture(self, duration=5):
        """
        Full capture cycle: begin trace, stream for duration, end trace,
        drain remaining data. Returns raw sample bytes.
        """
        self._samples           = bytearray()
        self._events            = []
        self._bad_crc           = 0
        self._raw_stream        = bytearray()
        self._sample_size       = SAMPLE_SIZE
        self._trace_frame_size  = TRACE_FRAME_SIZE

        # Begin trace (synchronous — no trace data yet)
        log(Level.INFO, "Sending BeginTrace...")
        resp = self.send_command_sync(CMD_BEGIN_TRACE)
        if resp != RESP_OKAY:
            raise RuntimeError(f"BeginTrace failed: {RESPONSE_NAMES.get(resp, hex(resp))}")
        log(Level.INFO, "Trace started")

        # Stream for the requested duration
        start = time.time()
        last_progress = 0
        log(Level.INFO, f"Streaming trace for {duration}s...")

        while True:
            self._pump()

            elapsed = time.time() - start

            # Progress update every second
            now_sec = int(elapsed)
            if now_sec > last_progress:
                last_progress = now_sec
                n = len(self._samples) // self._sample_size
                rate = n / elapsed if elapsed > 0 else 0
                log_sub(Level.DEBUG, f"{n} samples, {rate:.0f} samples/s, {elapsed:.1f}/{duration}s")

            if elapsed >= duration:
                break

        # End trace — the run loop keeps pumping while we wait for
        # the response, so the firmware's drain doesn't deadlock
        log(Level.INFO, "Sending EndTrace...")
        resp = self.send_command_sync(CMD_END_TRACE, timeout=10.0)
        if resp != RESP_OKAY:
            log(Level.WARN, f"EndTrace response: {RESPONSE_NAMES.get(resp, hex(resp))}")
        else:
            log(Level.INFO, "EndTrace acknowledged")

        # Drain remaining trace data until the stream goes quiet
        log(Level.INFO, "Draining remaining samples...")
        quiet_start = None
        while True:
            prev_len = len(self._samples)
            self._pump()

            if len(self._samples) > prev_len:
                quiet_start = None  # still receiving
            else:
                if quiet_start is None:
                    quiet_start = time.time()
                elif time.time() - quiet_start > 0.5:
                    break  # 500ms of silence — done

        # Report
        n_samples = len(self._samples) // self._sample_size
        total_elapsed = time.time() - start

        log_header(Level.INFO, "CDC CAPTURE")
        log_stat("Duration:", f"{total_elapsed:.1f}s")
        min_samples  = int(duration * 17_500)
        samples_level = Level.ERROR if n_samples == 0 else \
                        Level.WARN  if n_samples < min_samples else Level.INFO
        log_stat("Samples:", n_samples, samples_level)
        crc_level = Level.ERROR if self._bad_crc > 100 else \
                    Level.WARN  if self._bad_crc > 0   else Level.INFO
        log_stat("Bad CRC:", self._bad_crc, crc_level)
        if total_elapsed > 0:
            data_rate   = len(self._samples) / total_elapsed / 1024
            sample_rate = n_samples / total_elapsed
            rate_level  = Level.ERROR if sample_rate < 10_000 else \
                          Level.WARN  if sample_rate < 17_500 else Level.INFO
            log_stat("Data rate:",   f"{data_rate:.1f} KB/s")
            log_stat("Sample rate:", f"{sample_rate:.0f} Hz", rate_level)
        log_footer(Level.INFO)

        return bytes(self._samples), list(self._events), self._header, bytes(self._raw_stream)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            log(Level.INFO, "CDC port closed")
        if self._ocd is not None:
            self._ocd.stop()
            self._ocd = None

    @staticmethod
    def parse_stream(data: bytes):
        """
        Parse a raw .ezdc wire stream without a serial connection.
        Returns (sample_bytes, events, header) in the same format as capture().
        """
        import collections as _collections
        # Reuse the framing logic by creating a minimal stub instance
        stub = CdcCapture.__new__(CdcCapture)
        stub._buf               = data
        stub._samples           = bytearray()
        stub._events            = []
        stub._bad_crc           = 0
        stub._header            = None
        stub._raw_stream        = bytearray()
        stub._response_handlers = _collections.deque()
        stub._suppress_unexpected_responses = True
        stub._sample_size       = SAMPLE_SIZE
        stub._trace_frame_size  = TRACE_FRAME_SIZE

        # Drain the buffer using the existing frame parser
        stub._parse_buf()

        return bytes(stub._samples), list(stub._events), stub._header


# ======================== Standalone testing ========================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CDC trace capture")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port for USB-CDC")
    parser.add_argument("--duration", type=int, default=5)
    parser.add_argument("--output", default="trace.bin")
    parser.add_argument("--ocd", default=None,
                        help="Path to openocd binary (enables reset-halt control)")
    parser.add_argument("--board-cfg", default="board/esp32s3-builtin.cfg",
                        help="OpenOCD board config file")
    args = parser.parse_args()

    cap = CdcCapture(
        port=args.port,
        openocd=args.ocd,
        board_cfg=args.board_cfg,
    )
    try:
        raw = cap.capture(duration=args.duration)

        with open(args.output, "wb") as f:
            f.write(raw)
        log(Level.INFO, f"Saved {len(raw)} bytes to {args.output}")
    except KeyboardInterrupt:
        log(Level.WARN, "Interrupted")
    finally:
        cap.close()
