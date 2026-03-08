#include "Tracer.hpp"

#include "BLDC/MotorConfig.hpp"

#include "Timer.hpp"

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"

#include <algorithm>
#include <cstring>
#include <type_traits>

using namespace bldc;
using namespace esp;

#define TRACER 1

namespace bldc {
    static uint8_t crc8_dallas(const uint8_t* data, size_t length) {
        uint8_t crc = 0;
        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x31;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    static constexpr size_t kUSBTransmitBufferSize = 1 << 14;
    static constexpr size_t kUSBReceiveBufferSize = 256;
    static constexpr size_t kTraceFrameSize = 2 + sizeof(TraceSample) + 1;  // 10 bytes
    static constexpr size_t kCommandFrameSize = 8;                          // 2 magic + 1 cmd + 4 param + 1 crc
    static constexpr size_t kResponseFrameSize = 4;                         // 2 magic + 1 resp + 1 crc

    static constexpr size_t kSampleBatchMaxFrameCount = 256;
    static constexpr size_t kSampleBatchBufferSize = kSampleBatchMaxFrameCount * kTraceFrameSize;

    static constexpr const char* kEventNames[] = {"PhaseChangeRequested", "McpwmOutputChanged", "ProcessingCommands",
                                                  "StreamedData",         "ADCTaskLoopBegan",   "ADCTaskLoopEnded"};
    static_assert(sizeof(kEventNames) / sizeof(kEventNames[0]) == static_cast<size_t>(kTraceEventCount), "kEventNames length must match kTraceEventCount");

    void _usbTask(void* arg) {
        Tracer* tracer = static_cast<Tracer*>(arg);

        tracer->_usbTask();
    }
}  // namespace bldc

Tracer* Tracer::sharedTracer() {
    static Tracer tracer;
    return &tracer;
}

Tracer::Tracer() {
#if defined(TRACER) && TRACER
    static bool tracerInitialised = false;
    assert(tracerInitialised == false);
    tracerInitialised = true;

    xTaskCreatePinnedToCore(bldc::_usbTask, "USB CDC (Tracer)", 4096, this, tskIDLE_PRIORITY + 3, &_taskHandle, 1);

    esp_log_set_vprintf(_logVprintf);
#endif
}

int Tracer::_logVprintf(const char* fmt, va_list args) {
    return Tracer::sharedTracer()->_logVprintfShared(fmt, args);
}

int Tracer::_logVprintfShared(const char* fmt, va_list args) {
#if defined(TRACER) && TRACER
    LogFrame log;
    int length = vsnprintf(log.text, kLogMaxPayload, fmt, args);

    if (length <= 0) {
        return length;
    }
    if (length > (int)kLogMaxPayload) {
        length = kLogMaxPayload - 1;
        log.text[kLogMaxPayload - 1] = '\0';
    }

    _logRingbuffer.push(log);

    return length;
#else
    return 0;
#endif
}

void Tracer::_usbTask() {
#if defined(TRACER) && TRACER
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = kUSBTransmitBufferSize,
        .rx_buffer_size = kUSBReceiveBufferSize,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Failed to install USB Serial/JTAG driver: %s", esp_err_to_name(err));
        return;
    }

    while (true) {
        //sendEvent(TraceEvent::ProcessingCommands);
        _processUSBCommands();

        if (_tracing) {
            _streamData();
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        //sendEvent(TraceEvent::StreamedData);
    }
#endif
}

void Tracer::_processUSBCommands() {
#if defined(TRACER) && TRACER
    const int bytesRead = usb_serial_jtag_read_bytes(_receiveBuffer + _receiveBufferIndex, sizeof(_receiveBuffer) - _receiveBufferIndex, 0);

    uint8_t* commandPointer = _receiveBuffer;

    if (bytesRead <= 0) {
        return;
    }

    while (true) {
        const uint8_t remainingBytes = _receiveBuffer + _receiveBufferIndex + bytesRead - commandPointer;
        if (remainingBytes < kCommandLength) {
            _receiveBufferIndex = remainingBytes;
            std::memmove(_receiveBuffer, commandPointer, _receiveBufferIndex);
            return;
        }

        if (memcmp(commandPointer + kCommandMagicOffset, kCommandMagic, sizeof(kCommandMagic) / sizeof(kCommandMagic[0]))) {
            commandPointer++;
            continue;
        }

        const uint8_t receivedCRC = commandPointer[kCommandCRCOffset];
        const uint8_t calculatedCRC = crc8_dallas(commandPointer + 2, 5);
        if (receivedCRC != calculatedCRC) {
            commandPointer += kCommandLength;
            ESP_LOGW(_loggingTag, "Bad command CRC: (got 0x%02x, expected 0x%02x)", receivedCRC, calculatedCRC);
            continue;
        }

        const DebugCommand command = static_cast<DebugCommand>(commandPointer[kCommandCommandOffset]);
        uint32_t parameter = 0;
        std::memcpy(&parameter, commandPointer + kCommandParameterOffset, 4);
        CommandResponse response = _handleCommand(command, parameter);
        if (response != CommandResponse::ResponseAlreadySent) {
            _sendResponse(response);
        }
        commandPointer += kCommandLength;
    }
#endif
}

CommandResponse Tracer::_handleCommand(DebugCommand cmd, uint32_t param) {
    switch (cmd) {
        case DebugCommand::BeginTrace:
            return _beginTrace();
        case DebugCommand::EndTrace:
            return _endTrace();
        case DebugCommand::RunMotor:
        case DebugCommand::StopMotor:
        default:
            return CommandResponse::InvalidCommand;
    }
}

CommandResponse Tracer::_beginTrace() {
    if (_tracing) {
        return CommandResponse::InvalidState;
    }

    _lastTimestamp = Timer::now();
    _sampleCount = 0;

    _traceSampleRingbuffer.clear();
    _eventRingbuffer.clear();
    _logRingbuffer.clear();

    _tracing = true;
    _sendHeader();

    ESP_LOGI(_loggingTag, "Trace started");

    return CommandResponse::Okay;
}

CommandResponse Tracer::_endTrace() {
    if (!_tracing) {
        return CommandResponse::InvalidState;
    }

    _tracing = false;

    // Send Okay immediately so the host doesn't time out waiting while we
    // drain the ring buffer over a potentially congested USB connection.
    _sendResponse(CommandResponse::Okay);

    while (!_traceSampleRingbuffer.empty() || !_eventRingbuffer.empty() || !_logRingbuffer.empty()) {
        _streamData();
    }

    ESP_LOGI(_loggingTag, "Trace ended (%u samples)", _sampleCount);

    return CommandResponse::ResponseAlreadySent;
}

void Tracer::_sendResponse(CommandResponse response) {
    uint8_t frame[kResponseFrameSize];
    frame[kResponseMagicOffset] = kResponseMagic[0];
    frame[kResponseMagicOffset + 1] = kResponseMagic[1];
    frame[kResponseResponseOffset] = static_cast<uint8_t>(response);
    frame[kResponseCRCOffset] = crc8_dallas(frame + 2, 1);
    usb_serial_jtag_write_bytes((const char*)frame, sizeof(frame), pdMS_TO_TICKS(5));
}

void Tracer::_sendHeader() {
#if defined(TRACER) && TRACER
    // Build the variable-length payload, then add the rest around it
    //
    // Payload layout:
    //   version      uint8
    //   sample_bytes uint8
    //   pwm_freq_hz  uint32 LE
    //   event_count  uint8
    //   for each event:
    //     idx        uint8
    //     name       null-terminated UTF-8

    uint8_t frame[512];
    size_t p = 2;

    frame[p++] = kProtocolVersion;
    frame[p++] = static_cast<uint8_t>(sizeof(TraceSample));
    frame[p++] = (kMotorDriveFrequency >> 0) & 0xFF;
    frame[p++] = (kMotorDriveFrequency >> 8) & 0xFF;
    frame[p++] = (kMotorDriveFrequency >> 16) & 0xFF;
    frame[p++] = (kMotorDriveFrequency >> 24) & 0xFF;
    frame[p++] = kTraceEventCount;

    for (uint8_t i = 0; i < kTraceEventCount; ++i) {
        frame[p++] = i;
        const char* name = kEventNames[i];
        const size_t nameLen = strlen(name) + 1;  // include null terminator
        memcpy(frame + p, name, nameLen);
        p += nameLen;
    }

    // Frame: [E2 DC] payload[p] CRC8
    // CRC covers payload(p)
    const size_t frameLen = p + 1;

    frame[0] = kHeaderMagic[0];
    frame[1] = kHeaderMagic[1];
    frame[p] = crc8_dallas(frame + 2, frameLen - 3);

    usb_serial_jtag_write_bytes((const char*)frame, frameLen, pdMS_TO_TICKS(5));

    _startTimestamp = Timer::now();
#endif
}

void Tracer::sendEvent(TraceEvent evt) {
#if defined(TRACER) && TRACER
    if (!_tracing) {
        return;
    }

    const std::chrono::microseconds timestamp = Timer::now();
    const std::chrono::microseconds deltaTime = timestamp - _startTimestamp;

    _eventRingbuffer.emplace(deltaTime, evt);
#endif
}

void Tracer::_streamData() {
#if defined(TRACER) && TRACER
    static uint8_t transmitBuffer[kSampleBatchBufferSize];
    uint8_t* fillLocation = transmitBuffer;

    while (fillLocation - transmitBuffer + kTraceFrameSize <= sizeof(transmitBuffer)) {
        const TraceSample* sample = _traceSampleRingbuffer.peek();
        if (sample == nullptr) {
            break;
        }

        fillLocation[kTraceFrameMagicOffset] = kTraceMagic[0];
        fillLocation[kTraceFrameMagicOffset + 1] = kTraceMagic[1];
        memcpy(fillLocation + kTraceFrameSampleOffset, sample, sizeof(TraceSample));
        fillLocation[kTraceFrameCRCOffset] = crc8_dallas(fillLocation + kTraceFrameSampleOffset, sizeof(TraceSample));

        fillLocation += kTraceFrameLength;

        _traceSampleRingbuffer.pop();
    }

    while (fillLocation - transmitBuffer + kEventFrameLength <= sizeof(transmitBuffer)) {
        const PendingEvent* event = _eventRingbuffer.peek();
        if (event == nullptr) {
            break;
        }

        uint32_t deltaTime = static_cast<uint32_t>(event->deltaTime.count());

        fillLocation[kEventFrameMagicOffset] = kEventMagic[0];
        fillLocation[kEventFrameMagicOffset + 1] = kEventMagic[1];
        fillLocation[kEventFrameTsOffset] = (deltaTime >> 0) & 0xFF;
        fillLocation[kEventFrameTsOffset + 1] = (deltaTime >> 8) & 0xFF;
        fillLocation[kEventFrameTsOffset + 2] = (deltaTime >> 16) & 0xFF;
        fillLocation[kEventFrameTsOffset + 3] = (deltaTime >> 24) & 0xFF;
        fillLocation[kEventFrameIdxOffset] = static_cast<uint8_t>(event->evt);
        fillLocation[kEventFrameCRCOffset] = crc8_dallas(fillLocation + kEventFrameTsOffset, 5);

        fillLocation += kEventFrameLength;

        _eventRingbuffer.pop();
    }

    while (true) {
        const LogFrame* log = _logRingbuffer.peek();
        if (log == nullptr) {
            break;
        }

        size_t logLength = strnlen(log->text, kLogMaxPayload);

        if (fillLocation - transmitBuffer + logLength + 4 > sizeof(transmitBuffer)) {
            break;
        }

        fillLocation[kLogMagicOffset] = kLogMagic[0];
        fillLocation[kLogMagicOffset + 1] = kLogMagic[1];
        memcpy(fillLocation + kLogPayloadOffset, log->text, logLength + 1);
        fillLocation[kLogPayloadOffset + logLength + 1] = crc8_dallas(fillLocation + kLogPayloadOffset, logLength + 1);

        fillLocation += logLength + 4;

        _logRingbuffer.pop();
    }

    if (fillLocation == transmitBuffer) {
        return;
    }

    usb_serial_jtag_write_bytes((const char*)transmitBuffer, fillLocation - transmitBuffer, pdMS_TO_TICKS(5));

#endif
}

void Tracer::commitSample() {
#if defined(TRACER) && TRACER
    if (!_tracing) {
        return;
    }

    const std::chrono::microseconds time = Timer::now();
    const std::chrono::microseconds delta = time - _lastTimestamp;
    _lastTimestamp = time;
    _current.timestampDelta = static_cast<uint8_t>(delta.count());

    _sampleCount++;
    _traceSampleRingbuffer.push(_current);
#endif
}
