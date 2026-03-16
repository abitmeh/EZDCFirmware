#pragma once
#include "BLDC/Types.hpp"
#include "Utilities/MPSCRingbuffer.hpp"
#include "Utilities/SPSCRingbuffer.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <chrono>
#include <cstddef>
#include <cstdint>

using namespace std::chrono_literals;

namespace bldc {
    enum class DebugCommand : uint8_t {
        BeginTrace = 0x0,
        EndTrace = 0x1,
        RunMotor = 0x2,
        StopMotor = 0x3,
    };

    enum class CommandResponse : uint8_t {
        Okay = 0x0,
        InvalidCommand = 0x1,
        InvalidState = 0x2,
        ResponseAlreadySent = 0xFF,  // sentinel — do not transmit
    };

    enum class Error : uint8_t {
        None = 0x0,
        DrvError = 0x1,
        MotorStalled = 0x2,
    };

    enum class TraceEvent : uint8_t {
        PhaseChangeRequested = 0,  // Motor::_currentStep about to change
        McpwmOutputChanged = 1,    // mcpwm_comparator_set_compare_value called
        ProcessingCommands = 2,
        StreamedData = 3,
        ADCTaskLoopBegan = 4,
        ADCTaskLoopEnded = 5,
        StartEnded = 6,
        DragEnded = 7,
    };
    static constexpr uint8_t kTraceEventCount = static_cast<uint8_t>(TraceEvent::ADCTaskLoopEnded) + 1;

    struct __attribute__((packed)) TraceSample {
        uint8_t timestampDelta;
        uint16_t phaseValue : 12;
        uint16_t neutralValue : 12;
        PhaseAngle phase : 3;
        uint8_t controlMode : 5;
        uint8_t dutyCycle;
        uint8_t ticksToNextStep;
        int16_t valleyOffsetUs;
    };

    static constexpr uint8_t kProtocolVersion = 0;

    // CDC protocol framing constants
    // Everything little endian.
    // Trace frame:    [A5 5A] TraceSample[7] CRC8          = 10 bytes
    // Command frame:  [C0 C0] cmd[1] param[4] CRC8         =  8 bytes
    // Response frame: [C1 C1] response[1] CRC8             =  4 bytes
    // Log frame:      [D0 D0] len[2] text[len] CRC8        = variable
    // Header frame:   [E2 DC] len[2] payload[len] CRC8     = variable (sent once on BeginTrace)
    // Event frame:    [E1 E1] ts_us[4] idx[1] CRC8         =  8 bytes
    static constexpr uint8_t kTraceMagic[] = {0xa5, 0x5a};
    static constexpr uint8_t kCommandMagic[] = {0xc0, 0xc0};
    static constexpr uint8_t kResponseMagic[] = {0xc1, 0xc1};
    static constexpr uint8_t kLogMagic[] = {0xd0, 0xd0};
    static constexpr uint8_t kHeaderMagic[] = {0xe2, 0xdc};
    static constexpr uint8_t kEventMagic[] = {0xe1, 0xe1};

    static constexpr size_t kTraceFrameMagicOffset = 0;
    static constexpr size_t kTraceFrameSampleOffset = 2;
    static constexpr size_t kTraceFrameCRCOffset = 2 + sizeof(TraceSample);
    static constexpr size_t kTraceFrameLength = 3 + sizeof(TraceSample);

    static constexpr size_t kCommandMagicOffset = 0;
    static constexpr size_t kCommandCommandOffset = 2;
    static constexpr size_t kCommandParameterOffset = 3;
    static constexpr size_t kCommandCRCOffset = 7;
    static constexpr size_t kCommandLength = 8;

    static constexpr size_t kResponseMagicOffset = 0;
    static constexpr size_t kResponseResponseOffset = 2;
    static constexpr size_t kResponseCRCOffset = 3;
    static constexpr size_t kResponseLength = 4;

    static constexpr size_t kLogMagicOffset = 0;
    static constexpr size_t kLogPayloadOffset = 2;
    static constexpr size_t kLogMaxPayload = 512;

    static constexpr size_t kEventFrameMagicOffset = 0;
    static constexpr size_t kEventFrameTsOffset = 2;
    static constexpr size_t kEventFrameIdxOffset = 6;
    static constexpr size_t kEventFrameCRCOffset = 7;
    static constexpr size_t kEventFrameLength = 8;

    class Tracer {
    public:
        static Tracer* sharedTracer();

        void commitSample();

        TraceSample* currentSample() { return &_current; }

        void sendEvent(TraceEvent event);

    private:
        Tracer();

        static int _logVprintf(const char* fmt, va_list args);
        int _logVprintfShared(const char* fmt, va_list args);

        void _usbTask();
        void _processUSBCommands();
        void _streamData();
        void _sendResponse(CommandResponse response);
        void _sendHeader();
        CommandResponse _handleCommand(DebugCommand cmd, uint32_t param);
        CommandResponse _beginTrace();
        CommandResponse _endTrace();

        volatile bool _tracing = false;

        std::chrono::microseconds _lastTimestamp = 0us;
        std::chrono::microseconds _startTimestamp = 0us;
        uint32_t _sampleCount = 0;

        struct LogFrame {
            char text[kLogMaxPayload];
        };

        struct PendingEvent {
            std::chrono::microseconds deltaTime;
            TraceEvent evt;
        };

        TraceSample _current;
        SPSCRingbuffer<TraceSample, 12> _traceSampleRingbuffer;
        SPSCRingbuffer<PendingEvent, 6> _eventRingbuffer;
        MPSCRingbuffer<LogFrame, 4> _logRingbuffer;

        static constexpr size_t kUSBReceiveProcessingBufferSize = 32;
        uint8_t _receiveBuffer[kUSBReceiveProcessingBufferSize];
        size_t _receiveBufferIndex = 0;

        TaskHandle_t _taskHandle = nullptr;

        static constexpr char _loggingTag[] = "bldc::Tracer";

        friend void _usbTask(void* userInfo);
    };
}  // namespace bldc
