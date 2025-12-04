#pragma once

#include <cstdint>

namespace ezdc {
    enum class Command : uint8_t {
        Ping = 0x00,
        ReadRegister = 0x01,
        WriteRegister = 0x02,
        ReadInputPins = 0x03,
        RunPIDTune = 0x04,

        WriteFirmware = 0xfd,
        SetBootPartition = 0xfe,
        Restart = 0xff
    };

    enum class Register : uint16_t {
        // Driver Status
        Status = 0x0000,                  // Read only - See StatusBitsetFlags for available flags
        FirmwareVersion = 0x0001,         // Read only - Reports the firmware version
        RPM = 0x0002,                     // Read only - current motor RPM
        ProportionalCoefficient = 0x003,  // float, current proporotional coefficient in PID controller.
        IntegralCoefficient = 0x004,      // float, current integral coefficient in PID controller.
        DifferentialCoeeficient = 0x005,  // float, current differential coefficient in PID controller.

        // Configuration
        ResetRegisters = 0x0100,              // 0x00 - No effect.
                                              // 0x01 - all registers will be erased to zero.
                                              // All other values invalid.
        StandstillDetectionTimeout = 0x0101,  // Time that no zero crossings must be detected for before motor
                                              // is declared to be stopped.  Measured in multiples of 1e-6 seconds.
        Braking = 0x0102,                     // 0x00 - Motor will free wheel to lower speeds when commanded.
                                              // 0x01 - Motor will brake to lower speeds when commanded.
                                              // All other values invalid.

        // Motor Control
        EmergencyStop = 0x0200,  // 0x00 - No effect.
                                 // 0x01 - motor will stop, all control inputs will be ignored until flag is cleared.
                                 // All other values invalid.
        Direction = 0x0201,      // 0x00 - Reverse.
                                 // 0x01 - Forward.
                                 // All other values invalid.
        TargetRPM = 0x0202,      // RPM that the driver will attempt to maintain.
    };

    enum class StatusBitsetFlags : uint8_t {
        DriverError = 0x00,    // Whether the DRV is reporting an error.
        FirmwareError = 0x01,  // Whether the firmware is in an error state.
        Running = 0x02,        // Whether the motor is currently moving (0 - stopped, 1 - moving)
        Direction = 0x03,      // The direction the motor is currently moving in. (0 - backwards, 1 - forwards)
    }
}  // namespace ezdc
