#include "BLDC/Strategies/MotorControlStrategy.hpp"
#include "BLDC/Types.hpp"

using namespace bldc;

std::array<std::array<PhaseState, 3>, 6> kClockwisePhaseStateTable{
    std::array<PhaseState, 3>{PhaseState::HighZ, PhaseState::Low, PhaseState::High},
    std::array<PhaseState, 3>{PhaseState::Low, PhaseState::HighZ, PhaseState::High},
    std::array<PhaseState, 3>{PhaseState::Low, PhaseState::High, PhaseState::HighZ},
    std::array<PhaseState, 3>{PhaseState::HighZ, PhaseState::High, PhaseState::Low},
    std::array<PhaseState, 3>{PhaseState::High, PhaseState::HighZ, PhaseState::Low},
    std::array<PhaseState, 3>{PhaseState::High, PhaseState::Low, PhaseState::HighZ},
};

MotorState::MotorState(PhaseAngle phaseAngle, float dutyCycle)
    : _phaseStates(kClockwisePhaseStateTable[static_cast<uint8_t>(phaseAngle)]), _dutyCycles({dutyCycle, dutyCycle, dutyCycle}) {}

MotorState::MotorState(const std::array<PhaseState, 3>& phaseStates, float dutyCycle)
    : _phaseStates(phaseStates), _dutyCycles({dutyCycle, dutyCycle, dutyCycle}) {}

PhaseAngle MotorState::nearestPhaseAngle() const {
    uint8_t low = std::numeric_limits<uint8_t>::max();
    uint8_t highZ = std::numeric_limits<uint8_t>::max();

    for (size_t i = 0; i < 3; ++i) {
        if (_phaseStates[i] == PhaseState::Low) {
            low = i;
        } else if (_phaseStates[i] == PhaseState::HighZ) {
            highZ = i;
        }
    }

    if (highZ == 0) {
        return low == 1 ? PhaseAngle::Degrees0 : PhaseAngle::Degrees180;
    } else if (highZ == 1) {
        return low == 0 ? PhaseAngle::Degrees60 : PhaseAngle::Degrees240;
    } else {
        return low == 0 ? PhaseAngle::Degrees120 : PhaseAngle::Degrees300;
    }
}

MotorPhase MotorState::floatingPhase() const {
    for (size_t i = 0; i < 3; ++i) {
        if (_phaseStates[i] == PhaseState::HighZ) {
            return static_cast<MotorPhase>(i);
        }
    }
    return MotorPhase::U;
}

MotorPhase MotorState::lowPhase() const {
    for (size_t i = 0; i < 3; ++i) {
        if (_phaseStates[i] == PhaseState::Low) {
            return static_cast<MotorPhase>(i);
        }
    }
    return MotorPhase::U;
}

MotorPhase MotorState::highPhase() const {
    for (size_t i = 0; i < 3; ++i) {
        if (_phaseStates[i] == PhaseState::High) {
            return static_cast<MotorPhase>(i);
        }
    }
    return MotorPhase::U;
}

bool bldc::operator==(const MotorState& a, const MotorState& b) {
    for (size_t i = 0; i < 3; ++i) {
        if (a._phaseStates[i] != b._phaseStates[i] || a._dutyCycles[i] != b._dutyCycles[i]) {
            return false;
        }
    }
    return true;
}

bool bldc::operator!=(const MotorState& a, const MotorState& b) {
    return !operator==(a, b);
}
