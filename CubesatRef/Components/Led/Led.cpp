// Led.cpp

#include "CubesatRef/Components/Led/Led.hpp"

namespace Managers {

Led::Led(const char* const compName) : LedComponentBase(compName) {}

Led::~Led() {}

// command handler - toggle blinking on/off
void Led::BLINKING_ON_OFF_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, Fw::On onOff) {
    this->m_toggleCounter = 0;
    this->m_blinking = onOff;
    this->log_ACTIVITY_HI_SetBlinkingState(onOff);
    this->tlmWrite_BlinkingState(onOff);
    this->tlmWrite_BlinkCommandCount(++this->m_toggleCounter);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// called when blink interval param is updated from gds
void Led::parameterUpdated(FwPrmIdType id) {
    Fw::ParamValid isValid = Fw::ParamValid::INVALID;
    switch (id) {
        case PARAMID_BLINK_INTERVAL: {
            const U32 interval = this->paramGet_BLINK_INTERVAL(isValid);
            FW_ASSERT(isValid == Fw::ParamValid::VALID, static_cast<FwAssertArgType>(isValid));
            this->log_ACTIVITY_HI_BlinkIntervalSet(interval);
            break;
        }
        default:
            FW_ASSERT(0, static_cast<FwAssertArgType>(id));
            break;
    }
}

// rate group tick - toggles led at configured interval
void Led::run_handler(FwIndexType portNum, U32 context) {
    Fw::ParamValid isValid = Fw::ParamValid::INVALID;
    U32 interval = this->paramGet_BLINK_INTERVAL(isValid);
    FW_ASSERT((isValid != Fw::ParamValid::INVALID) && (isValid != Fw::ParamValid::UNINIT),
              static_cast<FwAssertArgType>(isValid));

    if (this->m_blinking && (interval != 0)) {
        if (this->m_toggleCounter == 0) {
            this->m_state = (this->m_state == Fw::On::ON) ? Fw::On::OFF : Fw::On::ON;
            this->m_transitions++;
            this->tlmWrite_LEDTransitions(this->m_transitions);
            if (this->isConnected_gpioSet_OutputPort(0)) {
                this->gpioSet_out(0, (Fw::On::ON == this->m_state) ? Fw::Logic::HIGH : Fw::Logic::LOW);
            }
            this->log_ACTIVITY_LO_LedState(this->m_state);
        }
        this->m_toggleCounter = (this->m_toggleCounter + 1) % interval;
    }
    else {
        if (this->m_state == Fw::On::ON) {
            if (this->isConnected_gpioSet_OutputPort(0)) {
                this->gpioSet_out(0, Fw::Logic::LOW);
            }
            this->m_state = Fw::On::OFF;
            this->log_ACTIVITY_LO_LedState(this->m_state);
        }
    }
}

}  // namespace Managers
