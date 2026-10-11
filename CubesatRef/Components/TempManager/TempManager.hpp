// TempManager.hpp

#ifndef tempManager_TempManager_HPP
#define tempManager_TempManager_HPP

#include "CubesatRef/Components/TempManager/TempManagerComponentAc.hpp"

namespace Managers {

class TempManager final : public TempManagerComponentBase {
  public:
    TempManager(const char* const compName);
    ~TempManager();

  private:
    U32 m_simTick = 0;
    U32 m_readCount = 0;
    static constexpr U32 READ_LOG_INTERVAL = 5;

    // state machine action handlers
    void Managers_TempManagerStateMachine_action_doInit(
        SmId smId, Managers_TempManagerStateMachine::Signal signal) override;
    void Managers_TempManagerStateMachine_action_doRead(
        SmId smId, Managers_TempManagerStateMachine::Signal signal) override;
    void Managers_TempManagerStateMachine_action_doFaultRecovery(
        SmId smId, Managers_TempManagerStateMachine::Signal signal) override;
    void Managers_TempManagerStateMachine_action_doSimRead(
        SmId smId, Managers_TempManagerStateMachine::Signal signal) override;

    // command handlers
    void run_handler(FwIndexType portNum, U32 context) override;
    void READ_TEMP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    // reads 2 bytes from tmp102 and converts to degrees c
    Drv::I2cStatus readRawTemp(F32& temperature);
};

}  // namespace Managers

#endif
