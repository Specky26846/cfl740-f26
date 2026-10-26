// PduManager.hpp

#ifndef pduManager_PduManager_HPP
#define pduManager_PduManager_HPP

#include "CubesatRef/Components/PduManager/PduManagerComponentAc.hpp"

namespace Managers {

class PduManager final : public PduManagerComponentBase {
  public:
    PduManager(const char* const compName);
    ~PduManager();

  private:
    // every converted value from one pass over the board
    struct PduReadings {
        F32 vbattRawV;
        F32 vbattRawI;
        F32 v3v3;
        F32 i3v3;
        F32 v5v0;
        F32 i5v0;
        F32 vbattV;
        F32 vbattI;
        F32 temp3v3Reg;
        F32 temp5v0Reg;
    };

    U32 m_simTick = 0;
    U32 m_readCount = 0;
    static constexpr U32 READ_LOG_INTERVAL = 5;

    // state machine action handlers
    void Managers_PduManagerStateMachine_action_doInit(
        SmId smId, Managers_PduManagerStateMachine::Signal signal) override;
    void Managers_PduManagerStateMachine_action_doRead(
        SmId smId, Managers_PduManagerStateMachine::Signal signal) override;
    void Managers_PduManagerStateMachine_action_doFaultRecovery(
        SmId smId, Managers_PduManagerStateMachine::Signal signal) override;
    void Managers_PduManagerStateMachine_action_doSimRead(
        SmId smId, Managers_PduManagerStateMachine::Signal signal) override;

    // command handlers
    void run_handler(FwIndexType portNum, U32 context) override;
    void READ_PDU_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    // reads every channel on both adcs and converts to engineering units
    Drv::I2cStatus readAll(PduReadings& readings);

    // single-ended read of one eps channel (0-15, the ADC_XX numbering on the schematic)
    Drv::I2cStatus readChannel(U8 epsChannel, F32& voltage);

    void writeTelemetry(const PduReadings& readings);
    void reportHealth(bool healthy);
};

}  // namespace Managers

#endif
