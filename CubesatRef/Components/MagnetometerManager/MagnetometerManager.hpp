// MagnetometerManager.hpp

#ifndef Managers_MagnetometerManager_HPP
#define Managers_MagnetometerManager_HPP

#include "CubesatRef/Components/MagnetometerManager/MagnetometerManagerComponentAc.hpp"

namespace Managers {

class MagnetometerManager final : public MagnetometerManagerComponentBase {
  public:
    MagnetometerManager(const char* const compName);
    ~MagnetometerManager();

    void configure(U32 i2cAddress);

  private:
    U32 m_i2cAddress = 0x20;

    static constexpr U8 POLL_REG = 0x00;
    static constexpr U8 POLL_XYZ = 0x70;
    static constexpr U8 MX_REG = 0x24;
    static constexpr U8 MAG_DATA_SIZE = 9;
    static constexpr F32 RM3100_SENSITIVITY = 75.0f;

    U32 m_simTick = 0;
    U32 m_readCount = 0;
    bool m_pollIssued = false;
    static constexpr U32 READ_LOG_INTERVAL = 10;

    // state machine action handlers
    void Managers_MagnetometerManagerStateMachine_action_doInit(
        SmId smId, Managers_MagnetometerManagerStateMachine::Signal signal) override;
    void Managers_MagnetometerManagerStateMachine_action_doRead(
        SmId smId, Managers_MagnetometerManagerStateMachine::Signal signal) override;
    void Managers_MagnetometerManagerStateMachine_action_doFaultRecovery(
        SmId smId, Managers_MagnetometerManagerStateMachine::Signal signal) override;
    void Managers_MagnetometerManagerStateMachine_action_doSimRead(
        SmId smId, Managers_MagnetometerManagerStateMachine::Signal signal) override;

    // command handlers
    void CALIBRATE_MAG_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void run_handler(FwIndexType portNum, U32 context) override;

    Drv::I2cStatus triggerMeasurement();
    Drv::I2cStatus readMagData(F32& mx, F32& my, F32& mz);
    void simulateMagData(F32& mx, F32& my, F32& mz);
};

}  // namespace Managers

#endif
