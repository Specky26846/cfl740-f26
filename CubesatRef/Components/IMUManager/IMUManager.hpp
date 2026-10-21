// IMUManager.hpp

#ifndef Managers_IMUManager_HPP
#define Managers_IMUManager_HPP

#include "CubesatRef/Components/IMUManager/IMUManagerComponentAc.hpp"

namespace Managers {

class IMUManager final : public IMUManagerComponentBase {
  public:
    IMUManager(const char* const compName);
    ~IMUManager();

    void configure(U32 i2cAddress);

  private:
    U32 m_i2cAddress = 0x69;

    // icm-20649 register and read size
    static constexpr U8 REG_BANK_SEL = 0x7F;
    static constexpr U8 PWR_MGMT_1 = 0x06;     // bank 0
    static constexpr U8 PWR_MGMT_1_WAKE = 0x01; // sleep bit cleared, auto-select best clock
    static constexpr U8 ACCEL_XOUT_H = 0x2D;   // bank 0
    static constexpr U8 DATA_SIZE = 14;

    U32 m_simTick = 0;

    // state machine action handlers
    void Managers_IMUManagerStateMachine_action_doInit(
        SmId smId, Managers_IMUManagerStateMachine::Signal signal) override;
    void Managers_IMUManagerStateMachine_action_doRead(
        SmId smId, Managers_IMUManagerStateMachine::Signal signal) override;
    void Managers_IMUManagerStateMachine_action_doFaultRecovery(
        SmId smId, Managers_IMUManagerStateMachine::Signal signal) override;
    void Managers_IMUManagerStateMachine_action_doSimRead(
        SmId smId, Managers_IMUManagerStateMachine::Signal signal) override;

    // command handlers
    void CALIBRATE_IMU_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void run_handler(FwIndexType portNum, U32 context) override;

    // selects bank 0 and clears the sleep bit - chip powers up asleep with all data regs reading 0
    Drv::I2cStatus wakeImu();
    // reads 14 bytes of accel/gyro/temp from icm-20649
    Drv::I2cStatus readImuData(F32& ax, F32& ay, F32& az,
                                F32& gx, F32& gy, F32& gz,
                                F32& temp);
    // generates fake imu data for sim mode
    void simulateImuData(F32& ax, F32& ay, F32& az,
                         F32& gx, F32& gy, F32& gz,
                         F32& temp);
};

}  // namespace Managers

#endif
