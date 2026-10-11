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
    U32 m_i2cAddress = 0x68;

    // icm-20649 register and read size
    static constexpr U8 ACCEL_XOUT_H = 0x2D;
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
