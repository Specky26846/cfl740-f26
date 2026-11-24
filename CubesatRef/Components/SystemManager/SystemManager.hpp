// SystemManager.hpp

#ifndef Managers_SystemManager_HPP
#define Managers_SystemManager_HPP

#include "CubesatRef/Components/SystemManager/SystemManagerComponentAc.hpp"

namespace Managers {

class SystemManager final : public SystemManagerComponentBase {
  public:
    SystemManager(const char* const compName);
    ~SystemManager();

  private:
    // state machine action handlers
    void Managers_SystemManagerStateMachine_action_runHealthCheck(
        SmId smId,
        Managers_SystemManagerStateMachine::Signal signal
    ) override;

    void Managers_SystemManagerStateMachine_action_monitorDegraded(
        SmId smId,
        Managers_SystemManagerStateMachine::Signal signal
    ) override;

    void Managers_SystemManagerStateMachine_action_performReboot(
        SmId smId,
        Managers_SystemManagerStateMachine::Signal signal
    ) override;

    void Managers_SystemManagerStateMachine_action_runLanded(
        SmId smId,
        Managers_SystemManagerStateMachine::Signal signal
    ) override;

    // port handlers
    void run_handler(FwIndexType portNum, U32 context) override;
    void tempHealth_handler(FwIndexType portNum, bool healthy) override;
    void gpsHealth_handler(FwIndexType portNum, bool healthy) override;
    void magHealth_handler(FwIndexType portNum, bool healthy) override;
    void imuHealth_handler(FwIndexType portNum, bool healthy) override;

    // imu data port handler - used for landing detection
    void imuData_handler(FwIndexType portNum, const Types::ImuData& data) override;

    // command handlers
    void REPORT_STATUS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void INJECT_COMPONENT_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void ESCALATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_TEMP_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_GPS_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_MAG_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_IMU_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void ARM_FLIGHT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void DISARM_FLIGHT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    U64 m_uptimeSeconds = 0;
    U32 m_totalComponentFaults = 0;
    U32 m_degradedTicks = 0;

    // sends faultCleared signal if no faults remain
    void checkAllClear();

    // per-sensor fault flags
    bool m_tempFaultActive = false;
    bool m_gpsFaultActive = false;
    bool m_magFaultActive = false;
    bool m_imuFaultActive = false;
    bool m_ledToggle = false;
    bool m_rebootIssued = false;
    bool m_flightArmed = false;
    bool m_inFlight = false;

    // auto-escalate to reboot after this many ticks in degraded (~500 seconds at 1Hz)
    static constexpr U32 ESCALATION_THRESHOLD = 500;

    // landing detection
    // a still imu reads ~1g total (gravity), not 0 - ~0g would mean free fall
    bool m_imuStill = false;
    bool m_imuFresh = false;
    bool m_landed = false;
    U32 m_stillTicks = 0;
    U32 m_moveTicks = 0;
    static constexpr U64 LANDING_ARM_SECONDS = 3600;  // ignore the first hour of flight
    static constexpr F32 STILL_ACCEL_TOL_G = 0.05f;   // | |a| - 1g | must be under this
    static constexpr F32 STILL_GYRO_TOL_DPS = 2.0f;   // |w| must be under this
    static constexpr U32 MOVE_TICKS_REQUIRED = 30;    // consecutive moving samples required to start still count
    static constexpr U32 STILL_TICKS_REQUIRED = 30;   // consecutive still samples required to detect landing (~30s at 1Hz)
};

}  // namespace Managers

#endif
