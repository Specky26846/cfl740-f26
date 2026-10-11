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

    // port handlers
    void run_handler(FwIndexType portNum, U32 context) override;
    void sensorHealth_handler(FwIndexType portNum, bool healthy) override;
    void tempHealth_handler(FwIndexType portNum, bool healthy) override;
    void gpsHealth_handler(FwIndexType portNum, bool healthy) override;
    void magHealth_handler(FwIndexType portNum, bool healthy) override;

    // command handlers
    void REPORT_STATUS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void INJECT_COMPONENT_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void ESCALATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_SENSOR_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_TEMP_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_GPS_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void CLEAR_MAG_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    U64 m_uptimeSeconds = 0;
    U32 m_totalComponentFaults = 0;
    U32 m_degradedTicks = 0;

    // sends faultCleared signal if no faults remain
    void checkAllClear();

    // per-sensor fault flags
    bool m_sensorFaultActive = false;
    bool m_tempFaultActive = false;
    bool m_gpsFaultActive = false;
    bool m_magFaultActive = false;
    bool m_ledToggle = false;

    // auto-escalate to reboot after this many ticks in degraded (~500 seconds at 1Hz)
    static constexpr U32 ESCALATION_THRESHOLD = 500;
};

}  // namespace Managers

#endif
