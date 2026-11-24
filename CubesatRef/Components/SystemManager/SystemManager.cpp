// SystemManager.cpp

#include "CubesatRef/Components/SystemManager/SystemManager.hpp"
#include <cmath>
#include <cstdlib>

namespace Managers {

SystemManager::SystemManager(const char* const compName)
    : SystemManagerComponentBase(compName),
      m_uptimeSeconds(0),
      m_totalComponentFaults(0),
      m_degradedTicks(0),
      m_imuFaultActive(false),
      m_tempFaultActive(false),
      m_gpsFaultActive(false),
      m_magFaultActive(false),
      m_ledToggle(false),
      m_rebootIssued(false) {}

SystemManager::~SystemManager() {}

void SystemManager::run_handler(FwIndexType portNum, U32 context) {
    m_uptimeSeconds++;                    // track how long we've been running
    this->systemMgrSm_sendSignal_tick();  // drive the state machine (NOMINAL/DEGRADED/REBOOT action)
    this->dispatchCurrentMessages();      // flush any queued health port messages that came in this tick
}

// state machine actions

// nominal state, emit telemetry and keep the status led solid on
void SystemManager::Managers_SystemManagerStateMachine_action_runHealthCheck(
    SmId smId,
    Managers_SystemManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::systemMgrSm);
    m_degradedTicks = 0;
    this->tlmWrite_SystemState(0); // switch to NOMINAL state
    this->tlmWrite_SystemUptime(m_uptimeSeconds);
    this->tlmWrite_CpuUsage(0.0f);
    this->tlmWrite_MemUsage(0.0f);
    this->tlmWrite_TotalComponentFaults(m_totalComponentFaults);
    this->tlmWrite_DegradedTicks(0);
    this->log_DIAGNOSTIC_HealthCheckComplete();
    if (this->isConnected_statusLedSet_OutputPort(0)) {
        this->statusLedSet_out(0, Fw::Logic::HIGH);
    }

    if (m_imuFresh) {
    m_imuFresh = false;  // we've used this reading, wait for the next one
        if (m_flightArmed && !m_inFlight) {
            // armed, but not flying yet: look for 30 moving samples in a row
            if (m_imuStill) {
                m_moveTicks = 0;  // a still sample breaks the streak
            } else {
                m_moveTicks = m_moveTicks + 1;
            }

            if (m_moveTicks >= MOVE_TICKS_REQUIRED) {
                m_inFlight = true;  // proven we're flying
                m_moveTicks = 0;
                m_stillTicks = 0;
            }

        } else if (m_inFlight) {
            // flying: look for 30 still samples in a row
            if (m_imuStill) {
                m_stillTicks = m_stillTicks + 1;
            } else {
                m_stillTicks = 0;  // a moving sample breaks the streak
            }

            if (m_stillTicks >= STILL_TICKS_REQUIRED) {
                m_landed = true;
                m_moveTicks = 0;
                this->log_ACTIVITY_HI_LandingDetected(m_uptimeSeconds);
                this->systemMgrSm_sendSignal_landed();
            }
        }
    }
    if (m_landed) { this->systemMgrSm_sendSignal_landed(); }  // existing: return to LANDED after a fault
}

// called every tick while in DEGRADED - blinks the led and escalates if fault persists too long
void SystemManager::Managers_SystemManagerStateMachine_action_monitorDegraded(
    SmId smId,
    Managers_SystemManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::systemMgrSm);
    m_degradedTicks++;
    this->tlmWrite_SystemState(1); // switch to DEGRADED state
    this->tlmWrite_SystemUptime(m_uptimeSeconds);
    this->tlmWrite_TotalComponentFaults(m_totalComponentFaults);
    this->tlmWrite_DegradedTicks(m_degradedTicks);

    // flip the led every tick - since this runs at 1Hz that gives a 0.5Hz blink (on one second, off the next)
    m_ledToggle = !m_ledToggle;
    if (this->isConnected_statusLedSet_OutputPort(0)) {
        this->statusLedSet_out(0, m_ledToggle ? Fw::Logic::HIGH : Fw::Logic::LOW);
    }

    // self-send escalate once we've been degraded long enough - ESCALATION_THRESHOLD is 500 ticks (~500s at 1Hz)
    // this sends us to REBOOT where we sit until a human clears it from GDS
    if (m_degradedTicks >= ESCALATION_THRESHOLD) {
       // this->systemMgrSm_sendSignal_escalate();
    }
}

// WATCHDOG
void SystemManager::Managers_SystemManagerStateMachine_action_performReboot(
    SmId smId,
    Managers_SystemManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::systemMgrSm);
    this->tlmWrite_SystemState(2); // switch to REBOOT state
    if (m_rebootIssued) {
        return;  // already requested, wait for the OS to go down
    }
    m_rebootIssued = true;
    this->log_WARNING_HI_SystemRebooting();  
    int rc = std::system("sudo /sbin/reboot");
    if (rc != 0) {
        this->log_ACTIVITY_HI_RebootFailed();
    }
}

// on the ground, emit telemetry and keep the status led solid on
void SystemManager::Managers_SystemManagerStateMachine_action_runLanded(
    SmId smId,
    Managers_SystemManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::systemMgrSm);
    m_degradedTicks = 0;
    this->tlmWrite_SystemState(3); // switch to LANDED state
    this->tlmWrite_SystemUptime(m_uptimeSeconds);
    this->tlmWrite_TotalComponentFaults(m_totalComponentFaults);
    this->tlmWrite_DegradedTicks(0);
    if (this->isConnected_statusLedSet_OutputPort(0)) {
        this->statusLedSet_out(0, Fw::Logic::HIGH);
    }
    if (m_imuFresh) {
        m_imuFresh = false;
        if (m_imuStill) {
            m_moveTicks = 0;
        } else {
            m_moveTicks = m_moveTicks + 1;
        }
        if (m_moveTicks >= MOVE_TICKS_REQUIRED) {
            m_landed = false;  // clear before signalling
            m_moveTicks = 0;
            m_stillTicks = 0;
            this->log_ACTIVITY_HI_LandingCancelled(m_uptimeSeconds);
            this->systemMgrSm_sendSignal_landingCancelled();
        }
    }
}

// landing detection
// after consecutive moving reads for 30 seconds, then check for 30 consecutive still readings to declare landed
// still means total accel ~1g (just gravity) and almost no rotation
void SystemManager::imuData_handler(FwIndexType portNum, const Types::ImuData& data) {
    const Types::GeometricVector3& a = data.get_accel();
    const Types::GeometricVector3& w = data.get_gyro();
    const F32 accelMag = std::sqrt(a.get_x() * a.get_x() + a.get_y() * a.get_y() + a.get_z() * a.get_z());
    const F32 gyroMag = std::sqrt(w.get_x() * w.get_x() + w.get_y() * w.get_y() + w.get_z() * w.get_z());

    m_imuStill = (std::fabs(accelMag - 1.0f) < STILL_ACCEL_TOL_G) && (gyroMag < STILL_GYRO_TOL_DPS);
    m_imuFresh = true;   // a new reading arrived since the last tick
}

// health port handlers
// each sensor calls healthOut_out(0, false) every tick it's in FAULT state
// without the per-sensor flag guard, m_totalComponentFaults would increment every second during a fault
// the guard ensures we only count and signal once when the fault first appears, not on every repeat call

// only send faultCleared when ALL four sensors are back - one recovering doesn't mean we're ok
void SystemManager::checkAllClear() {
    if (!m_imuFaultActive && !m_tempFaultActive &&
        !m_gpsFaultActive && !m_magFaultActive && !m_imuFaultActive) {
        this->systemMgrSm_sendSignal_faultCleared();
    }
}

void SystemManager::imuHealth_handler(FwIndexType portNum, bool healthy) {
    // !m_imuFaultActive guard: IMUManager calls healthOut(false) every tick it's faulting
    // without this check we'd increment m_totalComponentFaults every second and spam the state machine
    if (!healthy && !m_imuFaultActive) {
        m_imuFaultActive = true;
        m_totalComponentFaults++;
        this->log_WARNING_HI_EnteredDegraded(m_totalComponentFaults);
        this->systemMgrSm_sendSignal_imuFault();
    } else if (healthy && m_imuFaultActive) {
        m_imuFaultActive = false;
        checkAllClear();  // check if everyone else is also healthy before returning to NOMINAL
    }
}

void SystemManager::tempHealth_handler(FwIndexType portNum, bool healthy) {
    if (!healthy && !m_tempFaultActive) {
        m_tempFaultActive = true;
        m_totalComponentFaults++;
        this->log_WARNING_HI_EnteredDegraded(m_totalComponentFaults);
        this->systemMgrSm_sendSignal_tempFault();
    } else if (healthy && m_tempFaultActive) {
        m_tempFaultActive = false;
        checkAllClear();
    }
}

void SystemManager::gpsHealth_handler(FwIndexType portNum, bool healthy) {
    if (!healthy && !m_gpsFaultActive) {
        m_gpsFaultActive = true;
        m_totalComponentFaults++;
        this->log_WARNING_HI_EnteredDegraded(m_totalComponentFaults);
        this->systemMgrSm_sendSignal_gpsFault();
    } else if (healthy && m_gpsFaultActive) {
        m_gpsFaultActive = false;
        checkAllClear();
    }
}

void SystemManager::magHealth_handler(FwIndexType portNum, bool healthy) {
    if (!healthy && !m_magFaultActive) {
        m_magFaultActive = true;
        m_totalComponentFaults++;
        this->log_WARNING_HI_EnteredDegraded(m_totalComponentFaults);
        this->systemMgrSm_sendSignal_magFault();
    } else if (healthy && m_magFaultActive) {
        m_magFaultActive = false;
        checkAllClear();
    }
}

// command handlers

void SystemManager::REPORT_STATUS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->tlmWrite_CpuUsage(0.0f);
    this->tlmWrite_MemUsage(0.0f);
    this->tlmWrite_SystemUptime(m_uptimeSeconds);
    this->tlmWrite_TotalComponentFaults(m_totalComponentFaults);
    this->tlmWrite_DegradedTicks(m_degradedTicks);
    this->log_DIAGNOSTIC_HealthCheckComplete();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::INJECT_COMPONENT_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_totalComponentFaults++;
    this->log_WARNING_HI_ComponentFaultDetected(m_totalComponentFaults);
    this->systemMgrSm_sendSignal_imuFault();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::ESCALATE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_WARNING_HI_EscalatedToReboot();
    this->systemMgrSm_sendSignal_escalate();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::CLEAR_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_imuFaultActive = false;
    m_tempFaultActive = false;
    m_gpsFaultActive = false;
    m_magFaultActive = false;
    m_imuFaultActive = false;
    m_degradedTicks = 0;
    this->log_ACTIVITY_HI_RebootComplete();
    this->systemMgrSm_sendSignal_rebootComplete();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::CLEAR_IMU_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_imuFaultActive = false;
    checkAllClear();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::CLEAR_TEMP_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_tempFaultActive = false;
    checkAllClear();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::CLEAR_GPS_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_gpsFaultActive = false;
    checkAllClear();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::CLEAR_MAG_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_magFaultActive = false;
    checkAllClear();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::ARM_FLIGHT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    if (!m_flightArmed) {  // a repeat command must not restart detection mid-flight
        m_flightArmed = true;
        m_inFlight = false; // redeclare in case we were already flying and landed
        m_moveTicks = 0;
        m_stillTicks = 0;
    }
    this->log_ACTIVITY_HI_FlightArmed();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SystemManager::DISARM_FLIGHT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_flightArmed = false;
    m_inFlight = false;
    m_moveTicks = 0;
    m_stillTicks = 0;
    this->log_ACTIVITY_HI_FlightDisarmed();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Managers
