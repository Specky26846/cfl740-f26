module Managers {

    state machine SystemManagerStateMachine {

        initial enter NOMINAL

        signal tick           # rate group fires this every second
        signal sensorFault    # IMUManager reported an i2c failure
        signal tempFault      # TempManager reported an i2c failure
        signal gpsFault       # NavigationManager reported an i2c failure
        signal magFault       # MagnetometerManager reported an i2c failure
        signal faultCleared   # all four health reporters are back to healthy
        signal escalate       # been in DEGRADED too long - promote to REBOOT
        signal rebootComplete # GDS cleared the fault, return to NOMINAL

        action runHealthCheck  # NOMINAL tick: emit telemetry, keep status led solid on
        action monitorDegraded # DEGRADED tick: emit telemetry, blink led, escalate if it's been too long
        action performReboot   # REBOOT tick: emit telemetry, turn led off

        state NOMINAL {
            on tick do { runHealthCheck }   # everything fine, just report and keep the led on
            on sensorFault enter DEGRADED   # imu went down
            on tempFault enter DEGRADED     # temp sensor went down
            on gpsFault enter DEGRADED      # gps went down
            on magFault enter DEGRADED      # magnetometer went down
        }

        state DEGRADED {
            on tick do { monitorDegraded }  # blink the led each tick, escalate after ESCALATION_THRESHOLD ticks
            on faultCleared enter NOMINAL   # all sensors healthy again
            on escalate enter REBOOT        # fault stuck around too long, time to reboot
        }

        state REBOOT {
            on tick do { performReboot }    # led off, keep pushing telemetry while waiting for GDS to clear it
            on rebootComplete enter NOMINAL # GDS sent CLEAR_FAULT, back to normal
        }

    }

}
