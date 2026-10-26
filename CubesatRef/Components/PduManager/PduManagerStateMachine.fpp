module Managers {

    # same shape as the other sensor managers (see TempManagerStateMachine.fpp)
    state machine PduManagerStateMachine {

        initial enter INIT

        signal tick         # rate group fires this every tick
        signal success      # last action worked, move forward
        signal fault        # i2c error, transitions to FAULT
        signal enableSim    # skip hardware, return fake pdu data
        signal disableSim   # go back to real reads (re-probes on the way)

        action doInit          # wake both ads7828s (internal ref on) to confirm they're there
        action doRead          # read every pdu channel, convert, push telemetry
        action doFaultRecovery # retry the read; signal success if the bus comes back
        action doSimRead       # plausible fake rail values each tick

        state INIT {
            on tick do { doInit }   # probe the adcs each tick until they respond
            on success enter RUNNING
            on fault enter FAULT
            on enableSim enter SIM
        }

        state RUNNING {
            on tick do { doRead }
            on fault enter FAULT
            on enableSim enter SIM
        }

        state FAULT {
            on tick do { doFaultRecovery }
            on success enter RUNNING
            on enableSim enter SIM
        }

        state SIM {
            on tick do { doSimRead }
            on disableSim enter INIT    # goes through INIT to re-probe real hardware
        }

    }

}
