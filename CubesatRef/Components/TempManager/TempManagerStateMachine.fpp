module Managers {

    # every sensor manager follows this same shape: INIT probes real hardware and only
    # falls to FAULT/retry on a genuine i2c error, so behavior on target hardware is the
    # default path, not an opt-in. SIM is reachable only via ENABLE_SIM/DISABLE_SIM commands,
    # for dev builds with no bus wired or for demoing without hardware.
    state machine TempManagerStateMachine {

        initial enter INIT

        signal tick         # rate group fires this every second
        signal success      # last action worked, move forward
        signal fault        # i2c error, transitions to FAULT
        signal enableSim    # skip hardware, return fake temp data
        signal disableSim   # go back to real reads (re-probes on the way)

        action doInit          # read reg 0x00 on the tmp102 at 0x4a to confirm it's there
        action doRead          # read 2 bytes, decode 12-bit signed, scale to celsius, push telemetry
        action doFaultRecovery # retry the read; signal success if the bus comes back
        action doSimRead       # slow sinusoidal temp wobbling around 25C each tick

        state INIT {
            on tick do { doInit }   # probe the tmp102 each tick until it responds
            on success enter RUNNING
            on fault enter FAULT
            on enableSim enter SIM  # can jump to SIM before getting a real read
        }

        state RUNNING {
            on tick do { doRead }   # grab a fresh temperature every tick
            on fault enter FAULT    # any i2c failure goes here
            on enableSim enter SIM
        }

        state FAULT {
            on tick do { doFaultRecovery }  # keep retrying each tick until the bus comes back
            on success enter RUNNING
            on enableSim enter SIM          # can escape fault by switching to sim
        }

        state SIM {
            on tick do { doSimRead }        # no hardware needed, fake temp data each tick
            on disableSim enter INIT        # goes through INIT to re-probe real hardware
        }

    }

}
