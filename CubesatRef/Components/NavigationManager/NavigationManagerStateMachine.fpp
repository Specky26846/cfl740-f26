module Managers {

    # every sensor manager follows this same shape: INIT probes real hardware and only
    # falls to FAULT/retry on a genuine i2c error, so behavior on target hardware is the
    # default path, not an opt-in. SIM is reachable only via ENABLE_SIM/DISABLE_SIM commands,
    # for dev builds with no bus wired or for demoing without hardware.
    state machine NavigationManagerStateMachine {

        initial enter INIT

        signal tick         # rate group fires this every second
        signal success      # last action worked, move forward
        signal fault        # i2c error or bad checksum, transitions to FAULT
        signal enableSim    # skip hardware, return fake ann arbor coords
        signal disableSim   # go back to real reads (re-probes on the way)

        action doInit          # write to 0xff and confirm the gps acks it
        action doRead          # poll nav-pvt, parse lat/lon/alt/speed/sats, push telemetry
        action doFaultRecovery # retry the full read; signal success if the bus comes back
        action doSimRead       # hardcoded ann arbor coords (42.28, -83.74, 270m)

        state INIT {
            on tick do { doInit }   # probe the gps each tick until it responds
            on success enter RUNNING
            on fault enter FAULT
            on enableSim enter SIM  # can jump to SIM before ever getting a real fix
        }

        state RUNNING {
            on tick do { doRead }   # request nav-pvt and push telemetry every tick
            on fault enter FAULT    # any i2c failure or bad checksum goes here
            on enableSim enter SIM
        }

        state FAULT {
            on tick do { doFaultRecovery }  # keep retrying each tick until the bus comes back
            on success enter RUNNING
            on enableSim enter SIM          # can escape fault by switching to sim
        }

        state SIM {
            on tick do { doSimRead }        # no hardware needed, fake data each tick
            on disableSim enter INIT        # goes through INIT to re-probe real hardware
        }

    }

}
