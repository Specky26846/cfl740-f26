module Managers {

    # every sensor manager follows this same shape: INIT probes real hardware and only
    # falls to FAULT/retry on a genuine i2c error, so behavior on target hardware is the
    # default path, not an opt-in. SIM is reachable only via ENABLE_SIM/DISABLE_SIM commands,
    # for dev builds with no bus wired or for demoing without hardware.
    state machine IMUManagerStateMachine {

        initial enter INIT

        signal tick         # rate group fires this every second
        signal success      # last action worked, move forward
        signal fault        # i2c error, transitions to FAULT
        signal enableSim    # skip hardware, return sinusoidal fake imu data
        signal disableSim   # go back to real reads (re-probes on the way)

        action doInit          # probe the mpu-6050 at ACCEL_XOUT_H to confirm it's there
        action doRead          # read 14 bytes from reg 0x2d, decode big-endian accel/gyro/temp, push telemetry
        action doFaultRecovery # retry the read; signal success if the bus comes back
        action doSimRead       # sinusoidal accel, gyro, and temp each tick

        state INIT {
            on tick do { doInit }   # probe the imu each tick until it responds
            on success enter RUNNING
            on fault enter FAULT
            on enableSim enter SIM
        }

        state RUNNING {
            on tick do { doRead }   # read all 7 imu channels every tick
            on fault enter FAULT    # any i2c failure goes here
            on enableSim enter SIM
        }

        state FAULT {
            on tick do { doFaultRecovery }  # keep retrying each tick until the bus comes back
            on success enter RUNNING
            on enableSim enter SIM          # can escape fault by switching to sim
        }

        state SIM {
            on tick do { doSimRead }        # no hardware needed, fake sinusoidal imu data each tick
            on disableSim enter INIT        # goes through INIT to re-probe real hardware
        }

    }

}
