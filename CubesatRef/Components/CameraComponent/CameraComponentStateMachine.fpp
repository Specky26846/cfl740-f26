module Managers {

    # every sensor manager follows this same shape: INIT probes real hardware and only
    # falls to FAULT/retry on a genuine i2c error, so behavior on target hardware is the
    # default path, not an opt-in. SIM is reachable only via ENABLE_SIM/DISABLE_SIM commands,
    # for dev builds with no bus wired or for demoing without hardware.
    state machine CameraComponentStateMachine {

        initial enter RUNNING

        signal tick         # rate group fires this every second
        signal success      # last action worked, move forward
        signal fault        # i2c error, transitions to FAULT

        action takePhoto          # take a photo with the specified camera(s)
        action doFaultRecovery    # retry the last failed i2c command

        state RUNNING {
            on tick do { takePhoto }  # take a photo every tick
            on fault enter FAULT    # any i2c failure goes here
        }

        state FAULT {
            on tick do { doFaultRecovery }  # keep retrying each tick until the bus comes back
            on success enter RUNNING
        }


    }

}