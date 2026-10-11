module Managers {

    # reads tmp102 temperature sensor over i2c
    active component TempManager {

        state machine instance tempSm: TempManagerStateMachine

        # one-shot temp read command
        async command READ_TEMP

        # switches to sim mode, generates fake temp data
        async command ENABLE_SIM opcode 1

        # back to real i2c reads
        async command DISABLE_SIM opcode 2

        # temperature in degrees c
        telemetry Temperature: F32

        # rate group input
        async input port run: Svc.Sched

        # i2c port for tmp102
        output port busWriteRead: Drv.I2cWriteRead
        output port busWrite: Drv.I2c

        # tells system manager if we're healthy or not
        output port healthOut: Managers.ComponentHealth

        enum SensorState { INIT, RUNNING, FAULT, SIM }

        # reports state machine transitions
        event StateChange(newState: SensorState) \
            severity activity high \
            format "TempManager: {}"

        event TempReading(temperature: F32) \
            severity activity low \
            format "Temp reading: {f} C"

        # standard fprime ports
        time get port timeCaller
        command reg port cmdRegOut
        command recv port cmdIn
        command resp port cmdResponseOut
        text event port logTextOut
        event port logOut
        telemetry port tlmOut
        param get port prmGetOut
        param set port prmSetOut

    }
}
