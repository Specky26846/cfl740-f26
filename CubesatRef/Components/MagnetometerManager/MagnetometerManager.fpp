module Managers {

    # reads rm3100 magnetometer over i2c, 3-axis magnetic field in microtesla
    active component MagnetometerManager {

        state machine instance magSm: MagnetometerManagerStateMachine

        # triggers a mag calibration routine
        async command CALIBRATE_MAG opcode 0

        # switches to sim mode, generates fake mag data
        async command ENABLE_SIM opcode 1

        # back to real i2c reads
        async command DISABLE_SIM opcode 2

        enum SensorState { INIT, RUNNING, FAULT, SIM }

        # reports state machine transitions
        event StateChange(newState: SensorState) \
            severity activity high \
            format "MagnetometerManager: {}"

        event MagCalibrationStarted \
            severity activity high \
            format "Magnetometer calibration started"

        event MagReading(mx: F32, my: F32, mz: F32) \
            severity activity low \
            format "Mag reading: X={f} uT, Y={f} uT, Z={f} uT"

        # mag field xyz in microtesla
        telemetry MagX: F32
        telemetry MagY: F32
        telemetry MagZ: F32

        # rate group input
        async input port run: Svc.Sched

        # tells system manager if we're healthy or not
        output port healthOut: Managers.ComponentHealth

        # i2c ports for rm3100
        output port busWriteRead: Drv.I2cWriteRead
        output port busWrite: Drv.I2c

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
