module Managers {

    # reads icm-20649 imu (accel + gyro) over i2c
    active component IMUManager {

        state machine instance imuSm: IMUManagerStateMachine

        # triggers imu calibration routine
        async command CALIBRATE_IMU opcode 0

        # switches to sim mode, generates fake imu data
        async command ENABLE_SIM opcode 1

        # back to real i2c reads
        async command DISABLE_SIM opcode 2

        enum SensorState { INIT, RUNNING, FAULT, SIM }

        # reports state machine transitions
        event StateChange(newState: SensorState) \
            severity activity high \
            format "IMUManager: {}"

        event ImuCalibrationStarted \
            severity activity high \
            format "IMU calibration started"

        # accel in m/s^2
        telemetry AccelX: F32
        telemetry AccelY: F32
        telemetry AccelZ: F32

        # gyro in deg/s
        telemetry GyroX: F32
        telemetry GyroY: F32
        telemetry GyroZ: F32

        # imu die temp in degrees c
        telemetry ImuTemp: F32

        # rate group input
        async input port run: Svc.Sched

        # tells system manager if we're healthy or not
        output port healthOut: Managers.ComponentHealth

        # i2c ports for icm-20649
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
