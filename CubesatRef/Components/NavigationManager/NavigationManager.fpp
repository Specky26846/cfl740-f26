module Managers {

    # reads neo-m9n gps over i2c using ubx binary protocol
    active component NavigationManager {

        state machine instance navSm: NavigationManagerStateMachine

        # resets gps fix state
        async command GPS_RESET opcode 0

        # switches to sim mode, returns hardcoded coords
        async command ENABLE_SIM opcode 1

        # back to real i2c gps reads
        async command DISABLE_SIM opcode 2

        enum SensorState { INIT, RUNNING, FAULT, SIM }

        # reports state machine transitions
        event StateChange(newState: SensorState) \
            severity activity high \
            format "NavigationManager: {}"

        # gps fix events
        event GpsFixAcquired(
            numSats: U8
        ) severity activity high \
          format "GPS fix acquired with {} satellites"

        event GpsFixLost \
            severity warning high \
            format "GPS fix lost"

        event GpsReading(lat: F64, lon: F64, alt: F32, sats: U8) \
            severity activity low \
            format "GPS reading: lat={f}, lon={f}, alt={f} m, sats={}"

        # gps telemetry
        telemetry Latitude: F64
        telemetry Longitude: F64
        telemetry Altitude: F32
        telemetry GroundSpeed: F32
        telemetry NumSatellites: U8

        # rate group input
        async input port run: Svc.Sched

        # i2c ports for neo-m9n
        output port busWriteRead: Drv.I2cWriteRead
        output port busWrite: Drv.I2c

        # tells system manager if we're healthy or not
        output port healthOut: Managers.ComponentHealth

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
