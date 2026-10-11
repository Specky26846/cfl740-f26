module Managers {

    # monitors health of all sensor components, manages system state (nominal/degraded/reboot)
    queued component SystemManager {

        state machine instance systemMgrSm: SystemManagerStateMachine

        # rate group tick
        sync input port run: Svc.Sched

        # async so health signals go on SystemManager's queue and return immediately
        # - avoids blocking the sensor component's thread while SystemManager processes the fault
        async input port sensorHealth: Managers.ComponentHealth
        async input port tempHealth: Managers.ComponentHealth
        async input port gpsHealth: Managers.ComponentHealth
        async input port magHealth: Managers.ComponentHealth

        # gpio for status led
        output port statusLedSet: Drv.GpioWrite

        # commands
        async command REPORT_STATUS opcode 0
        async command INJECT_COMPONENT_FAULT opcode 1
        async command CLEAR_FAULT opcode 2
        async command ESCALATE opcode 3
        async command CLEAR_SENSOR_FAULT opcode 4
        async command CLEAR_TEMP_FAULT opcode 5
        async command CLEAR_GPS_FAULT opcode 6
        async command CLEAR_MAG_FAULT opcode 7

        # events
        event HealthCheckComplete \
            severity activity low \
            format "Health check completed"

        event EnteredDegraded(
            faultCount: U32
        ) severity warning high \
          format "System degraded - component fault detected (total faults: {})"

        event EscalatedToReboot \
            severity warning high \
            format "Fault persisted - escalating to REBOOT"

        event ComponentFaultDetected(
            faultCount: U32
        ) severity warning high \
          format "Component fault detected - entering REBOOT mode (total faults: {})"

        event RebootComplete \
            severity activity high \
            format "Reboot complete: returned to NOMINAL"

        # telemetry - 0=nominal, 1=degraded, 2=reboot
        telemetry SystemState: U32
        telemetry CpuUsage: F32
        telemetry MemUsage: F32
        telemetry SystemUptime: U64
        telemetry TotalComponentFaults: U32
        telemetry DegradedTicks: U32

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
