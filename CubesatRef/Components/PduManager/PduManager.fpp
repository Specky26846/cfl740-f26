module Managers {

    # reads the eddy pdu (eps board) over i2c - two ads7828 adcs at 0x48 and 0x49
    active component PduManager {

        state machine instance pduSm: PduManagerStateMachine

        # one-shot read of every pdu channel
        async command READ_PDU

        # switches to sim mode, generates fake pdu data
        async command ENABLE_SIM opcode 1

        # back to real i2c reads
        async command DISABLE_SIM opcode 2

        # battery input (before output regulation)
        telemetry VBattRawVoltage: F32 format "{.3f} V"
        telemetry VBattRawCurrent: F32 format "{.3f} A"

        # 3v3 regulated rail
        telemetry Rail3v3Voltage: F32 format "{.3f} V"
        telemetry Rail3v3Current: F32 format "{.3f} A"

        # 5v0 regulated rail
        telemetry Rail5v0Voltage: F32 format "{.3f} V"
        telemetry Rail5v0Current: F32 format "{.3f} A"

        # switched battery output
        telemetry VBattVoltage: F32 format "{.3f} V"
        telemetry VBattCurrent: F32 format "{.3f} A"

        # lm20 temps on the regulators, degrees c
        telemetry Reg3v3Temp: F32 format "{.2f} C"
        telemetry Reg5v0Temp: F32 format "{.2f} C"

        # rate group input
        async input port run: Svc.Sched

        # i2c port for the ads7828s
        output port busWriteRead: Drv.I2cWriteRead
        output port busWrite: Drv.I2c

        # tells system manager if we're healthy or not
        output port healthOut: Managers.ComponentHealth

        enum SensorState { INIT, RUNNING, FAULT, SIM }

        # reports state machine transitions
        event StateChange(newState: SensorState) \
            severity activity high \
            format "PduManager: {}"

        event PduReading(vbatt: F32, i3v3: F32, i5v0: F32) \
            severity activity low \
            format "PDU reading: VBATT {f} V, I_3V3 {f} A, I_5V0 {f} A"

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
