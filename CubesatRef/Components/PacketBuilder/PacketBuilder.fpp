module CubesatRef {
    @ Serializes data into RAP packet: inner TAP/Beacon/DAP, header, 2 Fletcher checksums, and tail
    active component PacketBuilder {

        @ Port for requesting the current time
        time get port timeCaller

        @ Enables command handling
        import Fw.Command

        @ Enables event handling
        import Fw.Event

        @ Enables telemetry channels handling
        import Fw.Channel

        @ Port to return the value of a parameter
        param get port prmGetOut

        @Port to set the value of a parameter
        param set port prmSetOut

        @ Rate Group Tick - build and send one RAP packet per tick
        async input port run: Svc.Sched

        @ Sync latest readings from the managers
            @ async so we use the PacketBuilder's own queue (no mutex needed)
            @ note: if queue is ever full, old sample is dropped
        async input port imuIn: Managers.ImuSample drop
        async input port magIn: Managers.MagSample drop
        async input port tempIn: Managers.TempSample drop
        async input port pduIn: Managers.PduSample drop
        async input port gpsIn: Managers.GpsSample drop

        @ Output port to send the RAP packet to the radio
        output port rapOut: Fw.Com

    }
}