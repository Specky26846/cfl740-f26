module CubesatRef {
    @ Serializes data into RAP packet: inner TAP/Beacon/DAP, header, 2 Fletcher checksums, and tail
    active component PacketBuilder {

        # One async command/port is required for active components
        # This should be overridden by the developers with a useful command/port
        @ TODO
        async command TODO opcode 0

        ##############################################################################
        #### Uncomment the following examples to start customizing your component ####
        ##############################################################################

        

        # @ Example telemetry counter
        # telemetry ExampleCounter: U64

        # @ Example event
        # event ExampleStateEvent(example_state: Fw.On) severity activity high id 0 format "State set to {}"

        # @ Example port: receiving calls from the rate group
        # sync input port run: Svc.Sched

        # @ Example parameter
        # param PARAMETER_NAME: U32

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
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
        #param get port tempIn: CubesatRef.TempManager.tlmOut
        #param get port magIn: CubesatRef.MagnetometerManager.tlmOut

        @ Output port to send the RAP packet to the radio
        output port rapOut: Fw.Com

    }
}