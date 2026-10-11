module Managers {

    @ Radio state enumeration
    enum RadioState {
        IDLE
        RX
        TX
        ERROR
    }

    @ Radio Manager - manages radio communication over SPI
    active component RadioManager {

        @ Get current radio configuration
        async command GET_CONFIG opcode 0

        @ Set radio TX power in dBm
        async command SET_TX_POWER(
            power: I8 @< TX power in dBm
        ) opcode 1

        @ Set radio frequency in MHz
        async command SET_FREQUENCY(
            frequency: U32 @< Frequency in MHz
        ) opcode 2

        @ Set LoRa modulation parameters
        async command SET_LORA_PARAMS(
            spreadingFactor: U8 @< Spreading factor (6-12)
            bandwidth: U8 @< Bandwidth index (0=7.8kHz, 1=10.4kHz, ... 9=500kHz)
            codingRate: U8 @< Coding rate (1=4/5, 2=4/6, 3=4/7, 4=4/8)
        ) opcode 3

        @ Transmit a packet - wrapped as a DAP inside a RAP (see MXL RAP format), so the
        @ real cap is 229 bytes, not 256 - the RAP/DAP header overhead eats the rest of
        @ the 238-byte RAP data section limit.
        async command TRANSMIT_PACKET(
            data: string size 229 @< Packet data to transmit (max 229 bytes - DAP/RAP framing overhead)
        ) opcode 4

        @ Request current telemetry snapshot
        async command GET_TELEMETRY opcode 5

        @ Reset/reboot the radio module
        async command RADIO_RESET opcode 6

        @ Radio configuration updated
        event ConfigUpdated(
            setting: string size 40 @< Setting that was changed
        ) severity activity high \
          format "Radio config updated: {}"

        @ Current radio configuration reported
        event ConfigReport(
            frequency: U32 @< Current frequency in MHz
            txPower: I8 @< Current TX power in dBm
            sf: U8 @< Current spreading factor
        ) severity activity high \
          format "Radio config: freq={}MHz, power={}dBm, SF={}"

        @ A packet was transmitted
        event PacketTransmitted(
            packetSize: U32 @< Size of the packet in bytes
        ) severity activity high \
          format "Radio packet transmitted, {} bytes"

        @ A packet was received from the radio
        event PacketReceived(
            packetSize: U32 @< Size of the received packet in bytes
        ) severity activity high \
          format "Radio packet received, {} bytes"

        @ An error occurred during radio communication
        event RadioError(
            spiStatus: I32 @< SPI status code
        ) severity warning high \
          format "Radio SPI error, status={}"

        @ Radio module reset
        event RadioResetComplete \
            severity activity high \
            format "Radio module reset complete"

        @ Total packets received since boot
        telemetry PacketsReceived: U32

        @ Total packets transmitted since boot
        telemetry PacketsTransmitted: U32

        @ Last received signal strength indicator (dBm)
        telemetry LastRssi: I32

        @ Current radio state
        telemetry RadioStateChannel: RadioState

        @ Current TX power setting (dBm)
        telemetry TxPower: I8

        @ Current frequency (MHz)
        telemetry Frequency: U32

        @ Current spreading factor
        telemetry SpreadingFactor: U8

        @ Current bandwidth index
        telemetry Bandwidth: U8

        @ Current coding rate
        telemetry CodingRate: U8

        @ Port receiving calls from the rate group
        async input port run: Svc.Sched

        @ SPI read/write port for radio communication
        output port spiReadWrite: Drv.SpiReadWrite

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Port to return the value of a parameter
        param get port prmGetOut

        @ Port to set the value of a parameter
        param set port prmSetOut
    }

}
