module Cfl740_f26 {
    @ Manages radio communication via SPI
    active component RadioManager {

        @ State of the radio
        enum RadioState {
            UNINITIALIZED
            IDLE
            RX
            TX
            ERROR
        }

        # ------ COM Adapter Interface -------

        # Import the Communications adapter interface
        import Svc.Com

        # ------ Scheduling -------

        @ Port receiving calls from the rate group
        async input port run: Svc.Sched drop

        @ Moves frame from dataIn onto our thread
        internal port transmitFrame(data: Fw.Buffer, context: ComCfg.FrameContext)

        # ------ Hardware/Memory -------

        @ SPI communication to the radio
        output port spiWriteRead: Drv.SpiWriteRead

        @ Buffers for received frames
        output port allocate: Fw.BufferGet

        @ Return buffer to buff manager
        output port deallocate: Fw.BufferSend

        # ------ Commands -------

        @ Read one radio register and report as an event
        async command READ_REGISTER(addr: U8)

        @ Write one radio register
        async command WRITE_REGISTER(addr: U8, value: U8)

        @ Reset radio
        async command RESET_RADIO

        # ------ Parameters -------

        @ Carrier freq in Hz
        param FREQUENCY_HZ: U32 default 437000000

        @ TX power dBm
        param TX_POWER_DBM: I8 default 14

        @ LoRa spreading factor
        param SPREADING_FACTOR: U8 default 9

        @ LoRa bandwidth
        param BANDWIDTH: U8 default 7

        @ LoRa coding rate
        param CODING_RATE: U8 default 1

        @ Ticks to wait for TX done before failure
        param TX_TIMEOUT_TICKS: U32 default 10


        # ------ Events -------

        @ Result of READ_REGISTER command
        event RegisterValue(addr: U8, value: U8) \
            severity activity high \
            format "Radio reg 0x{x} = 0x{x}"

        @ Radio is initialized
        event RadioInitialized(chipVersion: U8) \
            severity activity high \
            format "Radio initialized, chip version 0x{x}"

        @ Radio failed init
        event RadioInitFailed(readVersion: U8, expectedVersion: U8) \
            severity warning high \
            format "Radio init failed: read version 0x{x}, expected 0x{x}"

        @ Radio config written to chip
        event ConfigApplied(freqHz: U32, powerDbm: I8, sf: U8, bw: U8, cr: U8) \
            severity activity high \
            format "Radio config: {} Hz, {} dBm, SF{}, BW idx {}, CR idx {}"

        @ Radio reset requested and started
        event RadioResetStarted \
            severity activity high \
            format "Radio reset, re-initializing"

        @ SPI transaction failed
        event SpiError(status: Drv.SpiStatus) \
            severity warning high \
            format "SPI transaction failed: {}" \
            throttle 10

        @ Radio did not report TX done in time
        event TxTimeout \
            severity warning high \
            format "Radio did not report TX done before timeout"

        @ Could not get buffer for a received packet
        event RxBufferAllocFailed(requested: U32) \
            severity warning high \
            format "Could not allocate {} byte RX buffer" \
            throttle 10


        # ------ Telemetry -------

        @ Current radio state
        telemetry State: RadioState update on change

        @ Frames transmitted since boot
        telemetry FramesSent: U32

        @ Frames received since boot
        telemetry FramesReceived: U32

        @ Transmissions failed
        telemetry TxFailures: U32

        @ SPI transactions failed
        telemetry SpiErrors: U32

        @ RSSI of last received packet
        telemetry LastRssi: I16

        @ SNR of last received packet
        telemetry LastSnr: I8


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

    }
}