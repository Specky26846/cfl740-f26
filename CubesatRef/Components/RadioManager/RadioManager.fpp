module Managers {
    @ Default parameters and link loss fallbacks
    module RadioDefaults {
        constant FREQUENCY_HZ = 437075000
        constant TX_POWER_DBM = 14
        constant SPREADING_FACTOR = 7
        constant BANDWIDTH = 7
        constant CODING_RATE = 1
    }
    
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

        @ Raw RAP packet to transmit as-is
        async input port rapIn: Fw.Com drop

        @ Moves frame from dataIn onto our thread
        internal port transmitFrame(data: Fw.Buffer, context: ComCfg.FrameContext)

        @ Health check: ping in from Svc.Health
        async input port pingIn: Svc.Ping

        @ Health check: ping response
        output port pingOut: Svc.Ping

        # ------ Hardware/Memory -------

        @ SPI communication to the radio
        output port spiWriteRead: Drv.SpiWriteRead

        @ Buffers for received frames
        output port allocate: Fw.BufferGet

        @ Return buffer to buff manager
        output port deallocate: Fw.BufferSend

        @ Drives radio RST pin (high = run, pulse = reset)
        output port resetPin: Drv.GpioWrite

        # ------ Commands -------

        @ Read one radio register and report as an event
        async command READ_REGISTER(addr: U8)

        @ Write one radio register
        async command WRITE_REGISTER(addr: U8, value: U8)

        @ Reset radio
        async command RESET_RADIO

        # ------ Parameters -------

        @ Carrier freq in Hz
        param FREQUENCY_HZ: U32 default RadioDefaults.FREQUENCY_HZ

        @ TX power dBm
        param TX_POWER_DBM: I8 default RadioDefaults.TX_POWER_DBM

        @ LoRa spreading factor
        param SPREADING_FACTOR: U8 default RadioDefaults.SPREADING_FACTOR

        @ LoRa bandwidth
        param BANDWIDTH: U8 default RadioDefaults.BANDWIDTH

        @ LoRa coding rate
        param CODING_RATE: U8 default RadioDefaults.CODING_RATE

        @ Ticks (0.1 s each at 10 Hz) to wait for TX done before failure
        param TX_TIMEOUT_TICKS: U32 default 100

        @ Minimum ticks (0.1 s each) between the starts of consecutive transmissions (one frame per period)
        param TX_PERIOD_TICKS: U32 default 50

        @ Ticks (0.1 s each) to wait in ERROR before automatically re-initializing the radio
        param ERROR_RETRY_TICKS: U32 default 100

        @ Ticks (0.1 s each) with no uplink before reverting the radio to its default settings (0 = disabled)
        param LINK_LOSS_TICKS: U32 default 6000

        @ Ticks (0.1 s each) between RAP beacons (0 = never send beacons)
        param BEACON_PERIOD_TICKS: U32 default 50

        @ Minimum ticks (0.1 s each) between the end of one transmission and the start of the next (any type)
        param MIN_TX_GAP_TICKS: U32 default 20

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
            format "Radio init failed: read version 0x{x}, expected 0x{x}" \
            throttle 10

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

        @ A packet was received (debug view of its contents)
        event RxPacket(numBytes: U32, rssi: I16, snr: I8, hexData: string size 128) \
            severity diagnostic \
            format "RX {} bytes, RSSI {} dBm, SNR {} dB: {}"

        @ A packet was sent (debug view of its contents)
        event PacketSent(numBytes: U32) \
            severity activity high \
            format "Beacon TX {} bytes sent."

        @ Radio is retrying initialization after an error
        event RadioRecoveryAttempt(attempt: U32) \
            severity warning low \
            format "Radio in ERROR, re-initializing (attempt {})" \
            throttle 10

        @ No uplink for too long, radio reverted to default settings
        event LinkLossFallback(ticksWithoutUplink: U32) \
            severity warning high \
            format "No uplink for {} ticks, reverting radio to default settings"

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

        @ Packets received with a bad CRC
        telemetry RxCrcErrors: U32

        @ Packets received but dropped (bad length, not addressed to us, no buffer, read failure)
        telemetry RxDropped: U32


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