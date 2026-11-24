module Cfl740_f26 {

    @ Default S-band parameters and link loss fallbacks
    module SbandDefaults {
        constant FREQUENCY_HZ = 2425000000
        constant TX_POWER_DBM = 12
        constant SPREADING_FACTOR = 7
        constant CODING_RATE = 1
    }

    @ SX1280 LoRa bandwidth options
    enum SbandBandwidth {
        BW_203_KHZ
        BW_406_KHZ
        BW_812_KHZ
        BW_1625_KHZ
    }

    @ Bench test transmission modes
    enum SbandTestMode {
        CW            @< Unmodulated carrier (single line on a waterfall)
        LORA_PREAMBLE @< Continuous LoRa preamble (repeating chirps)
    }

    @ Manages the SX1280 S-band radio (LAMBDA80) via SPI
    active component SbandRadioManager {

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

        @ Drives radio NRESET pin (high = run, pulse low = reset)
        output port resetPin: Drv.GpioWrite

        @ Reads radio BUSY pin (must be low before every SPI command)
        output port busyPin: Drv.GpioRead

        # ------ Commands -------

        @ Send GetStatus (0xC0) and report the status byte as an event
        async command GET_STATUS

        @ Read one radio register (16-bit address) and report as an event
        async command READ_REGISTER(addr: U16)

        @ Write one radio register (16-bit address)
        async command WRITE_REGISTER(addr: U16, value: U8)

        @ Reset radio
        async command RESET_RADIO

        @ Bench test: transmit a CW tone or continuous LoRa preamble for a limited time
        async command TX_TEST(
            mode: SbandTestMode @< What to transmit
            powerDbm: I8        @< TX power, -18 to +13 dBm
            durationSec: U16    @< Seconds to transmit (1-60), then return to RX
        )

        @ Stop a running TX_TEST early and return to RX
        async command TX_TEST_STOP

        @ Bench test: send one LoRa packet ("SBAND TEST <n>") with the current settings
        async command TEST_PACKET

        # ------ Parameters -------

        @ Carrier freq in Hz (2400-2500 MHz)
        param FREQUENCY_HZ: U32 default SbandDefaults.FREQUENCY_HZ

        @ TX power dBm (SX1280 range -18 to +13)
        param TX_POWER_DBM: I8 default SbandDefaults.TX_POWER_DBM

        @ LoRa spreading factor (5-12)
        param SPREADING_FACTOR: U8 default SbandDefaults.SPREADING_FACTOR

        @ LoRa bandwidth
        param BANDWIDTH: SbandBandwidth default SbandBandwidth.BW_812_KHZ

        @ LoRa coding rate (1 = 4/5, 2 = 4/6, 3 = 4/7, 4 = 4/8)
        param CODING_RATE: U8 default SbandDefaults.CODING_RATE

        @ Max time (microseconds) to wait for BUSY to go low before giving up
        param BUSY_TIMEOUT_US: U32 default 10000

        @ Ticks (0.1s each) to wait for TX done before failure
        param TX_TIMEOUT_TICKS: U32 default 20

        @ Minimum ticks (0.1s each) between the starts of consecutive transmissions (one frame per period)
        param TX_PERIOD_TICKS: U32 default 10

        @ Ticks (0.1s each) to wait in ERROR before automatically re-initializing the radio
        param ERROR_RETRY_TICKS: U32 default 100

        @ Ticks (0.1s each) with no uplink before reverting the radio to its default settings (0 = disabled)
        param LINK_LOSS_TICKS: U32 default 6000

        # ------ Events -------

        @ Result of GET_STATUS command
        event StatusValue(status: U8) \
            severity activity high \
            format "SX1280 status 0x{x}"

        @ Result of READ_REGISTER command
        event RegisterValue(addr: U16, value: U8) \
            severity activity high \
            format "SX1280 reg 0x{x} = 0x{x}"

        @ Radio is initialized
        event RadioInitialized(firmwareVersion: U16) \
            severity activity high \
            format "S-band radio initialized, firmware version 0x{x}"

        @ Radio failed init (chip did not respond as expected)
        event RadioInitFailed(readValue: U16) \
            severity warning high \
            format "S-band radio init failed: read 0x{x}" \
            throttle 10

        @ Radio config written to chip
        event ConfigApplied(freqHz: U32, powerDbm: I8, sf: U8, bw: SbandBandwidth, cr: U8) \
            severity activity high \
            format "S-band config: {} Hz, {} dBm, SF{}, {}, CR idx {}"

        @ Radio reset requested and started
        event RadioResetStarted \
            severity activity high \
            format "S-band radio reset, re-initializing"

        @ SPI transaction failed
        event SpiError(status: Drv.SpiStatus) \
            severity warning high \
            format "SPI transaction failed: {}" \
            throttle 10

        @ BUSY pin stayed high past BUSY_TIMEOUT_US
        event BusyTimeout(timeoutUs: U32) \
            severity warning high \
            format "SX1280 BUSY stuck high for {} us" \
            throttle 10

        @ Radio did not report TX done in time
        event TxTimeout \
            severity warning high \
            format "S-band radio did not report TX done before timeout"

        @ Could not get buffer for a received packet
        event RxBufferAllocFailed(requested: U32) \
            severity warning high \
            format "Could not allocate {} byte RX buffer" \
            throttle 10

        @ A packet was received (debug view of its contents)
        event RxPacket(numBytes: U32, rssi: I16, snr: I8, hexData: string size 128) \
            severity diagnostic \
            format "RX {} bytes, RSSI {} dBm, SNR {} dB: {}"

        @ Radio is retrying initialization after an error
        event RadioRecoveryAttempt(attempt: U32) \
            severity warning low \
            format "S-band radio in ERROR, re-initializing (attempt {})" \
            throttle 10

        @ No uplink for too long, radio reverted to default settings
        event LinkLossFallback(ticksWithoutUplink: U32) \
            severity warning high \
            format "No uplink for {} ticks, reverting S-band radio to default settings"

        @ Bench test transmission started
        event TestTxStarted(mode: SbandTestMode, powerDbm: I8, durationSec: U16) \
            severity activity high \
            format "S-band test TX started: {}, {} dBm, {} s"

        @ Bench test transmission ended, radio back in RX
        event TestTxStopped \
            severity activity high \
            format "S-band test TX stopped"

        @ Test packet handed to the radio
        event TestPacketSent(count: U32) \
            severity activity high \
            format "S-band test packet {} transmitted"

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

        @ Times BUSY stayed high past the timeout
        telemetry BusyTimeouts: U32

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
