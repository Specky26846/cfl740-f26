// ======================================================================
// \title  SbandRadioManager.hpp
// \author tyler
// \brief  hpp file for SbandRadioManager component implementation class
//         SX1280 (LAMBDA80) 2.4 GHz LoRa radio, com adapter interface
// ======================================================================

#ifndef Cfl740_f26_SbandRadioManager_HPP
#define Cfl740_f26_SbandRadioManager_HPP

#include "Cfl740_f26/Components/SbandRadioManager/SbandRadioManagerComponentAc.hpp"

namespace Cfl740_f26 {

class SbandRadioManager final : public SbandRadioManagerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct SbandRadioManager object
    SbandRadioManager(const char* const compName  //!< The component name
    );

    //! Destroy SbandRadioManager object
    ~SbandRadioManager();

  private:
    // ----------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------

    SbandRadioManager_RadioState m_radioState = SbandRadioManager_RadioState::UNINITIALIZED;

    U32 m_framesReceived = 0;
    U32 m_framesTransmitted = 0;
    I16 m_lastRssi = 0;
    I8 m_lastSnr = 0;

    Fw::Buffer m_txBuffer;
    ComCfg::FrameContext m_txContext;
    bool m_txInProgress = false;
    U32 m_txTicks = 0;

    U32 m_spiErrors = 0;
    U32 m_busyTimeouts = 0;
    U32 m_txFailures = 0;
    U32 m_rxCrcErrors = 0;     // packets with a bad CRC
    U32 m_rxDropped = 0;       // packets dropped for any other reason

    bool m_statusPending = false;  // a comStatus is waiting for the TX period to end
    U32 m_ticksSinceTx = 0;        // ticks since the last transmission started

    U32 m_errorTicks = 0;          // ticks spent in ERROR
    U32 m_recoveryAttempts = 0;    // total recovery attempts

    bool m_configPending = false;  // a radio setting changed; apply it in RX

    U32 m_ticksSinceUplink = 0;    // ticks since the last uplink packet was accepted
    bool m_fallbackActive = false; // using default radio settings after link loss

    // Bench test state
    bool m_testActive = false;        // TX_TEST is running (radio held in IDLE)
    U32 m_testTicksRemaining = 0;     // ticks left in TX_TEST
    bool m_txIsTest = false;          // current TX is a TEST_PACKET, not an F' frame
    U32 m_testPacketCount = 0;
    U8 m_testPayload[32] = {0};

    // ----------------------------------------------------------------------
    // SX1280 command opcodes (Semtech SX1280 user guide, command table)
    // ----------------------------------------------------------------------

    static constexpr U8 OP_GET_STATUS             = 0xC0;
    static constexpr U8 OP_WRITE_REGISTER         = 0x18;
    static constexpr U8 OP_READ_REGISTER          = 0x19;
    static constexpr U8 OP_WRITE_BUFFER           = 0x1A;
    static constexpr U8 OP_READ_BUFFER            = 0x1B;
    static constexpr U8 OP_SET_STANDBY            = 0x80;
    static constexpr U8 OP_SET_RX                 = 0x82;
    static constexpr U8 OP_SET_TX                 = 0x83;
    static constexpr U8 OP_SET_RF_FREQUENCY       = 0x86;
    static constexpr U8 OP_SET_PACKET_TYPE        = 0x8A;
    static constexpr U8 OP_SET_MODULATION_PARAMS  = 0x8B;
    static constexpr U8 OP_SET_PACKET_PARAMS      = 0x8C;
    static constexpr U8 OP_SET_DIO_IRQ_PARAMS     = 0x8D;
    static constexpr U8 OP_SET_TX_PARAMS          = 0x8E;
    static constexpr U8 OP_SET_BUFFER_BASE_ADDR   = 0x8F;
    static constexpr U8 OP_GET_IRQ_STATUS         = 0x15;
    static constexpr U8 OP_CLR_IRQ_STATUS         = 0x97;
    static constexpr U8 OP_GET_RX_BUFFER_STATUS   = 0x17;
    static constexpr U8 OP_GET_PACKET_STATUS      = 0x1D;
    static constexpr U8 OP_SET_TX_CONT_WAVE       = 0xD1;
    static constexpr U8 OP_SET_TX_CONT_PREAMBLE   = 0xD2;
    static constexpr U16 TEST_MAX_DURATION_SEC    = 60;

    static constexpr U8 NOP = 0x00;

    // Command arguments
    static constexpr U8 STDBY_RC          = 0x00;
    static constexpr U8 PACKET_TYPE_LORA  = 0x01;
    static constexpr U8 PERIOD_BASE_15_US = 0x00;  // SetTx / SetRx timeout step
    static constexpr U8 RAMP_20_US        = 0xE0;  // SetTxParams ramp time
    static constexpr U8 PREAMBLE_12_SYM   = 0x0C;  // mantissa 12, exponent 0
    static constexpr U8 HEADER_EXPLICIT   = 0x00;
    static constexpr U8 LORA_CRC_ON       = 0x20;
    static constexpr U8 LORA_IQ_STD       = 0x40;

    // Status byte: circuit mode in bits 7:5
    static constexpr U8 STATUS_MODE_SHIFT = 5;
    static constexpr U8 STATUS_MODE_MASK  = 0x07;
    static constexpr U8 MODE_STDBY_RC     = 0x02;

    // IRQ bits (16-bit mask)
    static constexpr U16 IRQ_TX_DONE       = 0x0001;
    static constexpr U16 IRQ_RX_DONE       = 0x0002;
    static constexpr U16 IRQ_HEADER_ERROR  = 0x0020;
    static constexpr U16 IRQ_CRC_ERROR     = 0x0040;
    static constexpr U16 IRQ_RX_TX_TIMEOUT = 0x4000;
    static constexpr U16 IRQ_ALL           = 0xFFFF;

    // Registers (16-bit addresses)
    static constexpr U16 REG_FIRMWARE_VERSION = 0x0153;  // MSB at 0x0153, LSB at 0x0154
    static constexpr U16 REG_SF_ADDITIONAL    = 0x0925;  // must be set after SetModulationParams
    static constexpr U16 REG_FREQ_ERR_COMP    = 0x093C;  // write 0x01 in LoRa mode

    // Frequency: RF steps = f * 2^18 / 52 MHz
    static constexpr U32 F_XTAL = 52000000;

    // TX power range of the SX1280 (SetTxParams power = dBm + 18)
    static constexpr I8 TX_POWER_MIN = -18;
    static constexpr I8 TX_POWER_MAX = 13;

    // Buffer layout
    static constexpr U8 BUFFER_BASE_ADDR = 0x00;
    static constexpr U8 MAX_PAYLOAD = 255;

    // RadioHead-style header (same as the 433 MHz link)
    static constexpr U8 RH_HEADER_LEN     = 4;
    static constexpr U8 RH_ADDR_FLIGHT    = 0x01;
    static constexpr U8 RH_ADDR_GROUND    = 0x02;
    static constexpr U8 RH_ADDR_BROADCAST = 0xFF;
    static constexpr FwSizeType RX_DEBUG_MAX_BYTES = 40;  // 40 bytes x 3 chars = 120 < 128

    // ----------------------------------------------------------------------
    // Low-level SPI helpers
    // ----------------------------------------------------------------------

    //! Wait for BUSY to go low (true) or time out (false)
    bool waitWhileBusy();

    //! Raw full-duplex SPI transfer
    bool spiTransfer(U8* tx, U8* rx, FwSizeType length);

    //! Send a "Set" style command: opcode followed by parameters
    bool sendCommand(U8 opcode, const U8* params, FwSizeType numParams);

    //! Send a "Get" style command and return the data bytes after the status bytes
    bool readCommand(U8 opcode, U8* out, FwSizeType numBytes);

    //! Read the status byte (GetStatus)
    bool getStatus(U8& status);

    bool readRegister(U16 addr, U8& value);
    bool writeRegister(U16 addr, U8 value);

    //! Write header + payload into the data buffer in one transaction
    bool writeBuffer(U8 offset, const U8* header, FwSizeType headerLen, const U8* data, FwSizeType dataLen);
    bool readBuffer(U8 offset, U8* dest, FwSizeType len);

    bool getIrqStatus(U16& irq);
    bool clearIrq(U16 mask);

    bool setStandby();
    bool setRxContinuous();
    bool setTx();
    bool setPacketParams(U8 payloadLen);

    // ----------------------------------------------------------------------
    // Radio logic
    // ----------------------------------------------------------------------

    void setState(SbandRadioManager_RadioState newState);
    void hardwareReset();
    void initRadio();
    bool configureRadio();
    void finishTx(bool success);
    void checkTxDone();
    void pollRx();
    void dropRx();
    void trySendPendingStatus();  // send the deferred SUCCESS once the TX period has elapsed
    void stopTestTx();            // end TX_TEST and go back to RX

    void parameterUpdated(FwPrmIdType id) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for dataIn
    //!
    //! Data to be sent on the wire (coming in to the component)
    void dataIn_handler(FwIndexType portNum,  //!< The port number
                        Fw::Buffer& data,
                        const ComCfg::FrameContext& context) override;

    //! Handler implementation for dataReturnIn
    //!
    //! Port receiving back ownership of buffer sent out on dataOut
    void dataReturnIn_handler(FwIndexType portNum,  //!< The port number
                              Fw::Buffer& data,
                              const ComCfg::FrameContext& context) override;

    //! Handler implementation for pingIn
    //!
    //! Health check: ping in from Svc.Health
    void pingIn_handler(FwIndexType portNum,  //!< The port number
                        U32 key               //!< Value to return to pinger
                        ) override;

    //! Handler implementation for run
    //!
    //! Port receiving calls from the rate group
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command GET_STATUS
    //!
    //! Send GetStatus (0xC0) and report the status byte as an event
    void GET_STATUS_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                               U32 cmdSeq            //!< The command sequence number
                               ) override;

    //! Handler implementation for command READ_REGISTER
    //!
    //! Read one radio register (16-bit address) and report as an event
    void READ_REGISTER_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                  U32 cmdSeq,           //!< The command sequence number
                                  U16 addr) override;

    //! Handler implementation for command WRITE_REGISTER
    //!
    //! Write one radio register (16-bit address)
    void WRITE_REGISTER_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                   U32 cmdSeq,           //!< The command sequence number
                                   U16 addr,
                                   U8 value) override;

    //! Handler implementation for command RESET_RADIO
    //!
    //! Reset radio
    void RESET_RADIO_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                U32 cmdSeq            //!< The command sequence number
                                ) override;

    void TX_TEST_cmdHandler(FwOpcodeType opCode,
                            U32 cmdSeq,
                            const Cfl740_f26::SbandTestMode& mode,
                            I8 powerDbm,
                            U16 durationSec) override;

    void TX_TEST_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    void TEST_PACKET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for user-defined internal interfaces
    // ----------------------------------------------------------------------

    //! Handler implementation for transmitFrame
    //!
    //! Moves frame from dataIn onto our thread
    void transmitFrame_internalInterfaceHandler(const Fw::Buffer& data, const ComCfg::FrameContext& context) override;
};

}  // namespace Cfl740_f26

#endif
