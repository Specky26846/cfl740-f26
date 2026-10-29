// ======================================================================
// \title  RadioManager.hpp
// \author tyler
// \brief  hpp file for RadioManager component implementation class
// ======================================================================

#ifndef Cfl740_f26_RadioManager_HPP
#define Cfl740_f26_RadioManager_HPP

#include "Cfl740_f26/Components/RadioManager/RadioManagerComponentAc.hpp"

namespace Cfl740_f26 {

class RadioManager final : public RadioManagerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct RadioManager object
    RadioManager(const char* const compName  //!< The component name
    );

    //! Destroy RadioManager object
    ~RadioManager();

  private:

    U32  m_framesReceived = 0;
    U32 m_framesTransmitted = 0;
    I16 m_lastRssi = 0;
    I8 m_lastSnr = 0;
    RadioManager_RadioState m_radioState = RadioManager_RadioState::UNINITIALIZED;

    Fw::Buffer m_txBuffer;
    ComCfg::FrameContext m_txContext;
    bool m_txInProgress = false;
    U32 m_txTicks = 0;
    U32 m_spiErrors = 0;
    U32 m_txFailures = 0;

    static constexpr U8 VERSION_REGISTER = 0x42;
    static constexpr U8 EXPECTED_VERSION = 0x12;
    static constexpr U8 SPI_READ_MASK = 0x7F;
    static constexpr U8 SPI_WRITE_MASK = 0x80;

    // 433 MHz radio: low-frequency bit 0x08 set
    static constexpr U8 REG_OP_MODE       = 0x01;
    static constexpr U8 MODE_SLEEP        = 0x00;  // FSK sleep - required before switching to LoRa
    static constexpr U8 MODE_LORA_SLEEP   = 0x88;  // LoRa | low-freq | sleep
    static constexpr U8 MODE_LORA_STANDBY = 0x89;  // LoRa | low-freq | standby
    static constexpr U8 MODE_LORA_RX_CONT = 0x8D;  // LoRa | low-freq | RX continuous
    static constexpr U8 LORA_MODE_BIT     = 0x80;  // bit 7 of RegOpMode

    static constexpr U32 F_XOSC = 32000000; // Used to calculate Frf for register
    static constexpr U8 REG_FRF_MSB = 0x06; 
    static constexpr U8 REG_FRF_MID = 0x07; 
    static constexpr U8 REG_FRF_LSB = 0x08; 

    static constexpr U8 FIFO_BASE_ADDR = 0x00;
    static constexpr U8 REG_FIFO_TX_BASE_ADDR = 0x0E;
    static constexpr U8 REG_FIFO_RX_BASE_ADDR = 0x0F;

    static constexpr U8 REG_MODEM_CONFIG_1 = 0x1D;
    static constexpr U8 REG_MODEM_CONFIG_2 = 0x1E;
    static constexpr U8 CRC_ON_BIT = 0x04;
    static constexpr U8 REG_MODEM_CONFIG_3 = 0x26;
    static constexpr U8 AGC_ON_BIT = 0x04;
    static constexpr U8 LDRO_BIT = 0x08;

    static constexpr U8 REG_PA_CONFIG = 0x09;
    static constexpr U8 PA_BOOST_BIT    = 0x80;  // PaSelect: route output through PA_BOOST (RFM9x)
    static constexpr I8 TX_POWER_MIN    = 2;     // PA_BOOST range without RegPaDac high-power mode
    static constexpr I8 TX_POWER_MAX    = 17;

    static constexpr U8 REG_FIFO = 0x00;
    static constexpr U8 REG_FIFO_ADDR_PTR = 0x0D;
    static constexpr U8 REG_IRQ_FLAGS = 0x12;
    static constexpr U8 IRQ_CLEAR_ALL = 0xFF;
    static constexpr U8 TX_DONE_BIT = 0x08;
    static constexpr U8 REG_PAYLOAD_LENGTH = 0x22;
    static constexpr U8 MAX_PAYLOAD = 255;
    static constexpr U8 MODE_LORA_TX = 0x8B; // (LoRa + low-freq + 011)
    static constexpr U8 CRC_ERROR_BIT = 0x20;

    static constexpr U8 RX_DONE_BIT = 0x40;
    static constexpr U8 REG_RX_NB_BYTES = 0x13;
    static constexpr U8 REG_FIFO_RX_CURRENT_ADDR = 0x10;
    static constexpr U8 REG_PKT_SNR_VALUE = 0x19;
    static constexpr U8 REG_PKT_RSSI_VALUE = 0x1A;
    static constexpr I16 RSSI_OFFSET = -164;

    bool spiTransfer(U8* tx_array, U8* rx_array, FwSizeType length);

    bool readRegister(U8 addr, U8& value);

    bool writeRegister(U8 addr, U8 value);

    void setState(RadioManager_RadioState newState);

    void initRadio();

    bool configureRadio();

    bool writeFifo(const U8* data, FwSizeType len);
    
    bool readFifo(U8* dest, FwSizeType len);

    void finishTx(bool success);

    void checkTxDone();

    void pollRx();

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

    //! Handler implementation for command READ_REGISTER
    //!
    //! Read one radio register and report as an event
    void READ_REGISTER_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                  U32 cmdSeq,           //!< The command sequence number
                                  U8 addr) override;

    //! Handler implementation for command WRITE_REGISTER
    //!
    //! Write one radio register
    void WRITE_REGISTER_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                   U32 cmdSeq,           //!< The command sequence number
                                   U8 addr,
                                   U8 value) override;

    //! Handler implementation for command RESET_RADIO
    //!
    //! Reset radio
    void RESET_RADIO_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                U32 cmdSeq            //!< The command sequence number
                                ) override;

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
