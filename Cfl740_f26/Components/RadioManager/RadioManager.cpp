// ======================================================================
// \title  RadioManager.cpp
// \author tyler
// \brief  cpp file for RadioManager component implementation class
// ======================================================================

#include "Cfl740_f26/Components/RadioManager/RadioManager.hpp"

namespace Cfl740_f26 {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

RadioManager ::RadioManager(const char* const compName) : RadioManagerComponentBase(compName) {}

RadioManager ::~RadioManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

bool RadioManager ::spiTransfer(U8* tx_array, U8* rx_array, FwSizeType length) {
    if(this->isConnected_spiWriteRead_OutputPort(0) == false) {
        return false;
    }

    Fw::Buffer writeBuffer(tx_array, length);
    Fw::Buffer readBuffer(rx_array, length);

    Drv::SpiStatus status = this->spiWriteRead_out(0, writeBuffer, readBuffer);

    if (status != Drv::SpiStatus::SPI_OK) {
        this->m_spiErrors++;
        this->log_WARNING_HI_SpiError(status);
        this->tlmWrite_SpiErrors(this->m_spiErrors);
        return false;
    }
    return true;
}

bool RadioManager ::readRegister(U8 addr, U8& value) {
    U8 tx[2] = {static_cast<U8>(addr & SPI_READ_MASK), 0x00};
    U8 rx[2] = {0x00, 0x00};

    if(!this->spiTransfer(tx, rx, sizeof(tx))) {
        return false;
    }
    value = rx[1];
    return true;
}

bool RadioManager ::writeRegister(U8 addr, U8 value) {
    U8 tx[2] = {static_cast<U8>(addr | SPI_WRITE_MASK), value};
    U8 rx[2] = {0x00, 0x00};

    return this->spiTransfer(tx, rx, sizeof(tx));
}

void RadioManager ::setState(RadioManager_RadioState newState) {
    this->m_radioState = newState;
    this->tlmWrite_State(newState);
}

void RadioManager ::initRadio() {
    // Check chip version
    U8 version = 0;
    if(!this->readRegister(VERSION_REGISTER, version)) {
        this->setState(RadioManager_RadioState::ERROR);
        return;
    }
    if(version != EXPECTED_VERSION) {
        this->log_WARNING_HI_RadioInitFailed(version, EXPECTED_VERSION);
        this->setState(RadioManager_RadioState::ERROR);
        return;
    }

    // Configure Radio
    if (!this->configureRadio()) {
        this->setState(RadioManager_RadioState::ERROR);
        return;
    }

    // Report
    this->log_ACTIVITY_HI_RadioInitialized(version);

    // Now listening
    this->setState(RadioManager_RadioState::RX);

    // Start downlink
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        Fw::Success status = Fw::Success::SUCCESS;
        this->comStatusOut_out(0, status);
    }
}

bool RadioManager ::configureRadio() {
    U8 regValue = 0;
    Fw::ParamValid valid;
    U32 freqHz = this->paramGet_FREQUENCY_HZ(valid);
    U8 bandWidth = this->paramGet_BANDWIDTH(valid);
    U8 codingRate = this->paramGet_CODING_RATE(valid);
    U8 sf = this->paramGet_SPREADING_FACTOR(valid);
    I8 txPower = this->paramGet_TX_POWER_DBM(valid);

    // Sleep
    if (!this->writeRegister(REG_OP_MODE, MODE_SLEEP)) return false;

    // Enable LoRa
    if (!this->writeRegister(REG_OP_MODE, MODE_LORA_SLEEP)) return false;

    if (!this->readRegister(REG_OP_MODE, regValue) || (regValue & LORA_MODE_BIT) == 0) return false;

    // Set Freq
    U64 frf = (static_cast<U64>(freqHz) << 19) / F_XOSC;

    if (!this->writeRegister(REG_FRF_MSB, static_cast<U8>((frf >> 16) & 0xFF))) return false;
    if (!this->writeRegister(REG_FRF_MID, static_cast<U8>((frf >> 8) & 0xFF))) return false;
    if (!this->writeRegister(REG_FRF_LSB, static_cast<U8>(frf & 0xFF))) return false;

    // FIFO base address
    if (!this->writeRegister(REG_FIFO_TX_BASE_ADDR, FIFO_BASE_ADDR)) return false;
    if (!this->writeRegister(REG_FIFO_RX_BASE_ADDR, FIFO_BASE_ADDR)) return false;

    // Bandwidth + coding rate
    if (!this->writeRegister(REG_MODEM_CONFIG_1, static_cast<U8>((bandWidth << 4) | (codingRate << 1)))) return false;

    // Spreading factor + CRC
    if(sf < 7 || sf > 12) return false;
    if (!this->writeRegister(REG_MODEM_CONFIG_2, static_cast<U8>((sf << 4) | (CRC_ON_BIT)))) return false;

    // AGC; LDRO: symbol > 16ms; sf >= 11 assumes 125 kHz
    regValue = AGC_ON_BIT;
    if (sf >= 11) {
        regValue |= LDRO_BIT;
    }
    if (!this->writeRegister(REG_MODEM_CONFIG_3, regValue)) return false;

    // TX Power
    if (txPower < TX_POWER_MIN || txPower > TX_POWER_MAX) return false;

    regValue = static_cast<U8>(PA_BOOST_BIT | (txPower - TX_POWER_MIN));
    
    if (!this->writeRegister(REG_PA_CONFIG, regValue)) return false;

    //Standby -> RX
    if (!this->writeRegister(REG_OP_MODE, MODE_LORA_STANDBY)) return false;
    if (!this->writeRegister(REG_OP_MODE, MODE_LORA_RX_CONT)) return false;

    this->log_ACTIVITY_HI_ConfigApplied(freqHz, txPower, sf, bandWidth, codingRate);

    return true;
}

bool RadioManager ::writeFifo(const U8* data, FwSizeType len) {
    if (len > MAX_PAYLOAD || len == 0) return false;

    U8 local_TX[MAX_PAYLOAD+1] = {static_cast<U8>(REG_FIFO | SPI_WRITE_MASK)};

    for (FwSizeType i = 1; i <= len; i++) {
        local_TX[i] = data[i-1];
    }

    U8 local_RX[MAX_PAYLOAD+1] = {0};

    return this->spiTransfer(local_TX, local_RX, len+1);
}

void RadioManager ::finishTx(bool success) {
    if (!this->m_txInProgress) return;

    this->m_txInProgress = false; 

    if (this->isConnected_dataReturnOut_OutputPort(0)) {
        this->dataReturnOut_out(0, m_txBuffer, m_txContext);
    }

    if (success) {
        m_framesTransmitted++;
        this->tlmWrite_FramesSent(m_framesTransmitted);
    }
    else {
        m_txFailures++;
        this->tlmWrite_TxFailures(m_txFailures);
    }

    bool radioOk = false;

    if (this->m_radioState == RadioManager_RadioState::TX ||
        this->m_radioState == RadioManager_RadioState::RX) {
        radioOk = this->writeRegister(REG_OP_MODE, MODE_LORA_RX_CONT);
        
        if (radioOk) {
            this->setState(RadioManager_RadioState::RX);
        }
        else {
            this->setState(RadioManager_RadioState::ERROR);
        }
    }
    
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        Fw::Success  status = radioOk ? Fw::Success::SUCCESS : Fw::Success::FAILURE;
        this->comStatusOut_out(0, status);
    }
}

void RadioManager ::checkTxDone() {
    this->m_txTicks++;

    U8 flag = 0;
    if(!this->readRegister(REG_IRQ_FLAGS, flag)) {
        this->finishTx(false);
        return;
    }

    U8 regValue = 0;
    if ((flag & TX_DONE_BIT) != 0) {
        this->writeRegister(REG_IRQ_FLAGS, TX_DONE_BIT);
        this->finishTx(true);
        return;
    }

    Fw::ParamValid valid;
    U32 timeout = this->paramGet_TX_TIMEOUT_TICKS(valid);

    if (this->m_txTicks >= timeout) {
        this->log_WARNING_HI_TxTimeout();
        this->finishTx(false);
    }
}

bool RadioManager ::readFifo(U8* dest, FwSizeType len) {
    if (len > MAX_PAYLOAD || len == 0) return false;

    U8 local_TX[MAX_PAYLOAD+1] = {static_cast<U8>(REG_FIFO & SPI_READ_MASK), 0};

    U8 local_RX[MAX_PAYLOAD+1] = {0};

    if(!this->spiTransfer(local_TX, local_RX, len+1)) return false;

    for (FwSizeType i = 1; i <= len; i++) {
        dest[i-1] = local_RX[i];
    }
    return true;
}

void RadioManager ::pollRx() {
    U8 flag = 0;
    if (!this->readRegister(REG_IRQ_FLAGS, flag)) return;

    if ((flag & RX_DONE_BIT) == 0) return;

    if ((flag & CRC_ERROR_BIT) != 0) {
        this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);
        return;
    }

    U8 rxLen = 0;
    U8 rxAddr = 0;
    if (!this->readRegister(REG_RX_NB_BYTES, rxLen) ||
        rxLen == 0 ||
        !this->readRegister(REG_FIFO_RX_CURRENT_ADDR, rxAddr) ||
        !this->writeRegister(REG_FIFO_ADDR_PTR, rxAddr)) {

        this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);
        return;
    }
    
    if (!this->isConnected_allocate_OutputPort(0)) {
        this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);
        return;
    } 

    Fw::Buffer rxBuffer = this->allocate_out(0, rxLen);

    if (rxBuffer.getData() == nullptr || rxBuffer.getSize() < rxLen) {
        if (rxBuffer.getData() != nullptr && this->isConnected_deallocate_OutputPort(0)) {
            this->deallocate_out(0, rxBuffer);
        }
        this->log_WARNING_HI_RxBufferAllocFailed(rxLen);
        this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);
        return;
    }

    if(!this->readFifo(rxBuffer.getData(), rxLen)) {
        if (this->isConnected_deallocate_OutputPort(0)) {
            this->deallocate_out(0, rxBuffer);
        }
        this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);
        return;
    }

    rxBuffer.setSize(rxLen);


    U8 rawSnr = 0;
    U8 rawRssi = 0;

    if(this->readRegister(REG_PKT_SNR_VALUE, rawSnr) && this->readRegister(REG_PKT_RSSI_VALUE, rawRssi)) {
        I8 snr = static_cast<I8>(static_cast<I8>(rawSnr) / 4);
        I16 rssi = RSSI_OFFSET + static_cast<I16>(rawRssi);
        if (snr < 0) {
            rssi += snr;
        }

        this->m_lastSnr = snr;
        this->m_lastRssi = rssi;
        this->tlmWrite_LastSnr(this->m_lastSnr);
        this->tlmWrite_LastRssi(this->m_lastRssi);
    }

    this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);

    if(this->isConnected_dataOut_OutputPort(0)) {
        ComCfg::FrameContext context;
        this->dataOut_out(0, rxBuffer, context);
        this->m_framesReceived++;
        this->tlmWrite_FramesReceived(this->m_framesReceived);
    }
    else {
        if (this->isConnected_deallocate_OutputPort(0)) {
            this->deallocate_out(0, rxBuffer);
        }
    }
}

void RadioManager ::dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    this->transmitFrame_internalInterfaceInvoke(data, context);
}

void RadioManager ::dataReturnIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    if(this->isConnected_deallocate_OutputPort(0)) {
        this->deallocate_out(0, data);
    }
}

void RadioManager ::run_handler(FwIndexType portNum, U32 context) {
    switch (this->m_radioState.e)
    {
    case RadioManager_RadioState::UNINITIALIZED:
        this->initRadio();
        break;

    case RadioManager_RadioState::RX:
        this->pollRx();
        break;

    case RadioManager_RadioState::TX:
        this->checkTxDone();
        break;

    case RadioManager_RadioState::IDLE:
        //Waiting for parameter change
        break;

    case RadioManager_RadioState::ERROR:
        // Waiting for RESET_RADIO
        break;
    
    default:
        break;
    }
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void RadioManager ::READ_REGISTER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 addr) {
    U8 value = 0;
    
    if(this->readRegister(addr, value)) {
        this->log_ACTIVITY_HI_RegisterValue(addr, value);

        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    }
    else {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

void RadioManager ::WRITE_REGISTER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 addr, U8 value) {
    if(this->writeRegister(addr, value)) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    }
    else {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

void RadioManager ::RESET_RADIO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined internal interfaces
// ----------------------------------------------------------------------

void RadioManager ::transmitFrame_internalInterfaceHandler(const Fw::Buffer& data,
                                                           const ComCfg::FrameContext& context) {
    
    if (this->m_txInProgress) {
        if (this->isConnected_dataReturnOut_OutputPort(0)) {
            Fw::Buffer returnBuffer = data;
            this->dataReturnOut_out(0, returnBuffer, context);
        } 
        return;
    }
    
    this->m_txBuffer = data;
    this->m_txContext = context;
    this->m_txInProgress = true;
    this->m_txTicks = 0;

    FwSizeType size = this->m_txBuffer.getSize();

    if (this->m_radioState != RadioManager_RadioState::RX || size == 0 || size > MAX_PAYLOAD) {
        this->finishTx(false);
        return;
    }

    if (!this->writeRegister(REG_OP_MODE, MODE_LORA_STANDBY) ||
        !this->writeRegister(REG_FIFO_ADDR_PTR, FIFO_BASE_ADDR) ||
        !this->writeFifo(this->m_txBuffer.getData(), size) ||
        !this->writeRegister(REG_PAYLOAD_LENGTH, static_cast<U8>(size)) ||
        !this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL) ||
        !this->writeRegister(REG_OP_MODE, MODE_LORA_TX)) {
        
        this->finishTx(false);
        return;
    }

    this->setState(RadioManager_RadioState::TX);
}

}  // namespace Cfl740_f26
