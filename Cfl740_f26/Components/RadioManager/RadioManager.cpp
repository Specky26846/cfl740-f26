// ======================================================================
// \title  RadioManager.cpp
// \author tyler
// \brief  cpp file for RadioManager component implementation class
// ======================================================================

#include "Cfl740_f26/Components/RadioManager/RadioManager.hpp"
#include "Cfl740_f26/Components/RadioManager/FppConstantsAc.hpp"
#include <cstdio>
#include <Fw/Types/String.hpp>
#include <Os/Task.hpp>
#include <Fw/Time/TimeInterval.hpp>

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
    if (newState == RadioManager_RadioState::ERROR &&
        this->m_radioState != RadioManager_RadioState::ERROR) {
        this->m_errorTicks = 0;
    }
    this->m_radioState = newState;
    this->tlmWrite_State(newState);
}

void RadioManager ::initRadio() {
    this->hardwareReset();
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

    this->m_configPending = false;

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

    if (this->m_fallbackActive) {
        freqHz     = static_cast<U32>(RadioDefaults::FREQUENCY_HZ);
        txPower    = static_cast<I8>(RadioDefaults::TX_POWER_DBM);
        sf         = static_cast<U8>(RadioDefaults::SPREADING_FACTOR);
        bandWidth  = static_cast<U8>(RadioDefaults::BANDWIDTH);
        codingRate = static_cast<U8>(RadioDefaults::CODING_RATE);
    }

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

    if (bandWidth > 9 || codingRate < 1 || codingRate > 4) return false;

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
    
    if (radioOk) {
        // Listen for a while before asking ComQueue for the next frame
        this->m_statusPending = true;
    }
    else if (this->isConnected_comStatusOut_OutputPort(0)) {
        Fw::Success status = Fw::Success::FAILURE;
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

    if ((flag & TX_DONE_BIT) != 0) {
        this->writeRegister(REG_IRQ_FLAGS, TX_DONE_BIT);
        this->finishTx(true);
        this->trySendPendingStatus();   // next frame now, if the period has already passed
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
        this->m_rxCrcErrors++;
        this->tlmWrite_RxCrcErrors(this->m_rxCrcErrors);
        this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);
        return;
    }

    U8 rxLen = 0;
    U8 rxAddr = 0;
    if (!this->readRegister(REG_RX_NB_BYTES, rxLen) ||
        rxLen <= RH_HEADER_LEN ||
        !this->readRegister(REG_FIFO_RX_CURRENT_ADDR, rxAddr) ||
        !this->writeRegister(REG_FIFO_ADDR_PTR, rxAddr)) {

        this->dropRx();
        return;
    }
    
    U8 rhHeader[RH_HEADER_LEN] = {0};
    if (!this->readFifo(rhHeader, RH_HEADER_LEN) ||
        (rhHeader[0] != RH_ADDR_FLIGHT && rhHeader[0] != RH_ADDR_BROADCAST)) {
        this->dropRx();
        return;
    }

    this->m_ticksSinceUplink = 0;
    
    if (!this->isConnected_allocate_OutputPort(0)) {
        this->dropRx();
        return;
    } 

    const U8 frameLen = static_cast<U8>(rxLen - RH_HEADER_LEN);

    Fw::Buffer rxBuffer = this->allocate_out(0, frameLen);

    if (rxBuffer.getData() == nullptr || rxBuffer.getSize() < frameLen) {
        if (rxBuffer.getData() != nullptr && this->isConnected_deallocate_OutputPort(0)) {
            this->deallocate_out(0, rxBuffer);
        }
        this->log_WARNING_HI_RxBufferAllocFailed(frameLen);
        this->dropRx();
        return;
    }

    if(!this->readFifo(rxBuffer.getData(), frameLen)) {
        if (this->isConnected_deallocate_OutputPort(0)) {
            this->deallocate_out(0, rxBuffer);
        }
        this->dropRx();
        return;
    }

    rxBuffer.setSize(frameLen);


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

    // Done with the radio for this packet - ready for the next one
    this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);

    // Count every packet received
    this->m_framesReceived++;
    this->tlmWrite_FramesReceived(this->m_framesReceived);

    // Debug: show up to the first RX_DEBUG_MAX_BYTES bytes as hex in the GDS
    // (must happen BEFORE the buffer is sent away or freed)
    char hex[128] = {0};
    FwSizeType showLen = (frameLen < RX_DEBUG_MAX_BYTES) ? frameLen : RX_DEBUG_MAX_BYTES;
    const U8* bytes = rxBuffer.getData();
    for (FwSizeType i = 0; i < showLen; i++) {
        snprintf(&hex[i * 3], 4, "%02X ", bytes[i]);
    }
    Fw::String hexStr(hex);
    this->log_DIAGNOSTIC_RxPacket(frameLen, this->m_lastRssi, this->m_lastSnr, hexStr);

    // Hand the frame to the comm stack, or free it if nothing is connected
    if (this->isConnected_dataOut_OutputPort(0)) {
        ComCfg::FrameContext context;
        this->dataOut_out(0, rxBuffer, context);
    }
    else if (this->isConnected_deallocate_OutputPort(0)) {
        this->deallocate_out(0, rxBuffer);
    }
}

void RadioManager ::hardwareReset() {
    if (!this->isConnected_resetPin_OutputPort(0)) {
        return;
    }

    this->resetPin_out(0, Fw::Logic::LOW);
    Os::Task::delay(Fw::TimeInterval(0, 1000));
    this->resetPin_out(0, Fw::Logic::HIGH);
    Os::Task::delay(Fw::TimeInterval(0, 10000));
}

void RadioManager ::parameterUpdated(FwPrmIdType id) {
    switch (id) {
        case PARAMID_FREQUENCY_HZ:


        case PARAMID_TX_POWER_DBM:


        case PARAMID_SPREADING_FACTOR:


        case PARAMID_BANDWIDTH:


        case PARAMID_CODING_RATE:
            this->m_configPending = true;
            this->m_fallbackActive = false;
            break;
        
        default:
            break;
    }
}

void RadioManager ::dropRx() {
    this->m_rxDropped++;
    this->tlmWrite_RxDropped(this->m_rxDropped);
    this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL);
}

void RadioManager ::trySendPendingStatus() {
    if (!this->m_statusPending) {
        return;
    }
    Fw::ParamValid valid;
    if (this->m_ticksSinceTx >= this->paramGet_TX_PERIOD_TICKS(valid)) {
        this->m_statusPending = false;
        if (this->isConnected_comStatusOut_OutputPort(0)) {
            Fw::Success status = Fw::Success::SUCCESS;
            this->comStatusOut_out(0, status);
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

void RadioManager ::pingIn_handler(FwIndexType portNum, U32 key) {
    // Reply on our own thread: proves the radio task is alive and processing its queue
    if (this->isConnected_pingOut_OutputPort(0)) {
        this->pingOut_out(0, key);
    }
}

void RadioManager ::run_handler(FwIndexType portNum, U32 context) {
    this->m_ticksSinceUplink++;
    this->m_ticksSinceTx++;
    switch (this->m_radioState.e)
    {
    case RadioManager_RadioState::UNINITIALIZED:
        this->initRadio();
        break;

    case RadioManager_RadioState::RX:
        {
            Fw::ParamValid valid;
            const U32 limit = this->paramGet_LINK_LOSS_TICKS(valid);
            if (limit > 0 && !this->m_fallbackActive && this->m_ticksSinceUplink >= limit) {
                this->log_WARNING_HI_LinkLossFallback(this->m_ticksSinceUplink);
                this->m_fallbackActive = true;
                this->m_ticksSinceUplink = 0;
                this->m_configPending = true;
            }
        }

        if (this->m_configPending) {
            this->m_configPending = false;
            this->setState(RadioManager_RadioState::IDLE);
            if (!this->configureRadio()) {
                this->setState(RadioManager_RadioState::ERROR);
                break;
            }
            this->setState(RadioManager_RadioState::RX);
        }

        this->pollRx();
        this->trySendPendingStatus();
        break;

    case RadioManager_RadioState::TX:
        this->checkTxDone();
        break;

    case RadioManager_RadioState::IDLE:
        // Reconfiguring
        break;

    case RadioManager_RadioState::ERROR: {
        this->m_errorTicks++;
        Fw::ParamValid valid;
        if (this->m_errorTicks >= this->paramGet_ERROR_RETRY_TICKS(valid)) {
            this->m_recoveryAttempts++;
            this->log_WARNING_LO_RadioRecoveryAttempt(this->m_recoveryAttempts);
            this->m_statusPending = false;
            this->setState(RadioManager_RadioState::UNINITIALIZED);
        }
        break;
    }
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
    this->log_ACTIVITY_HI_RadioResetStarted();

    if (this->m_txInProgress) {
        this->finishTx(false);
    }

    this->m_statusPending = false;
    this->setState(RadioManager_RadioState::UNINITIALIZED);

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

    if (this->m_radioState != RadioManager_RadioState::RX || size == 0 || size > MAX_PAYLOAD - RH_HEADER_LEN) {
        this->finishTx(false);
        return;
    }

    U8 rhHeader[RH_HEADER_LEN] = {RH_ADDR_BROADCAST, RH_ADDR_FLIGHT, 0x00, 0x00};

    if (!this->writeRegister(REG_OP_MODE, MODE_LORA_STANDBY) ||
        !this->writeRegister(REG_FIFO_ADDR_PTR, FIFO_BASE_ADDR) ||
        !this->writeFifo(rhHeader, RH_HEADER_LEN) ||
        !this->writeFifo(this->m_txBuffer.getData(), size) ||
        !this->writeRegister(REG_PAYLOAD_LENGTH, static_cast<U8>(size + RH_HEADER_LEN)) ||
        !this->writeRegister(REG_IRQ_FLAGS, IRQ_CLEAR_ALL) ||
        !this->writeRegister(REG_OP_MODE, MODE_LORA_TX)) {
        
        this->finishTx(false);
        return;
    }

    this->m_ticksSinceTx = 0;
    this->setState(RadioManager_RadioState::TX);
}

}  // namespace Cfl740_f26
