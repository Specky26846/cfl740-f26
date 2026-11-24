// ======================================================================
// \title  SbandRadioManager.cpp
// \author tyler
// \brief  cpp file for SbandRadioManager component implementation class
//         SX1280 (LAMBDA80) 2.4 GHz LoRa radio, com adapter interface
// ======================================================================

#include "Cfl740_f26/Components/SbandRadioManager/SbandRadioManager.hpp"
#include "Cfl740_f26/Components/SbandRadioManager/FppConstantsAc.hpp"
#include <cstdio>
#include <cstring>
#include <Fw/Types/String.hpp>
#include <Fw/Time/TimeInterval.hpp>
#include <Os/RawTime.hpp>
#include <Os/Task.hpp>

namespace Cfl740_f26 {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

SbandRadioManager ::SbandRadioManager(const char* const compName) : SbandRadioManagerComponentBase(compName) {}

SbandRadioManager ::~SbandRadioManager() {}

// ----------------------------------------------------------------------
// Low-level SPI helpers
// ----------------------------------------------------------------------

bool SbandRadioManager ::waitWhileBusy() {
    if (!this->isConnected_busyPin_OutputPort(0)) {
        return false;
    }

    Fw::ParamValid valid;
    const U32 timeoutUs = this->paramGet_BUSY_TIMEOUT_US(valid);

    Os::RawTime start;
    start.now();

    while (true) {
        Fw::Logic busy = Fw::Logic::HIGH;
        const Drv::GpioStatus status = this->busyPin_out(0, busy);
        if (status == Drv::GpioStatus::OP_OK && busy == Fw::Logic::LOW) {
            return true;
        }

        Os::RawTime now;
        now.now();
        U32 elapsedUs = 0;
        now.getDiffUsec(start, elapsedUs);
        if (elapsedUs > timeoutUs) {
            this->m_busyTimeouts++;
            this->tlmWrite_BusyTimeouts(this->m_busyTimeouts);
            this->log_WARNING_HI_BusyTimeout(timeoutUs);
            return false;
        }
    }
}

bool SbandRadioManager ::spiTransfer(U8* tx, U8* rx, FwSizeType length) {
    if (!this->isConnected_spiWriteRead_OutputPort(0)) {
        return false;
    }

    Fw::Buffer writeBuffer(tx, length);
    Fw::Buffer readBuffer(rx, length);

    const Drv::SpiStatus status = this->spiWriteRead_out(0, writeBuffer, readBuffer);

    if (status != Drv::SpiStatus::SPI_OK) {
        this->m_spiErrors++;
        this->log_WARNING_HI_SpiError(status);
        this->tlmWrite_SpiErrors(this->m_spiErrors);
        return false;
    }
    return true;
}

bool SbandRadioManager ::sendCommand(U8 opcode, const U8* params, FwSizeType numParams) {
    // Largest Set command (SetDioIrqParams) has 8 parameter bytes
    static constexpr FwSizeType MAX_PARAMS = 8;
    if (numParams > MAX_PARAMS || (numParams > 0 && params == nullptr)) {
        return false;
    }
    if (!this->waitWhileBusy()) {
        return false;
    }

    U8 tx[1 + MAX_PARAMS] = {opcode};
    U8 rx[1 + MAX_PARAMS] = {0};
    for (FwSizeType i = 0; i < numParams; i++) {
        tx[1 + i] = params[i];
    }
    return this->spiTransfer(tx, rx, 1 + numParams);
}

bool SbandRadioManager ::readCommand(U8 opcode, U8* out, FwSizeType numBytes) {
    // Get commands: [opcode, NOP, data...]; RX: [status, status, data...]
    static constexpr FwSizeType MAX_DATA = 5;
    static constexpr FwSizeType DATA_OFFSET = 2;
    if (numBytes == 0 || numBytes > MAX_DATA || out == nullptr) {
        return false;
    }
    if (!this->waitWhileBusy()) {
        return false;
    }

    U8 tx[DATA_OFFSET + MAX_DATA] = {opcode};
    U8 rx[DATA_OFFSET + MAX_DATA] = {0};
    if (!this->spiTransfer(tx, rx, DATA_OFFSET + numBytes)) {
        return false;
    }
    for (FwSizeType i = 0; i < numBytes; i++) {
        out[i] = rx[DATA_OFFSET + i];
    }
    return true;
}

bool SbandRadioManager ::getStatus(U8& status) {
    if (!this->waitWhileBusy()) {
        return false;
    }
    // The opcode byte clocks out 0xFF; the status comes back on the following NOP
    U8 tx[2] = {OP_GET_STATUS, NOP};
    U8 rx[2] = {0};
    if (!this->spiTransfer(tx, rx, sizeof(tx))) {
        return false;
    }
    status = rx[1];
    return true;
}

bool SbandRadioManager ::readRegister(U16 addr, U8& value) {
    // [0x19, addrHi, addrLo, NOP, NOP] -> value at rx[4]
    if (!this->waitWhileBusy()) {
        return false;
    }
    U8 tx[5] = {OP_READ_REGISTER, static_cast<U8>(addr >> 8), static_cast<U8>(addr & 0xFF), NOP, NOP};
    U8 rx[5] = {0};
    if (!this->spiTransfer(tx, rx, sizeof(tx))) {
        return false;
    }
    value = rx[4];
    return true;
}

bool SbandRadioManager ::writeRegister(U16 addr, U8 value) {
    // [0x18, addrHi, addrLo, value]
    if (!this->waitWhileBusy()) {
        return false;
    }
    U8 tx[4] = {OP_WRITE_REGISTER, static_cast<U8>(addr >> 8), static_cast<U8>(addr & 0xFF), value};
    U8 rx[4] = {0};
    return this->spiTransfer(tx, rx, sizeof(tx));
}

bool SbandRadioManager ::writeBuffer(U8 offset,
                                     const U8* header,
                                     FwSizeType headerLen,
                                     const U8* data,
                                     FwSizeType dataLen) {
    // [0x1A, offset, header..., data...]
    const FwSizeType total = headerLen + dataLen;
    if (total == 0 || total > MAX_PAYLOAD) {
        return false;
    }
    if (!this->waitWhileBusy()) {
        return false;
    }

    U8 tx[2 + MAX_PAYLOAD] = {OP_WRITE_BUFFER, offset};
    U8 rx[2 + MAX_PAYLOAD] = {0};
    if (headerLen > 0) {
        memcpy(&tx[2], header, headerLen);
    }
    if (dataLen > 0) {
        memcpy(&tx[2 + headerLen], data, dataLen);
    }
    return this->spiTransfer(tx, rx, 2 + total);
}

bool SbandRadioManager ::readBuffer(U8 offset, U8* dest, FwSizeType len) {
    // [0x1B, offset, NOP, NOP...] -> data starts at rx[3]
    if (len == 0 || len > MAX_PAYLOAD || dest == nullptr) {
        return false;
    }
    if (!this->waitWhileBusy()) {
        return false;
    }

    U8 tx[3 + MAX_PAYLOAD] = {OP_READ_BUFFER, offset, NOP};
    U8 rx[3 + MAX_PAYLOAD] = {0};
    if (!this->spiTransfer(tx, rx, 3 + len)) {
        return false;
    }
    memcpy(dest, &rx[3], len);
    return true;
}

bool SbandRadioManager ::getIrqStatus(U16& irq) {
    U8 out[2] = {0};
    if (!this->readCommand(OP_GET_IRQ_STATUS, out, sizeof(out))) {
        return false;
    }
    irq = static_cast<U16>((static_cast<U16>(out[0]) << 8) | out[1]);
    return true;
}

bool SbandRadioManager ::clearIrq(U16 mask) {
    const U8 params[2] = {static_cast<U8>(mask >> 8), static_cast<U8>(mask & 0xFF)};
    return this->sendCommand(OP_CLR_IRQ_STATUS, params, sizeof(params));
}

bool SbandRadioManager ::setStandby() {
    const U8 params[1] = {STDBY_RC};
    return this->sendCommand(OP_SET_STANDBY, params, sizeof(params));
}

bool SbandRadioManager ::setRxContinuous() {
    // periodBaseCount 0xFFFF = continuous RX
    const U8 params[3] = {PERIOD_BASE_15_US, 0xFF, 0xFF};
    return this->sendCommand(OP_SET_RX, params, sizeof(params));
}

bool SbandRadioManager ::setTx() {
    // periodBaseCount 0 = no chip timeout (TX_TIMEOUT_TICKS handles that)
    const U8 params[3] = {PERIOD_BASE_15_US, 0x00, 0x00};
    return this->sendCommand(OP_SET_TX, params, sizeof(params));
}

bool SbandRadioManager ::setPacketParams(U8 payloadLen) {
    // LoRa: preamble, header type, payload length, CRC, IQ, 2 unused
    const U8 params[7] = {PREAMBLE_12_SYM, HEADER_EXPLICIT, payloadLen, LORA_CRC_ON, LORA_IQ_STD, 0x00, 0x00};
    return this->sendCommand(OP_SET_PACKET_PARAMS, params, sizeof(params));
}

// ----------------------------------------------------------------------
// Radio logic
// ----------------------------------------------------------------------

void SbandRadioManager ::setState(SbandRadioManager_RadioState newState) {
    if (newState == SbandRadioManager_RadioState::ERROR &&
        this->m_radioState != SbandRadioManager_RadioState::ERROR) {
        this->m_errorTicks = 0;
    }
    this->m_radioState = newState;
    this->tlmWrite_State(newState);
}

void SbandRadioManager ::hardwareReset() {
    if (!this->isConnected_resetPin_OutputPort(0)) {
        return;
    }

    this->resetPin_out(0, Fw::Logic::LOW);
    Os::Task::delay(Fw::TimeInterval(0, 1000));    // 1 ms low pulse
    this->resetPin_out(0, Fw::Logic::HIGH);
    Os::Task::delay(Fw::TimeInterval(0, 10000));   // 10 ms for the chip to boot
    // BUSY is checked before the first command
}

void SbandRadioManager ::initRadio() {
    this->hardwareReset();

    // After reset the chip should be in STDBY_RC
    U8 status = 0;
    if (!this->getStatus(status)) {
        this->setState(SbandRadioManager_RadioState::ERROR);
        return;
    }
    const U8 mode = static_cast<U8>((status >> STATUS_MODE_SHIFT) & STATUS_MODE_MASK);

    // Firmware version is a sanity check that SPI reads real data
    U8 fwHi = 0;
    U8 fwLo = 0;
    if (!this->readRegister(REG_FIRMWARE_VERSION, fwHi) || !this->readRegister(REG_FIRMWARE_VERSION + 1, fwLo)) {
        this->setState(SbandRadioManager_RadioState::ERROR);
        return;
    }
    const U16 fwVersion = static_cast<U16>((static_cast<U16>(fwHi) << 8) | fwLo);

    // All 0x00 or all 0xFF means nothing is answering on the bus
    if (mode != MODE_STDBY_RC || fwVersion == 0x0000 || fwVersion == 0xFFFF) {
        this->log_WARNING_HI_RadioInitFailed(fwVersion);
        this->setState(SbandRadioManager_RadioState::ERROR);
        return;
    }

    if (!this->configureRadio()) {
        this->setState(SbandRadioManager_RadioState::ERROR);
        return;
    }

    this->m_configPending = false;

    this->log_ACTIVITY_HI_RadioInitialized(fwVersion);

    // Now listening
    this->setState(SbandRadioManager_RadioState::RX);

    // Start downlink
    if (this->isConnected_comStatusOut_OutputPort(0)) {
        Fw::Success comStatus = Fw::Success::SUCCESS;
        this->comStatusOut_out(0, comStatus);
    }
}

bool SbandRadioManager ::configureRadio() {
    Fw::ParamValid valid;
    U32 freqHz = this->paramGet_FREQUENCY_HZ(valid);
    I8 txPower = this->paramGet_TX_POWER_DBM(valid);
    U8 sf = this->paramGet_SPREADING_FACTOR(valid);
    SbandBandwidth bandwidth = this->paramGet_BANDWIDTH(valid);
    U8 codingRate = this->paramGet_CODING_RATE(valid);

    if (this->m_fallbackActive) {
        freqHz = static_cast<U32>(SbandDefaults::FREQUENCY_HZ);
        txPower = static_cast<I8>(SbandDefaults::TX_POWER_DBM);
        sf = static_cast<U8>(SbandDefaults::SPREADING_FACTOR);
        bandwidth = SbandBandwidth::BW_812_KHZ;
        codingRate = static_cast<U8>(SbandDefaults::CODING_RATE);
    }

    // Validate before touching the chip
    if (freqHz < 2400000000U || freqHz > 2500000000U) return false;
    if (txPower < TX_POWER_MIN || txPower > TX_POWER_MAX) return false;
    if (sf < 5 || sf > 12) return false;
    if (codingRate < 1 || codingRate > 4) return false;

    U8 bwCode = 0;
    switch (bandwidth.e) {
        case SbandBandwidth::BW_203_KHZ:  bwCode = 0x34; break;
        case SbandBandwidth::BW_406_KHZ:  bwCode = 0x26; break;
        case SbandBandwidth::BW_812_KHZ:  bwCode = 0x18; break;
        case SbandBandwidth::BW_1625_KHZ: bwCode = 0x0A; break;
        default: return false;
    }

    // 1. Standby (RC oscillator) - required before changing packet type
    if (!this->setStandby()) return false;

    // 2. Packet type LoRa
    {
        const U8 params[1] = {PACKET_TYPE_LORA};
        if (!this->sendCommand(OP_SET_PACKET_TYPE, params, sizeof(params))) return false;
    }

    // 3. RF frequency: steps = f * 2^18 / 52 MHz
    {
        const U32 steps = static_cast<U32>((static_cast<U64>(freqHz) << 18) / F_XTAL);
        const U8 params[3] = {static_cast<U8>((steps >> 16) & 0xFF), static_cast<U8>((steps >> 8) & 0xFF),
                              static_cast<U8>(steps & 0xFF)};
        if (!this->sendCommand(OP_SET_RF_FREQUENCY, params, sizeof(params))) return false;
    }

    // 4. TX and RX both start at buffer offset 0
    {
        const U8 params[2] = {BUFFER_BASE_ADDR, BUFFER_BASE_ADDR};
        if (!this->sendCommand(OP_SET_BUFFER_BASE_ADDR, params, sizeof(params))) return false;
    }

    // 5. Modulation: SF, BW, CR
    {
        const U8 params[3] = {static_cast<U8>(sf << 4), bwCode, codingRate};
        if (!this->sendCommand(OP_SET_MODULATION_PARAMS, params, sizeof(params))) return false;
    }

    // 6. Required register writes after SetModulationParams (user guide, LoRa operation)
    {
        U8 sfReg = 0x32;              // SF9-SF12
        if (sf <= 6) sfReg = 0x1E;    // SF5-SF6
        else if (sf <= 8) sfReg = 0x37;  // SF7-SF8
        if (!this->writeRegister(REG_SF_ADDITIONAL, sfReg)) return false;
        if (!this->writeRegister(REG_FREQ_ERR_COMP, 0x01)) return false;
    }

    // 7. Packet params (payload length is set again before each TX)
    if (!this->setPacketParams(MAX_PAYLOAD)) return false;

    // 8. TX power and ramp
    {
        const U8 params[2] = {static_cast<U8>(txPower - TX_POWER_MIN), RAMP_20_US};
        if (!this->sendCommand(OP_SET_TX_PARAMS, params, sizeof(params))) return false;
    }

    // 9. IRQs we care about; also routed to DIO1 in case it gets wired later
    {
        const U16 mask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HEADER_ERROR | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
        const U8 params[8] = {static_cast<U8>(mask >> 8), static_cast<U8>(mask & 0xFF),   // IRQ mask
                              static_cast<U8>(mask >> 8), static_cast<U8>(mask & 0xFF),   // DIO1
                              0x00, 0x00,                                                 // DIO2
                              0x00, 0x00};                                                // DIO3
        if (!this->sendCommand(OP_SET_DIO_IRQ_PARAMS, params, sizeof(params))) return false;
    }

    // 10. Clear anything left over, then listen
    if (!this->clearIrq(IRQ_ALL)) return false;
    if (!this->setRxContinuous()) return false;

    this->log_ACTIVITY_HI_ConfigApplied(freqHz, txPower, sf, bandwidth, codingRate);
    return true;
}

void SbandRadioManager ::finishTx(bool success) {
    if (!this->m_txInProgress) return;

    this->m_txInProgress = false;

    const bool wasTest = this->m_txIsTest;
    this->m_txIsTest = false;

    // A test packet uses our own memory - never hand it to the com stack
    if (!wasTest && this->isConnected_dataReturnOut_OutputPort(0)) {
        this->dataReturnOut_out(0, this->m_txBuffer, this->m_txContext);
    }

    if (success) {
        this->m_framesTransmitted++;
        this->tlmWrite_FramesSent(this->m_framesTransmitted);
    } else {
        this->m_txFailures++;
        this->tlmWrite_TxFailures(this->m_txFailures);
    }

    bool radioOk = false;

    if (this->m_radioState == SbandRadioManager_RadioState::TX ||
        this->m_radioState == SbandRadioManager_RadioState::RX) {
        // After TxDone the chip drops to standby on its own; put it back in RX
        radioOk = this->setPacketParams(MAX_PAYLOAD) && this->clearIrq(IRQ_ALL) && this->setRxContinuous();

        if (radioOk) {
            this->setState(SbandRadioManager_RadioState::RX);
        } else {
            this->setState(SbandRadioManager_RadioState::ERROR);
        }
    }

    if (wasTest) {
        if (success) {
            this->log_ACTIVITY_HI_TestPacketSent(this->m_testPacketCount);
        }
        return;  // no comStatus for test packets
    }

    if (radioOk) {
        // Listen for a while before asking ComQueue for the next frame
        this->m_statusPending = true;
    } else if (this->isConnected_comStatusOut_OutputPort(0)) {
        Fw::Success comStatus = Fw::Success::FAILURE;
        this->comStatusOut_out(0, comStatus);
    }
}

void SbandRadioManager ::checkTxDone() {
    this->m_txTicks++;

    U16 irq = 0;
    if (!this->getIrqStatus(irq)) {
        this->finishTx(false);
        return;
    }

    if ((irq & IRQ_TX_DONE) != 0) {
        this->clearIrq(IRQ_TX_DONE);
        this->finishTx(true);
        this->trySendPendingStatus();  // next frame now, if the period has already passed
        return;
    }

    Fw::ParamValid valid;
    const U32 timeout = this->paramGet_TX_TIMEOUT_TICKS(valid);

    if (this->m_txTicks >= timeout) {
        this->log_WARNING_HI_TxTimeout();
        this->finishTx(false);
    }
}

void SbandRadioManager ::pollRx() {
    U16 irq = 0;
    if (!this->getIrqStatus(irq)) return;

    if ((irq & IRQ_RX_DONE) == 0) {
        // A bad header with no RxDone still leaves flags behind
        if ((irq & IRQ_HEADER_ERROR) != 0) {
            this->dropRx();
        }
        return;
    }

    if ((irq & (IRQ_CRC_ERROR | IRQ_HEADER_ERROR)) != 0) {
        this->m_rxCrcErrors++;
        this->tlmWrite_RxCrcErrors(this->m_rxCrcErrors);
        this->clearIrq(IRQ_ALL);
        return;
    }

    // GetRxBufferStatus -> [payloadLength, rxStartBufferPointer]
    U8 bufStatus[2] = {0};
    if (!this->readCommand(OP_GET_RX_BUFFER_STATUS, bufStatus, sizeof(bufStatus))) {
        this->dropRx();
        return;
    }
    const U8 rxLen = bufStatus[0];
    const U8 rxStart = bufStatus[1];

    if (rxLen <= RH_HEADER_LEN) {
        this->dropRx();
        return;
    }

    U8 rhHeader[RH_HEADER_LEN] = {0};
    if (!this->readBuffer(rxStart, rhHeader, RH_HEADER_LEN) ||
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

    if (!this->readBuffer(static_cast<U8>(rxStart + RH_HEADER_LEN), rxBuffer.getData(), frameLen)) {
        if (this->isConnected_deallocate_OutputPort(0)) {
            this->deallocate_out(0, rxBuffer);
        }
        this->dropRx();
        return;
    }

    rxBuffer.setSize(frameLen);

    // GetPacketStatus (LoRa) -> [rssiSync, snr, ...]; RSSI = -rssiSync/2, SNR = snr/4
    U8 pktStatus[2] = {0};
    if (this->readCommand(OP_GET_PACKET_STATUS, pktStatus, sizeof(pktStatus))) {
        const I8 snr = static_cast<I8>(static_cast<I8>(pktStatus[1]) / 4);
        I16 rssi = static_cast<I16>(-static_cast<I16>(pktStatus[0]) / 2);
        if (snr < 0) {
            rssi = static_cast<I16>(rssi + snr);
        }

        this->m_lastSnr = snr;
        this->m_lastRssi = rssi;
        this->tlmWrite_LastSnr(this->m_lastSnr);
        this->tlmWrite_LastRssi(this->m_lastRssi);
    }

    // Done with the radio for this packet - ready for the next one
    this->clearIrq(IRQ_ALL);

    this->m_framesReceived++;
    this->tlmWrite_FramesReceived(this->m_framesReceived);

    // Debug: show up to the first RX_DEBUG_MAX_BYTES bytes as hex in the GDS
    // (must happen BEFORE the buffer is sent away or freed)
    char hex[128] = {0};
    const FwSizeType showLen = (frameLen < RX_DEBUG_MAX_BYTES) ? frameLen : RX_DEBUG_MAX_BYTES;
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
    } else if (this->isConnected_deallocate_OutputPort(0)) {
        this->deallocate_out(0, rxBuffer);
    }
}

void SbandRadioManager ::dropRx() {
    this->m_rxDropped++;
    this->tlmWrite_RxDropped(this->m_rxDropped);
    this->clearIrq(IRQ_ALL);
}

void SbandRadioManager ::stopTestTx() {
    this->m_testActive = false;
    this->m_testTicksRemaining = 0;
    // Standby ends the test transmission; configureRadio restores power and RX
    if (this->setStandby() && this->configureRadio()) {
        this->setState(SbandRadioManager_RadioState::RX);
    } else {
        this->setState(SbandRadioManager_RadioState::ERROR);
    }
    this->log_ACTIVITY_HI_TestTxStopped();
}

void SbandRadioManager ::trySendPendingStatus() {
    if (!this->m_statusPending) {
        return;
    }
    Fw::ParamValid valid;
    if (this->m_ticksSinceTx >= this->paramGet_TX_PERIOD_TICKS(valid)) {
        this->m_statusPending = false;
        if (this->isConnected_comStatusOut_OutputPort(0)) {
            Fw::Success comStatus = Fw::Success::SUCCESS;
            this->comStatusOut_out(0, comStatus);
        }
    }
}

void SbandRadioManager ::parameterUpdated(FwPrmIdType id) {
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

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void SbandRadioManager ::dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    this->transmitFrame_internalInterfaceInvoke(data, context);
}

void SbandRadioManager ::dataReturnIn_handler(FwIndexType portNum,
                                              Fw::Buffer& data,
                                              const ComCfg::FrameContext& context) {
    if (this->isConnected_deallocate_OutputPort(0)) {
        this->deallocate_out(0, data);
    }
}

void SbandRadioManager ::pingIn_handler(FwIndexType portNum, U32 key) {
    // Reply on our own thread: proves the radio task is alive and processing its queue
    if (this->isConnected_pingOut_OutputPort(0)) {
        this->pingOut_out(0, key);
    }
}

void SbandRadioManager ::run_handler(FwIndexType portNum, U32 context) {
    this->m_ticksSinceUplink++;
    this->m_ticksSinceTx++;

    switch (this->m_radioState.e) {
        case SbandRadioManager_RadioState::UNINITIALIZED:
            this->initRadio();
            break;

        case SbandRadioManager_RadioState::RX: {
            Fw::ParamValid valid;
            const U32 limit = this->paramGet_LINK_LOSS_TICKS(valid);
            if (limit > 0 && !this->m_fallbackActive && this->m_ticksSinceUplink >= limit) {
                this->log_WARNING_HI_LinkLossFallback(this->m_ticksSinceUplink);
                this->m_fallbackActive = true;
                this->m_ticksSinceUplink = 0;
                this->m_configPending = true;
            }

            if (this->m_configPending) {
                this->m_configPending = false;
                this->setState(SbandRadioManager_RadioState::IDLE);
                if (!this->configureRadio()) {
                    this->setState(SbandRadioManager_RadioState::ERROR);
                    break;
                }
                this->setState(SbandRadioManager_RadioState::RX);
            }

            this->pollRx();
            this->trySendPendingStatus();
            break;
        }

        case SbandRadioManager_RadioState::TX:
            this->checkTxDone();
            break;

        case SbandRadioManager_RadioState::IDLE:
            // Reconfiguring, or holding a TX_TEST
            if (this->m_testActive) {
                if (this->m_testTicksRemaining > 0) {
                    this->m_testTicksRemaining--;
                }
                if (this->m_testTicksRemaining == 0) {
                    this->stopTestTx();
                }
            }
            break;

        case SbandRadioManager_RadioState::ERROR: {
            this->m_errorTicks++;
            Fw::ParamValid valid;
            if (this->m_errorTicks >= this->paramGet_ERROR_RETRY_TICKS(valid)) {
                this->m_recoveryAttempts++;
                this->log_WARNING_LO_RadioRecoveryAttempt(this->m_recoveryAttempts);
                this->m_statusPending = false;
                this->setState(SbandRadioManager_RadioState::UNINITIALIZED);
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

void SbandRadioManager ::GET_STATUS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    U8 status = 0;
    if (this->getStatus(status)) {
        this->log_ACTIVITY_HI_StatusValue(status);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

void SbandRadioManager ::READ_REGISTER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U16 addr) {
    U8 value = 0;
    if (this->readRegister(addr, value)) {
        this->log_ACTIVITY_HI_RegisterValue(addr, value);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

void SbandRadioManager ::WRITE_REGISTER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U16 addr, U8 value) {
    if (this->writeRegister(addr, value)) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

void SbandRadioManager ::RESET_RADIO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_ACTIVITY_HI_RadioResetStarted();

    if (this->m_txInProgress) {
        this->finishTx(false);
    }

    this->m_statusPending = false;
    this->m_testActive = false;
    this->m_testTicksRemaining = 0;
    this->setState(SbandRadioManager_RadioState::UNINITIALIZED);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SbandRadioManager ::TX_TEST_cmdHandler(FwOpcodeType opCode,
                                            U32 cmdSeq,
                                            const Cfl740_f26::SbandTestMode& mode,
                                            I8 powerDbm,
                                            U16 durationSec) {
    // Only start from a quiet RX state
    if (this->m_radioState != SbandRadioManager_RadioState::RX || this->m_txInProgress || this->m_testActive) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::BUSY);
        return;
    }
    if (powerDbm < TX_POWER_MIN || powerDbm > TX_POWER_MAX || durationSec == 0 ||
        durationSec > TEST_MAX_DURATION_SEC) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }

    // Standby -> test power -> continuous TX
    const U8 txParams[2] = {static_cast<U8>(powerDbm - TX_POWER_MIN), RAMP_20_US};
    const U8 opcode = (mode == SbandTestMode::CW) ? OP_SET_TX_CONT_WAVE : OP_SET_TX_CONT_PREAMBLE;

    if (!this->setStandby() || !this->sendCommand(OP_SET_TX_PARAMS, txParams, sizeof(txParams)) ||
        !this->sendCommand(opcode, nullptr, 0)) {
        this->setState(SbandRadioManager_RadioState::ERROR);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    this->m_testActive = true;
    this->m_testTicksRemaining = static_cast<U32>(durationSec) * 10;  // run is 10 Hz
    this->setState(SbandRadioManager_RadioState::IDLE);
    this->log_ACTIVITY_HI_TestTxStarted(mode, powerDbm, durationSec);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SbandRadioManager ::TX_TEST_STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    if (!this->m_testActive) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
        return;
    }
    this->stopTestTx();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SbandRadioManager ::TEST_PACKET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    if (this->m_radioState != SbandRadioManager_RadioState::RX || this->m_txInProgress || this->m_testActive) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::BUSY);
        return;
    }

    this->m_testPacketCount++;
    const int n = snprintf(reinterpret_cast<char*>(this->m_testPayload), sizeof(this->m_testPayload),
                           "SBAND TEST %lu", static_cast<unsigned long>(this->m_testPacketCount));
    const FwSizeType len = (n > 0 && static_cast<FwSizeType>(n) < sizeof(this->m_testPayload))
                               ? static_cast<FwSizeType>(n)
                               : sizeof(this->m_testPayload) - 1;

    Fw::Buffer testBuffer(this->m_testPayload, len);
    ComCfg::FrameContext context;
    this->m_txIsTest = true;
    this->transmitFrame_internalInterfaceHandler(testBuffer, context);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined internal interfaces
// ----------------------------------------------------------------------

void SbandRadioManager ::transmitFrame_internalInterfaceHandler(const Fw::Buffer& data,
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

    const FwSizeType size = this->m_txBuffer.getSize();

    if (this->m_radioState != SbandRadioManager_RadioState::RX || size == 0 ||
        size > static_cast<FwSizeType>(MAX_PAYLOAD - RH_HEADER_LEN)) {
        this->finishTx(false);
        return;
    }

    const U8 rhHeader[RH_HEADER_LEN] = {RH_ADDR_BROADCAST, RH_ADDR_FLIGHT, 0x00, 0x00};

    // Standby -> length -> buffer -> clear IRQs -> TX
    if (!this->setStandby() ||
        !this->setPacketParams(static_cast<U8>(size + RH_HEADER_LEN)) ||
        !this->writeBuffer(BUFFER_BASE_ADDR, rhHeader, RH_HEADER_LEN, this->m_txBuffer.getData(), size) ||
        !this->clearIrq(IRQ_ALL) ||
        !this->setTx()) {
        this->finishTx(false);
        return;
    }

    this->m_ticksSinceTx = 0;
    this->setState(SbandRadioManager_RadioState::TX);
}

}  // namespace Cfl740_f26
