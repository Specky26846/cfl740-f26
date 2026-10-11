// ======================================================================
// \title  RadioManager.cpp
// \author yuktivijay
// \brief  cpp file for RadioManager component implementation class
// ======================================================================

#include <cstring>
#include "CubesatRef/Components/RadioManager/RadioManager.hpp"

namespace Managers {

RadioManager::RadioManager(const char* const compName) : RadioManagerComponentBase(compName) {}

RadioManager::~RadioManager() {}

// sum1/sum2 mod 255, standard Fletcher-16
U16 RadioManager::fletcher16(const U8* data, U32 len) {
    U16 sum1 = 0;
    U16 sum2 = 0;
    for (U32 i = 0; i < len; i++) {
        sum1 = static_cast<U16>((sum1 + data[i]) % 255);
        sum2 = static_cast<U16>((sum2 + sum1) % 255);
    }
    return static_cast<U16>((sum2 << 8) | sum1);
}

void RadioManager::emitTelemetry() {
    this->tlmWrite_PacketsReceived(this->m_packetsReceived);
    this->tlmWrite_PacketsTransmitted(this->m_packetsTransmitted);
    this->tlmWrite_LastRssi(this->m_lastRssi);
    this->tlmWrite_RadioStateChannel(this->m_radioState);
    this->tlmWrite_TxPower(this->m_txPower);
    this->tlmWrite_Frequency(this->m_frequency);
    this->tlmWrite_SpreadingFactor(this->m_spreadingFactor);
    this->tlmWrite_Bandwidth(this->m_bandwidth);
    this->tlmWrite_CodingRate(this->m_codingRate);
}

void RadioManager::GET_CONFIG_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_ACTIVITY_HI_ConfigReport(this->m_frequency, this->m_txPower, this->m_spreadingFactor);
    this->emitTelemetry();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void RadioManager::SET_TX_POWER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, I8 power) {
    this->m_txPower = power;

    if (this->isConnected_spiReadWrite_OutputPort(0)) {
        U8 writeData[2] = {0x09, static_cast<U8>(power)};
        U8 readData[2] = {};
        Fw::Buffer writeBuffer(writeData, sizeof(writeData));
        Fw::Buffer readBuffer(readData, sizeof(readData));
        this->spiReadWrite_out(0, writeBuffer, readBuffer);
    }

    Fw::LogStringArg paramName("TX Power");
    this->log_ACTIVITY_HI_ConfigUpdated(paramName);
    this->tlmWrite_TxPower(this->m_txPower);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void RadioManager::SET_FREQUENCY_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U32 frequency) {
    this->m_frequency = frequency;

    if (this->isConnected_spiReadWrite_OutputPort(0)) {
        U8 writeData[4] = {0x06,
                           static_cast<U8>((frequency >> 16) & 0xFF),
                           static_cast<U8>((frequency >> 8) & 0xFF),
                           static_cast<U8>(frequency & 0xFF)};
        U8 readData[4] = {};
        Fw::Buffer writeBuffer(writeData, sizeof(writeData));
        Fw::Buffer readBuffer(readData, sizeof(readData));
        this->spiReadWrite_out(0, writeBuffer, readBuffer);
    }

    Fw::LogStringArg paramName("Frequency");
    this->log_ACTIVITY_HI_ConfigUpdated(paramName);
    this->tlmWrite_Frequency(this->m_frequency);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void RadioManager::SET_LORA_PARAMS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                               U8 spreadingFactor, U8 bandwidth, U8 codingRate) {
    this->m_spreadingFactor = spreadingFactor;
    this->m_bandwidth = bandwidth;
    this->m_codingRate = codingRate;

    if (this->isConnected_spiReadWrite_OutputPort(0)) {
        U8 writeData[4] = {0x1D, spreadingFactor, bandwidth, codingRate};
        U8 readData[4] = {};
        Fw::Buffer writeBuffer(writeData, sizeof(writeData));
        Fw::Buffer readBuffer(readData, sizeof(readData));
        this->spiReadWrite_out(0, writeBuffer, readBuffer);
    }

    Fw::LogStringArg paramName("LoRa Params");
    this->log_ACTIVITY_HI_ConfigUpdated(paramName);
    this->tlmWrite_SpreadingFactor(this->m_spreadingFactor);
    this->tlmWrite_Bandwidth(this->m_bandwidth);
    this->tlmWrite_CodingRate(this->m_codingRate);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// wraps data as a DAP (Data Application Packet) inside a RAP (Radio Application Packet)
// per the MXL radio packet format (docs/MXL_RAP_Format.md) and sends the whole frame
// over SPI. HMAC is stubbed to zero - see RAP_PRIMARY_ID note in the header for why.
void RadioManager::TRANSMIT_PACKET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                                const Fw::CmdStringArg& data) {
    U32 dataLen = data.length();
    if (dataLen > MAX_DAP_DATA) {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }

    this->m_radioState = RadioState::TX;
    this->tlmWrite_RadioStateChannel(this->m_radioState);

    // --- build the DAP (goes in the RAP's data section) ---
    U32 dapLen = DAP_HEADER_LEN + dataLen;
    U8 dap[DAP_HEADER_LEN + MAX_DAP_DATA] = {};
    dap[0] = DAP_PRIMARY_ID;
    dap[1] = static_cast<U8>(dapLen & 0xFF);
    dap[2] = static_cast<U8>((dapLen >> 8) & 0xFF);
    dap[3] = 0;  // File Number - not a real file transfer, single generic buffer
    dap[4] = 0;
    dap[5] = 0;  // File Part
    dap[6] = 0;
    dap[7] = 1;  // Total Parts in File - always 1, we don't fragment across DAPs
    dap[8] = 0;
    memcpy(&dap[DAP_HEADER_LEN], data.toChar(), dataLen);

    // --- wrap the DAP in a RAP ---
    U32 rapLen = RAP_HEADER_LEN + RAP_TRAILER_LEN + dapLen;
    U8 rap[RAP_HEADER_LEN + RAP_TRAILER_LEN + DAP_HEADER_LEN + MAX_DAP_DATA] = {};
    rap[0] = static_cast<U8>((RAP_SYNC >> 8) & 0xFF);  // AB
    rap[1] = static_cast<U8>(RAP_SYNC & 0xFF);          // CD
    rap[2] = static_cast<U8>(RAP_PRIMARY_ID & 0xFF);
    rap[3] = static_cast<U8>((RAP_PRIMARY_ID >> 8) & 0xFF);
    rap[4] = static_cast<U8>((RAP_SECONDARY_ID >> 8) & 0xFF);  // big-endian per spec
    rap[5] = static_cast<U8>(RAP_SECONDARY_ID & 0xFF);
    rap[6] = RAP_FLAG_TYPE_DAP;
    rap[7] = static_cast<U8>(rapLen & 0xFF);
    rap[8] = static_cast<U8>((rapLen >> 8) & 0xFF);

    U16 headerChecksum = fletcher16(rap, RAP_HEADER_LEN);
    rap[9] = static_cast<U8>(headerChecksum & 0xFF);
    rap[10] = static_cast<U8>((headerChecksum >> 8) & 0xFF);

    memcpy(&rap[RAP_HEADER_LEN + 2], dap, dapLen);

    U16 dataChecksum = fletcher16(dap, dapLen);
    U32 checksumOffset = RAP_HEADER_LEN + 2 + dapLen;
    rap[checksumOffset] = static_cast<U8>(dataChecksum & 0xFF);
    rap[checksumOffset + 1] = static_cast<U8>((dataChecksum >> 8) & 0xFF);
    // rap[checksumOffset+2 .. +5] (HMAC) already zeroed above - TODO real HMAC-SHA1

    if (this->isConnected_spiReadWrite_OutputPort(0)) {
        U8 readData[RAP_HEADER_LEN + RAP_TRAILER_LEN + DAP_HEADER_LEN + MAX_DAP_DATA] = {};
        Fw::Buffer writeBuffer(rap, rapLen);
        Fw::Buffer readBuffer(readData, rapLen);
        this->spiReadWrite_out(0, writeBuffer, readBuffer);
    }

    this->m_packetsTransmitted++;
    this->log_ACTIVITY_HI_PacketTransmitted(rapLen);
    this->tlmWrite_PacketsTransmitted(this->m_packetsTransmitted);

    this->m_radioState = RadioState::IDLE;
    this->tlmWrite_RadioStateChannel(this->m_radioState);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void RadioManager::GET_TELEMETRY_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->emitTelemetry();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void RadioManager::RADIO_RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->m_packetsReceived = 0;
    this->m_packetsTransmitted = 0;
    this->m_lastRssi = 0;
    this->m_radioState = RadioState::IDLE;

    if (this->isConnected_spiReadWrite_OutputPort(0)) {
        U8 writeData[2] = {0x01, 0x80};
        U8 readData[2] = {};
        Fw::Buffer writeBuffer(writeData, sizeof(writeData));
        Fw::Buffer readBuffer(readData, sizeof(readData));
        this->spiReadWrite_out(0, writeBuffer, readBuffer);
    }

    this->log_ACTIVITY_HI_RadioResetComplete();
    this->emitTelemetry();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void RadioManager::run_handler(FwIndexType portNum, U32 context) {
    if (this->isConnected_spiReadWrite_OutputPort(0)) {
        U8 writeData[2] = {0x12, 0x00};
        U8 readData[2] = {0x00, 0x00};
        Fw::Buffer writeBuffer(writeData, sizeof(writeData));
        Fw::Buffer readBuffer(readData, sizeof(readData));

        this->spiReadWrite_out(0, writeBuffer, readBuffer);
        this->emitTelemetry();
    }
}

}  // namespace Managers
