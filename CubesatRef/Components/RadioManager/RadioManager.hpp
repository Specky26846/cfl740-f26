// ======================================================================
// \title  RadioManager.hpp
// \author yuktivijay
// \brief  hpp file for RadioManager component implementation class
// ======================================================================

#ifndef Managers_RadioManager_HPP
#define Managers_RadioManager_HPP

#include "CubesatRef/Components/RadioManager/RadioManagerComponentAc.hpp"

namespace Managers {

class RadioManager final : public RadioManagerComponentBase {
  public:
    RadioManager(const char* const compName);
    ~RadioManager();

  private:
    U32 m_packetsReceived = 0;
    U32 m_packetsTransmitted = 0;
    I32 m_lastRssi = 0;
    RadioState m_radioState = RadioState::IDLE;

    I8 m_txPower = 14;
    U32 m_frequency = 915;
    U8 m_spreadingFactor = 7;
    U8 m_bandwidth = 7;
    U8 m_codingRate = 1;

    // MXL Radio Application Packet (RAP) format - see docs/MXL_RAP_Format.md
    // TODO: RAP_PRIMARY_ID/RAP_SECONDARY_ID/DAP_PRIMARY_ID are placeholders - real
    // values come from whatever ground-station/mission registry assigns satellite
    // and application IDs, which this reference system has no visibility into.
    static constexpr U16 RAP_SYNC = 0xABCD;
    static constexpr U16 RAP_PRIMARY_ID = 0x0001;
    static constexpr U16 RAP_SECONDARY_ID = 0x0001;  // big-endian on the wire per spec
    static constexpr U8 DAP_PRIMARY_ID = 0x01;
    static constexpr U8 RAP_FLAG_TYPE_DAP = 0x1;
    static constexpr U32 RAP_HEADER_LEN = 9;   // sync(2)+primaryId(2)+secondaryId(2)+flags(1)+length(2)
    static constexpr U32 RAP_TRAILER_LEN = 8;  // headerChecksum(2)+checksum(2)+hmac(4)
    static constexpr U32 DAP_HEADER_LEN = 9;   // primaryId(1)+length(2)+fileNumber(2)+filePart(2)+totalParts(2)
    static constexpr U32 MAX_DAP_DATA = 229;   // caps RAP data section at 238 bytes per spec

    // FLETCHER 16 checksum, per spec
    static U16 fletcher16(const U8* data, U32 len);

    void GET_CONFIG_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void SET_TX_POWER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, I8 power) override;
    void SET_FREQUENCY_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U32 frequency) override;
    void SET_LORA_PARAMS_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                    U8 spreadingFactor, U8 bandwidth, U8 codingRate) override;
    void TRANSMIT_PACKET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                     const Fw::CmdStringArg& data) override;
    void GET_TELEMETRY_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void RADIO_RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void run_handler(FwIndexType portNum, U32 context) override;

    void emitTelemetry();
};

}  // namespace Managers

#endif
