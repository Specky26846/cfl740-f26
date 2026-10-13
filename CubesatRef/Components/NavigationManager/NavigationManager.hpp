// NavigationManager.hpp

#ifndef Managers_NavigationManager_HPP
#define Managers_NavigationManager_HPP

#include "CubesatRef/Components/NavigationManager/NavigationManagerComponentAc.hpp"

namespace Managers {

class NavigationManager final : public NavigationManagerComponentBase {
  public:
    NavigationManager(const char* const compName);
    ~NavigationManager();
    void configure();

  private:
    bool m_hasFix = false;
    U32 m_readCount = 0;
    U32 m_missedPackets = 0;  // consecutive ticks with no gga returned
    static constexpr U8  MIN_SATELLITES_FOR_FIX = 4;
    static constexpr U32 READ_LOG_INTERVAL = 10;
    // 1 miss is normal timing jitter - fault after 5 consecutive misses at 1Hz (~5s with no data)
    static constexpr U32 MAX_MISSED_PACKETS = 5;

    // nmea sentence assembler - sentences can straddle i2c chunks and run ticks, so this persists
    static constexpr U32 NMEA_MAX_LEN = 96;  // spec max is 82 incl $ and crlf, leave some slack
    char m_nmeaLine[NMEA_MAX_LEN] = {};
    U32 m_nmeaLen = 0;

    // last parsed values - gga gives position/alt/sats, rmc gives speed
    F64 m_lat = 0;
    F64 m_lon = 0;
    F32 m_alt = 0;
    F32 m_speed = 0;
    U8  m_numSats = 0;

    // state machine action handlers
    void Managers_NavigationManagerStateMachine_action_doInit(
        SmId smId, Managers_NavigationManagerStateMachine::Signal signal) override;
    void Managers_NavigationManagerStateMachine_action_doRead(
        SmId smId, Managers_NavigationManagerStateMachine::Signal signal) override;
    void Managers_NavigationManagerStateMachine_action_doFaultRecovery(
        SmId smId, Managers_NavigationManagerStateMachine::Signal signal) override;
    void Managers_NavigationManagerStateMachine_action_doSimRead(
        SmId smId, Managers_NavigationManagerStateMachine::Signal signal) override;

    // command handlers
    void GPS_RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void run_handler(FwIndexType portNum, U32 context) override;

    // drains nmea output from the neo-m9n and parses gga/rmc into position/velocity
    Drv::I2cStatus readGpsData(F64& lat, F64& lon, F32& alt, F32& speed, U8& numSats, bool& packetFound);
    // adds one byte to the sentence assembler, sets ggaFound when a valid gga completes
    void feedNmeaByte(U8 byte, bool& ggaFound);
    // checksum + dispatch a complete sentence, returns true if it was a gga
    bool parseNmeaSentence(char* line);
    // sends an nmea sentence (body without $ or *checksum) over i2c
    void sendNmea(const char* body);
    // sends a ubx config message over i2c
    void sendUbxCfg(const U8* payload, U8 payloadLen);
    // writes all gps telemetry channels and tracks fix state
    void reportGpsTelemetry(F64 lat, F64 lon, F32 alt, F32 speed, U8 numSats);
};

}  // namespace Managers

#endif
