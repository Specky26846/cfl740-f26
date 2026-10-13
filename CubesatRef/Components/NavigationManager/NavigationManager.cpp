// NavigationManager.cpp

#include "CubesatRef/Components/NavigationManager/NavigationManager.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>


namespace Managers {

static constexpr U8  GPS_I2C_ADDRESS = 0x42;
static constexpr U8  GPS_DATA_REG    = 0xFF;
static constexpr U32 GPS_CHUNK_SIZE  = 32;
static constexpr U32 GPS_BUFFER_SIZE = 512;

// nmea parsing
static constexpr U32 NMEA_MAX_FIELDS = 20;
static constexpr F64 KNOTS_TO_MPS    = 0.514444;

// gga field indices (field 0 is the talker+type, e.g. "GNGGA")
static constexpr U32 GGA_LAT      = 2;   // ddmm.mmmmm
static constexpr U32 GGA_NS       = 3;
static constexpr U32 GGA_LON      = 4;   // dddmm.mmmmm
static constexpr U32 GGA_EW       = 5;
static constexpr U32 GGA_QUALITY  = 6;   // 0 = no fix
static constexpr U32 GGA_NUM_SV   = 7;   // satellites used
static constexpr U32 GGA_ALT      = 9;   // altitude above msl (m)

// rmc field indices
static constexpr U32 RMC_STATUS   = 2;   // A = valid, V = warning
static constexpr U32 RMC_SPEED    = 7;   // speed over ground (knots)

NavigationManager::NavigationManager(const char* const compName) : NavigationManagerComponentBase(compName) {}

NavigationManager::~NavigationManager() {}

void NavigationManager::run_handler(FwIndexType portNum, U32 context) {
    this->navSm_sendSignal_tick();
}

// state machine actions

// poke the gps once to make sure it's actually sitting on the bus
void NavigationManager::Managers_NavigationManagerStateMachine_action_doInit(
    SmId smId, Managers_NavigationManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::navSm);
    U8 regAddr = GPS_DATA_REG;
    Fw::Buffer writeBuffer(&regAddr, sizeof(regAddr));
    U8 chunk[GPS_CHUNK_SIZE] = {};
    Fw::Buffer readBuffer(chunk, GPS_CHUNK_SIZE);

    Drv::I2cStatus status = this->busWriteRead_out(0, GPS_I2C_ADDRESS, writeBuffer, readBuffer);
    if (status == Drv::I2cStatus::I2C_OK) {
        this->log_ACTIVITY_HI_StateChange(NavigationManager_SensorState::RUNNING);
        this->navSm_sendSignal_success();
    } else {
        this->log_ACTIVITY_HI_StateChange(NavigationManager_SensorState::FAULT);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, false);
        }
        this->navSm_sendSignal_fault();
    }
}

// drain nmea from the gps, parse out position and speed, push telemetry
void NavigationManager::Managers_NavigationManagerStateMachine_action_doRead(
    SmId smId, Managers_NavigationManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::navSm);
    F64 lat = 0, lon = 0;
    F32 alt = 0, speed = 0;
    U8 sats = 0;
    bool packetFound = false;
    Drv::I2cStatus status = this->readGpsData(lat, lon, alt, speed, sats, packetFound);

    if (status != Drv::I2cStatus::I2C_OK) {
        this->log_ACTIVITY_HI_StateChange(NavigationManager_SensorState::FAULT);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, false);
        }
        this->navSm_sendSignal_fault();
        return;
    }

    // no gga is not a fault - gps only outputs at its nav rate, and a sentence can straddle ticks
    if (packetFound) {
        this->reportGpsTelemetry(lat, lon, alt, speed, sats);
    }
}

// retry the gps read, if it comes back we flip back to running
void NavigationManager::Managers_NavigationManagerStateMachine_action_doFaultRecovery(
    SmId smId, Managers_NavigationManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::navSm);
    F64 lat = 0, lon = 0;
    F32 alt = 0, speed = 0;
    U8 sats = 0;
    bool packetFound = false;
    Drv::I2cStatus status = this->readGpsData(lat, lon, alt, speed, sats, packetFound);
    if (status == Drv::I2cStatus::I2C_OK && packetFound) {
        this->reportGpsTelemetry(lat, lon, alt, speed, sats);
        this->log_ACTIVITY_HI_StateChange(NavigationManager_SensorState::RUNNING);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, true);
        }
        this->navSm_sendSignal_success();
    } else {
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, false);
        }
    }
}

// fake gps data (ann arbor coords) when there's no hardware
void NavigationManager::Managers_NavigationManagerStateMachine_action_doSimRead(
    SmId smId, Managers_NavigationManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::navSm);
    this->reportGpsTelemetry(42.2808, -83.7430, 270.0f, 0.0f, 8);
}

// command handlers

void NavigationManager::GPS_RESET_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->m_hasFix = false;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void NavigationManager::ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_ACTIVITY_HI_StateChange(NavigationManager_SensorState::SIM);
    this->navSm_sendSignal_enableSim();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void NavigationManager::DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_ACTIVITY_HI_StateChange(NavigationManager_SensorState::INIT);
    this->navSm_sendSignal_disableSim();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// helpers

// push all the gps telemetry channels and log when we acquire or lose a fix
void NavigationManager::reportGpsTelemetry(F64 lat, F64 lon, F32 alt, F32 speed, U8 numSats) {
    bool currentFix = (numSats >= MIN_SATELLITES_FOR_FIX);

    this->tlmWrite_Latitude(lat);
    this->tlmWrite_Longitude(lon);
    this->tlmWrite_Altitude(alt);
    this->tlmWrite_GroundSpeed(speed);
    this->tlmWrite_NumSatellites(numSats);

    // throttle the event log - gga fires every second and event bandwidth is limited
    if (++m_readCount % READ_LOG_INTERVAL == 0) {
        this->log_ACTIVITY_LO_GpsReading(lat, lon, alt, numSats);
    }

    if (currentFix && !this->m_hasFix) {
        this->m_hasFix = true;
        this->log_ACTIVITY_HI_GpsFixAcquired(numSats);
    } else if (!currentFix && this->m_hasFix) {
        this->m_hasFix = false;
        this->log_WARNING_HI_GpsFixLost();
    }
}

// ubx frame: 0xb5 0x62 (sync), class 0x06, id 0x8a (cfg-valset), len, payload, then fletcher checksum
void NavigationManager::sendUbxCfg(const U8* payload, U8 payloadLen) {
    U8 msg[64] = {};
    msg[0] = 0xB5;   // ubx sync char 1
    msg[1] = 0x62;   // ubx sync char 2
    msg[2] = 0x06;   // class: cfg
    msg[3] = 0x8A;   // id: valset
    msg[4] = payloadLen;
    msg[5] = 0x00;   // length high byte (always 0 for short configs)
    memcpy(&msg[6], payload, payloadLen);

    // fletcher checksum covers everything from the class byte onward (not the two sync bytes)
    U8 ck_a = 0, ck_b = 0;
    for (U8 i = 2; i < 6 + payloadLen; i++) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    msg[6 + payloadLen]     = ck_a;
    msg[6 + payloadLen + 1] = ck_b;

    Fw::Buffer writeBuffer(msg, 6 + payloadLen + 2);
    this->busWrite_out(0, GPS_I2C_ADDRESS, writeBuffer);
}

// wrap a body in $...*hh\r\n and write it to the gps
void NavigationManager::sendNmea(const char* body) {
    U8 cs = 0;
    for (const char* c = body; *c != '\0'; c++) {
        cs ^= static_cast<U8>(*c);
    }
    char msg[NMEA_MAX_LEN] = {};
    int len = snprintf(msg, sizeof(msg), "$%s*%02X\r\n", body, cs);
    FW_ASSERT(len > 0 && static_cast<U32>(len) < sizeof(msg), len);

    Fw::Buffer writeBuffer(reinterpret_cast<U8*>(msg), static_cast<U32>(len));
    this->busWrite_out(0, GPS_I2C_ADDRESS, writeBuffer);
}

void NavigationManager::configure() {
    // tell the gps we're a cubesat, airborne dynamic model under 4g
    // using default NEO M9N dynamic model (0)
    U8 payload[] = {
        0x00, 0x03, 0x00, 0x00,
        0x21, 0x00, 0x11, 0x20,
        0x08
    };

    // we only need gga + rmc - turn the rest off on the i2c port so a full epoch fits in one read
    // pubx,40 rates are per port: ddc(i2c), uart1, uart2, usb, spi, reserved - leave the others at default 1
    this->sendNmea("PUBX,40,GSV,0,1,1,1,1,0");
    this->sendNmea("PUBX,40,GSA,0,1,1,1,1,0");
    this->sendNmea("PUBX,40,GLL,0,1,1,1,1,0");
    this->sendNmea("PUBX,40,VTG,0,1,1,1,1,0");
}

static I32 hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// nmea coords are ddmm.mmmm / dddmm.mmmm plus a hemisphere letter - convert to signed decimal degrees
static bool parseNmeaCoord(const char* value, const char* hemi, F64& out) {
    if (value[0] == '\0' || hemi[0] == '\0') return false;
    F64 raw = strtod(value, nullptr);
    F64 degrees = static_cast<F64>(static_cast<I32>(raw / 100.0));
    F64 minutes = raw - degrees * 100.0;
    out = degrees + minutes / 60.0;
    if (hemi[0] == 'S' || hemi[0] == 'W') out = -out;
    return true;
}

// sentences look like $GNGGA,...*hh - build one up byte by byte and parse when the line ends
void NavigationManager::feedNmeaByte(U8 byte, bool& ggaFound) {
    // '$' always starts a fresh sentence, even if the last one got cut off
    if (byte == '$') {
        this->m_nmeaLine[0] = '$';
        this->m_nmeaLen = 1;
        return;
    }
    // still waiting for a '$'
    if (this->m_nmeaLen == 0) return;

    if (byte == '\r' || byte == '\n') {
        this->m_nmeaLine[this->m_nmeaLen] = '\0';
        if (this->parseNmeaSentence(this->m_nmeaLine)) {
            ggaFound = true;
        }
        this->m_nmeaLen = 0;
        return;
    }

    // too long to be real nmea - drop it and wait for the next '$'
    if (this->m_nmeaLen >= NMEA_MAX_LEN - 1) {
        this->m_nmeaLen = 0;
        return;
    }
    this->m_nmeaLine[this->m_nmeaLen++] = static_cast<char>(byte);
}

bool NavigationManager::parseNmeaSentence(char* line) {
    // checksum is xor of everything between '$' and '*', sent as two hex digits
    char* star = strchr(line, '*');
    if (star == nullptr) return false;
    I32 hi = hexValue(star[1]);
    I32 lo = (hi < 0) ? -1 : hexValue(star[2]);
    if (hi < 0 || lo < 0) return false;

    U8 cs = 0;
    for (char* c = line + 1; c < star; c++) {
        cs ^= static_cast<U8>(*c);
    }
    if (cs != static_cast<U8>((hi << 4) | lo)) return false;
    *star = '\0';

    // split on commas in place - can't use strtok since it skips empty fields, and nmea uses empty fields for "no data"
    char* fields[NMEA_MAX_FIELDS] = {};
    U32 numFields = 0;
    char* cursor = line + 1;
    fields[numFields++] = cursor;
    while (*cursor != '\0' && numFields < NMEA_MAX_FIELDS) {
        if (*cursor == ',') {
            *cursor = '\0';
            fields[numFields++] = cursor + 1;
        }
        cursor++;
    }

    // match on sentence type and ignore the talker id (GP, GN, GL...) since multi-gnss uses GN
    if (strlen(fields[0]) != 5) return false;
    const char* type = fields[0] + 2;

    if (strcmp(type, "GGA") == 0) {
        if (numFields <= GGA_ALT) return false;

        I32 sats = atoi(fields[GGA_NUM_SV]);
        this->m_numSats = static_cast<U8>((sats < 0) ? 0 : (sats > 255) ? 255 : sats);

        // quality 0 means no fix - lat/lon/alt are empty, so keep the last known position and report 0 sats
        F64 lat = 0, lon = 0;
        if (atoi(fields[GGA_QUALITY]) == 0 ||
            !parseNmeaCoord(fields[GGA_LAT], fields[GGA_NS], lat) ||
            !parseNmeaCoord(fields[GGA_LON], fields[GGA_EW], lon)) {
            this->m_numSats = 0;
            return true;
        }
        this->m_lat = lat;
        this->m_lon = lon;
        this->m_alt = strtof(fields[GGA_ALT], nullptr);
        return true;
    }

    if (strcmp(type, "RMC") == 0) {
        if (numFields <= RMC_SPEED) return false;
        if (fields[RMC_STATUS][0] == 'A') {
            this->m_speed = static_cast<F32>(strtod(fields[RMC_SPEED], nullptr) * KNOTS_TO_MPS);
        }
        return false;
    }

    return false;
}

// NEO-M9N only gives data through reg 0xff in 32-byte chunks padded with 0xff fill - nmea is ascii so 0xff is never real data
Drv::I2cStatus NavigationManager::readGpsData(F64& lat, F64& lon, F32& alt, F32& speed, U8& numSats, bool& packetFound) {
    packetFound = false;
    U32 emptyChunks = 0;

    // read up to 16 chunks of 32 bytes from reg 0xff
    for (U32 attempts = 0; attempts < 16; attempts++) {
        U8 regAddr = GPS_DATA_REG;
        Fw::Buffer writeBuffer(&regAddr, sizeof(regAddr));
        U8 chunk[GPS_CHUNK_SIZE] = {};
        Fw::Buffer readBuffer(chunk, GPS_CHUNK_SIZE);

        Drv::I2cStatus status = this->busWriteRead_out(0, GPS_I2C_ADDRESS, writeBuffer, readBuffer);
        if (status != Drv::I2cStatus::I2C_OK) {
            return status;
        }

        // feed real bytes into the sentence assembler, skip the 0xff fill
        bool hasData = false;
        for (U32 i = 0; i < GPS_CHUNK_SIZE; i++) {
            if (chunk[i] != 0xFF) {
                this->feedNmeaByte(chunk[i], packetFound);
                hasData = true;
            }
        }

        // 3 empty chunks in a row means the gps has nothing more to send
        if (!hasData) {
            emptyChunks++;
            if (emptyChunks >= 3) break;
        } else {
            emptyChunks = 0;
        }
    }

    lat     = this->m_lat;
    lon     = this->m_lon;
    alt     = this->m_alt;
    speed   = this->m_speed;
    numSats = this->m_numSats;
    return Drv::I2cStatus::I2C_OK;
}

}  // namespace Managers
