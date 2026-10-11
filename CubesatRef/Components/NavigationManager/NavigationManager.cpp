// NavigationManager.cpp

#include "CubesatRef/Components/NavigationManager/NavigationManager.hpp"
#include <cstring>


namespace Managers {

static constexpr U8  GPS_I2C_ADDRESS = 0x42;
static constexpr U8  GPS_DATA_REG    = 0xFF;
static constexpr U32 GPS_CHUNK_SIZE  = 32;
static constexpr U32 GPS_BUFFER_SIZE = 512;

// ubx-nav-pvt payload offsets
static constexpr U32 PVT_NUM_SV     = 23;   // number of satellites
static constexpr U32 PVT_LON        = 24;   // longitude (1e-7 degrees)
static constexpr U32 PVT_LAT        = 28;   // latitude (1e-7 degrees)
static constexpr U32 PVT_HEIGHT     = 36;   // height above msl (mm)
static constexpr U32 PVT_GSPEED     = 60;   // ground speed (mm/s)
static constexpr U32 PVT_PAYLOAD_LEN = 92;

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

// ask for a nav-pvt packet, parse out position and speed, push telemetry
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

    // no packet is not a fault gps only produces nav-pvt at its configured rate, not every i2c poll
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

    // throttle the event log - nav-pvt fires every second and event bandwidth is limited
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

// tell the gps we're a cubesat, airborne dynamic model under 4g
void NavigationManager::configure() {
    U8 payload[] = {
        0x00, 0x03, 0x00, 0x00,
        0x21, 0x00, 0x11, 0x20,
        0x08
    };
    this->sendUbxCfg(payload, sizeof(payload));
}

// pull a little-endian i32 out of a byte buffer
static I32 readI32LE(const U8* buf) {
    return static_cast<I32>(
        static_cast<U32>(buf[0]) |
        (static_cast<U32>(buf[1]) << 8) |
        (static_cast<U32>(buf[2]) << 16) |
        (static_cast<U32>(buf[3]) << 24)
    );
}

// NEO-M9N only gives data through reg 0xff in 32-byte chunks padded with 0xff fill - strip those and scan for the packet
Drv::I2cStatus NavigationManager::readGpsData(F64& lat, F64& lon, F32& alt, F32& speed, U8& numSats, bool& packetFound) {
    packetFound = false;

    // send the ubx-nav-pvt poll request so the gps queues up a fresh packet
    {
        U8 poll[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
        Fw::Buffer pollBuf(poll, sizeof(poll));
        this->busWrite_out(0, GPS_I2C_ADDRESS, pollBuf);
    }

    U8 accumBuf[GPS_BUFFER_SIZE] = {};
    U32 accumLen = 0;
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

        // copy only non-0xff bytes into accumBuf - 0xff is fill, not real data
        bool hasData = false;
        for (U32 i = 0; i < GPS_CHUNK_SIZE && accumLen < GPS_BUFFER_SIZE; i++) {
            if (chunk[i] != 0xFF) {
                accumBuf[accumLen++] = chunk[i];
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

    // nothing came back at all - that's ok, just no packet this tick
    if (accumLen == 0) {
        return Drv::I2cStatus::I2C_OK;
    }

    // scan the accumulated bytes for ubx-nav-pvt: sync 0xb5 0x62, class 0x01, id 0x07
    for (U32 i = 0; i + 6 < accumLen; i++) {
        if (accumBuf[i] != 0xB5 || accumBuf[i + 1] != 0x62) continue;
        if (accumBuf[i + 2] != 0x01 || accumBuf[i + 3] != 0x07) continue;

        // payload length is little-endian in bytes 4-5 of the ubx header
        U16 payloadLen = static_cast<U16>(accumBuf[i + 4]) |
                         (static_cast<U16>(accumBuf[i + 5]) << 8);

        // sanity check: nav-pvt is always 92 bytes, and we need to have the full packet plus 2 checksum bytes
        if (payloadLen != PVT_PAYLOAD_LEN) continue;
        if (i + 6 + payloadLen + 2 > accumLen) continue;

        const U8* payload = &accumBuf[i + 6];

        // verify fletcher checksum - runs over everything from class byte onward (skips the two sync bytes)
        U8 ck_a = 0, ck_b = 0;
        for (U32 j = 2; j < 6 + payloadLen; j++) {
            ck_a += accumBuf[i + j];
            ck_b += ck_a;
        }
        if (ck_a != accumBuf[i + 6 + payloadLen] ||
            ck_b != accumBuf[i + 6 + payloadLen + 1]) {
            continue;  // checksum failed, keep scanning in case there's another packet
        }

        // pull the fields we care about - all are little-endian i32 except numSv which is a plain byte
        numSats = payload[PVT_NUM_SV];

        I32 lonRaw    = readI32LE(&payload[PVT_LON]);
        I32 latRaw    = readI32LE(&payload[PVT_LAT]);
        I32 heightMSL = readI32LE(&payload[PVT_HEIGHT]);
        I32 gSpeed    = readI32LE(&payload[PVT_GSPEED]);

        // scale from ubx units to human units: 1e-7 deg, mm→m, mm/s→m/s
        lon   = static_cast<F64>(lonRaw)    * 1e-7;
        lat   = static_cast<F64>(latRaw)    * 1e-7;
        alt   = static_cast<F32>(heightMSL) / 1000.0f;
        speed = static_cast<F32>(gSpeed)    / 1000.0f;
        packetFound = true;

        break;
    }

    return Drv::I2cStatus::I2C_OK;
}

}  // namespace Managers
