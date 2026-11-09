// ======================================================================
// \title  PacketBuilder.cpp
// \author lauraf26846
// \brief  cpp file for PacketBuilder component implementation class
// ======================================================================

#include "CubesatRef/Components/PacketBuilder/PacketBuilder.hpp"
#include "CubesatRef/Components/PacketBuilder/RapEncoder.hpp"

#include <Os/File.hpp>
#include <Os/FileSystem.hpp>
#include <Os/Memory.hpp>

#include <cmath>
#include <cstdlib>
#include <limits>

namespace CubesatRef {

// sensor scale factors back to raw counts - same numbers the managers divide by
static constexpr F64 IMU_ACCEL_LSB_PER_G = 8192.0;   // icm-20649 at +-4g
static constexpr F64 IMU_GYRO_LSB_PER_DPS = 65.5;    // icm-20649 at +-500 dps
static constexpr F64 RM3100_COUNTS_PER_UT = 75.0;    // rm3100 at cycle count 200

static const char* const CPU_TEMP_PATH = "/sys/class/thermal/thermal_zone0/temp";

// same as python's struct.pack(fmt, int(value)): truncate toward zero, little-endian,
// and if the value doesn't fit (or isn't a number) python's except branch writes zeros
template <typename T>

// truncates floating point value to integer, checks for overflow, and serializes to buffer
// this is done to match Python truncation behavior in encode.py
static void packField(Fw::ComBuffer& buf, F64 value) {
    F64 truncated = std::trunc(value);
    if (!std::isfinite(truncated) || truncated < static_cast<F64>(std::numeric_limits<T>::min()) ||
        truncated > static_cast<F64>(std::numeric_limits<T>::max())) {
        truncated = 0.0;
    }
    const Fw::SerializeStatus stat = buf.serializeFrom(static_cast<T>(truncated), Fw::Endianness::LITTLE);
    FW_ASSERT(stat == Fw::FW_SERIALIZE_OK, stat);
}

// pdu adc conversions from the python encoder - volts/amps back to 12-bit ads7828 counts
static F64 voltageToCounts(F64 volts, F64 rTop, F64 rBottom) {
    return volts * (rBottom / (rTop + rBottom)) / 2.5 * 4095;
}

static F64 currentToCounts(F64 amps, F64 shuntOhms) {
    return amps * (shuntOhms * 100.0) / 2.5 * 4095;
}

static F64 regTempToCounts(F64 tempC) {
    const F64 counts = ((1.8639 - 3.88e-6 * ((tempC + 1481.96) * (tempC + 1481.96) - 2.1962e6)) / 2.5) * 4095;
    return std::fmax(0.0, std::fmin(4095.0, counts));
}

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

PacketBuilder ::PacketBuilder(const char* const compName) : PacketBuilderComponentBase(compName) {}

PacketBuilder ::~PacketBuilder() {}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

// each handler just stores the latest reading from each manager, used to build next RAP packet in run_handler()
void PacketBuilder ::imuIn_handler(FwIndexType portNum, const Types::ImuData& data) {
    m_imu = data;
}

void PacketBuilder ::magIn_handler(FwIndexType portNum, const Types::GeometricVector3& data) {
    m_mag = data;
}

void PacketBuilder ::tempIn_handler(FwIndexType portNum, F32 temperature) {
    m_temp = temperature;
}

void PacketBuilder ::pduIn_handler(FwIndexType portNum, const Types::PowerData& data) {
    m_power = data;
}

void PacketBuilder ::gpsIn_handler(FwIndexType portNum, const Types::GpsData& data) {
    m_gps = data;
}

void PacketBuilder ::run_handler(FwIndexType portNum, U32 context) {
    Fw::ComBuffer beacon;
    this->serializeBeacon(beacon);

    // wrap the beacon straight into the output buffer's memory
    Fw::ComBuffer rap;
    const FwSizeType rapLen = Rap::encode(Rap::FLAG_BEACON, beacon.getBuffAddr(), beacon.getSize(),
                                          rap.getBuffAddr(), rap.getCapacity());
    FW_ASSERT(rapLen > 0, static_cast<FwAssertArgType>(beacon.getSize()));
    
    // tell ComBuffer how many bytes are valid (avoids radio receiving 0 byte packet)
    const Fw::SerializeStatus stat = rap.setBuffLen(rapLen);
    
    FW_ASSERT(stat == Fw::FW_SERIALIZE_OK, stat); // only asserts if someone changed beacon fields so they don't fit in the buffer

    if (this->isConnected_rapOut_OutputPort(0)) {
        this->rapOut_out(0, rap, 0);
    }

    m_beaconCount++;
}


// field order and formats match BEACON_DEFINITION in ALPHA_TX_LORA.py
void PacketBuilder ::serializeBeacon(Fw::ComBuffer& beacon) {
    // memory and disk in KiB - bytes overflow the U32 fields on the pi
    Os::Memory::Usage mem = {0, 0};
    if (Os::Memory::getUsage(mem) != Os::Generic::OP_OK) {
        mem = {0, 0};
    }
    FwSizeType diskTotal = 0;
    FwSizeType diskFree = 0;
    if (Os::FileSystem::getFreeSpace("/", diskTotal, diskFree) != Os::FileSystem::OP_OK) {
        diskTotal = 0;
        diskFree = 0;
    }

    packField<U32>(beacon, this->getTime().getSeconds());  // unix_time
    packField<U16>(beacon, m_beaconCount);                  // packet_count
    packField<U16>(beacon, 0);                              // num_resets - not tracked yet

    // imu as raw counts
    packField<I16>(beacon, m_imu.get_gyro().get_x() * IMU_GYRO_LSB_PER_DPS);
    packField<I16>(beacon, m_imu.get_gyro().get_y() * IMU_GYRO_LSB_PER_DPS);
    packField<I16>(beacon, m_imu.get_gyro().get_z() * IMU_GYRO_LSB_PER_DPS);
    packField<I16>(beacon, m_imu.get_accel().get_x() * IMU_ACCEL_LSB_PER_G);
    packField<I16>(beacon, m_imu.get_accel().get_y() * IMU_ACCEL_LSB_PER_G);
    packField<I16>(beacon, m_imu.get_accel().get_z() * IMU_ACCEL_LSB_PER_G);

    // mag_x/y/z - the icm-20649 has no magnetometer, the rm3100 goes in its own fields below
    packField<I16>(beacon, 0);
    packField<I16>(beacon, 0);
    packField<I16>(beacon, 0);

    packField<I16>(beacon, m_temp);            // tmp1 - tmp102, whole degrees C
    packField<I16>(beacon, m_imu.get_temp());  // tmp2 - imu die temp, whole degrees C

    // no bme280 on this board
    packField<I16>(beacon, 0);  // bme_temp
    packField<U32>(beacon, 0);  // bme_pressure
    packField<U16>(beacon, 0);  // bme_humidity

    packField<U32>(beacon, static_cast<F64>(mem.used) / 1024.0);              // used_memory
    packField<U32>(beacon, static_cast<F64>(mem.total - mem.used) / 1024.0);  // free_memory
    packField<U32>(beacon, static_cast<F64>(diskTotal - diskFree) / 1024.0);  // used_disk
    packField<U32>(beacon, static_cast<F64>(diskFree) / 1024.0);              // free_disk
    packField<U16>(beacon, 0);                                                // cpu_load - TODO
    packField<I16>(beacon, this->readCpuTemp() * 10);                         // cpu_temp

    // power - batt_* is the raw battery input, v/i_vbatt is after the switch
    packField<U16>(beacon, voltageToCounts(m_power.get_vbattRawV(), 100.0, 12.0));
    packField<U16>(beacon, currentToCounts(m_power.get_vbattRawI(), 0.012));
    packField<U16>(beacon, voltageToCounts(m_power.get_v3v3(), 100.0, 60.4));
    packField<U16>(beacon, currentToCounts(m_power.get_i3v3(), 0.005));
    packField<U16>(beacon, voltageToCounts(m_power.get_v5v0(), 100.0, 33.2));
    packField<U16>(beacon, currentToCounts(m_power.get_i5v0(), 0.012));
    packField<U16>(beacon, voltageToCounts(m_power.get_vbattV(), 100.0, 12.0));
    packField<U16>(beacon, currentToCounts(m_power.get_vbattI(), 0.012));
    packField<U16>(beacon, regTempToCounts(m_power.get_temp3v3Reg()));
    packField<U16>(beacon, regTempToCounts(m_power.get_temp5v0Reg()));

    // gps
    packField<I32>(beacon, m_gps.get_latitude() * 1e7);
    packField<I32>(beacon, m_gps.get_longitude() * 1e7);
    packField<U32>(beacon, m_gps.get_altitude() * 1e5);
    packField<U16>(beacon, m_gps.get_groundSpeed() * 1000);
    packField<U8>(beacon, m_gps.get_numSatellites());

    // rm3100 as raw counts
    packField<I32>(beacon, m_mag.get_x() * RM3100_COUNTS_PER_UT);
    packField<I32>(beacon, m_mag.get_y() * RM3100_COUNTS_PER_UT);
    packField<I32>(beacon, m_mag.get_z() * RM3100_COUNTS_PER_UT);
}

// sysfs reports millidegrees as ascii, e.g. "48312\n"
F64 PacketBuilder ::readCpuTemp() {
    Os::File file;
    if (file.open(CPU_TEMP_PATH, Os::File::OPEN_READ) != Os::File::OP_OK) {
        return 0.0;
    }
    char text[16] = {};
    FwSizeType size = sizeof(text) - 1;
    const Os::File::Status status = file.read(reinterpret_cast<U8*>(text), size);
    file.close();
    if (status != Os::File::OP_OK || size == 0) {
        return 0.0;
    }
    return std::strtod(text, nullptr) / 1000.0;
}

}  // namespace CubesatRef
