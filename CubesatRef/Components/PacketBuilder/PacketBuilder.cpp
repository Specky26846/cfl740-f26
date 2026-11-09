// ======================================================================
// \title  PacketBuilder.cpp
// \author lauraf26846
// \brief  cpp file for PacketBuilder component implementation class
// ======================================================================

#include "CubesatRef/Components/PacketBuilder/PacketBuilder.hpp"

namespace CubesatRef {

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

void PacketBuilder::run_handler(FwIndexType portNum, U32 context) {}


}  // namespace CubesatRef
