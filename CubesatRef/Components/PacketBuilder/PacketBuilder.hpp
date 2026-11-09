// ======================================================================
// \title  PacketBuilder.hpp
// \author lauraf26846
// \brief  hpp file for PacketBuilder component implementation class
// ======================================================================

#ifndef CubesatRef_PacketBuilder_HPP
#define CubesatRef_PacketBuilder_HPP

#include "CubesatRef/Components/PacketBuilder/PacketBuilderComponentAc.hpp"

namespace CubesatRef {

class PacketBuilder final : public PacketBuilderComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct PacketBuilder object
    PacketBuilder(const char* const compName  //!< The component name
    );

    //! Destroy PacketBuilder object
    ~PacketBuilder();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    void run_handler(FwIndexType portNum, U32 context) override;

    void imuIn_handler(FwIndexType portNum, const Types::ImuData& data) override;
    void magIn_handler(FwIndexType portNum, const Types::GeometricVector3& data) override;
    void tempIn_handler(FwIndexType portNum, F32 temperature) override;
    void pduIn_handler(FwIndexType portNum, const Types::PowerData& data) override;
    void gpsIn_handler(FwIndexType portNum, const Types::GpsData& data) override;

    // serialize the beacon fields in the same order as BEACON_DEFINITION in the encode.py
    void serializeBeacon(Fw::ComBuffer& beacon);

    // Rpi soc temperature in C, 0 if it can't be read
    F64 readCpuTemp();

    // latest readings - zero until each manager reports in
    Types::ImuData m_imu;
    Types::GeometricVector3 m_mag;
    F32 m_temp = 0.0f;
    Types::PowerData m_power;
    Types::GpsData m_gps;

    U32 m_beaconCount = 0;

  };
}  // namespace CubesatRef

#endif
