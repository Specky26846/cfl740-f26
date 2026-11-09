module Types {
@ defines common data types used by multiple components

    @ 3D vector for geometric data (acceleration, rotation, magnetic field)
    struct GeometricVector3 {
        x: F32
        y: F32
        z: F32
    }

        @ One IMU sample (accel in g, gyro in deg/s, temp in C)
    struct ImuData {
        accel: GeometricVector3
        gyro: GeometricVector3
        temp: F32
    }

    @ One pass over the PDU board (volts, amps, C)
    struct PowerData {
        vbattRawV: F32
        vbattRawI: F32
        v3v3: F32
        i3v3: F32
        v5v0: F32
        i5v0: F32
        vbattV: F32
        vbattI: F32
        temp3v3Reg: F32
        temp5v0Reg: F32
    }

    @ Latest GPS fix (degrees, m, m/s)
    struct GpsData {
        latitude: F64
        longitude: F64
        altitude: F32
        groundSpeed: F32
        numSatellites: U8
    }


}

module Managers {
@ defines port types for managers to send data to PacketBuilder

    @ Health status reported by a component to SystemManager each tick.
    @ healthy = true means the component is operating normally.
    @ healthy = false means a fault was detected (e.g. I2C read failure)
    port ComponentHealth(healthy: bool)

    @ Latest sensor readings pushed from each manager to PacketBuilder
    port ImuSample(data: Types.ImuData)
    port MagSample(data: Types.GeometricVector3)
    port TempSample(temperature: F32)
    port PduSample(data: Types.PowerData)
    port GpsSample(data: Types.GpsData)


}
