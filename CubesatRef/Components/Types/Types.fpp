module Types {

    @ 3D vector for geometric data (acceleration, rotation, magnetic field)
    struct GeometricVector3 {
        x: F32
        y: F32
        z: F32
    }

}

module Managers {

    @ Health status reported by a component to SystemManager each tick.
    @ healthy = true means the component is operating normally.
    @ healthy = false means a fault was detected (e.g. I2C read failure)
    port ComponentHealth(healthy: bool)

}
