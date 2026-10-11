module Managers {

    # blinky led component, toggles gpio at configurable interval
    active component Led {

        # turn blinking on or off
        async command BLINKING_ON_OFF(
            onOff: Fw.On
        )

        event SetBlinkingState($state: Fw.On) \
            severity activity high \
            format "Set blinking state to {}."

        event BlinkIntervalSet(interval: U32) \
            severity activity high \
            format "LED blink interval set to {}"

        event LedState($state: Fw.On) \
            severity activity low \
            format "LED state driven to {}"

        telemetry BlinkCommandCount: U32
        telemetry LEDTransitions: U64
        telemetry BlinkingState: Fw.On

        param BLINK_INTERVAL: U32 default 1

        # rate group input
        async input port run: Svc.Sched

        # gpio output to drive the led
        output port gpioSet: Drv.GpioWrite

        # standard fprime ports
        time get port timeCaller
        command reg port cmdRegOut
        command recv port cmdIn
        command resp port cmdResponseOut
        text event port logTextOut
        event port logOut
        telemetry port tlmOut
        param get port prmGetOut
        param set port prmSetOut

    }
}
