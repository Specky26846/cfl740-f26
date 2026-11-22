module Managers {
    @ Camera Module Component
    active component CameraComponent {

        state machine instance camSm: CameraComponentStateMachine

        @ Rate group tick, drives the state machine
        async input port run: Svc.Sched

        @ Command to take a photo using a specified camera and file path
        async command TAKE_PHOTO(
            cameraId: U32 @< Camera index
            filePath: string size 256 @< Directory where image will be saved
        )

        @ Event indicating a photo capture request
        event TakePhotoEvent(
            cameraId: U32 @< Camera index
            filePath: string size 256 @< Image output path
        ) severity activity high format "Photo captured using camera {}: {}"

        @ A TAKE_PHOTO step failed. exitCode is the program's exit status:
        @ 127 = program not found, 128+N = killed by signal N, -1 = fork failed
        event CameraStepFailed(
            step: string size 40 @< Which step failed
            exitCode: I32 @< Exit status of the external program
        ) severity warning high format "TAKE_PHOTO failed at {} (exit code {})"

        @ A count of the number of photos successfully taken
        telemetry PhotoCount: U32

        @ Send saved image path to port after image capture finishes
        output port imageCaptured: Managers.ImageFile

        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Enables command handling
        import Fw.Command

        @ Enables event handling
        import Fw.Event

        @ Enables telemetry channels handling
        import Fw.Channel

        @ Port to return the value of a parameter
        param get port prmGetOut

        @Port to set the value of a parameter
        param set port prmSetOut
    

    }
}