module Vision {
  active component AprilTag {
    @ Process a completed camera image on this component's task.
    async input port processImage: Managers.ImageFile

    @ Process one saved image using the settings in AprilTag.cpp and append its tag poses to CSV.
    async command PROCESS_IMAGE(
      imagePath: string size 40
    ) opcode 0

    import Fw.Command
  }
}
