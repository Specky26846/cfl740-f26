module fprimecubesat {

  # ----------------------------------------------------------------------
  # Symbolic constants for port numbers
  # ----------------------------------------------------------------------

  enum Ports_RateGroups {
    rateGroup1
    rateGroup2
    rateGroup3
  }

  topology fprimecubesat {

  # ----------------------------------------------------------------------
  # Subtopology imports
  # ----------------------------------------------------------------------
    import CdhCore.Subtopology
    import ComCcsds.Subtopology
    import DataProducts.Subtopology
    import FileHandling.Subtopology

  # ----------------------------------------------------------------------
  # Instances used in the topology
  # ----------------------------------------------------------------------
    instance led
    instance led2
    instance gpioDriver
    instance gpioDriver2
    instance gpioDriver3
    instance chronoTime
    instance rateGroup1
    instance rateGroup2
    instance rateGroup3
    instance rateGroupDriver
    instance systemResources
    instance timer
    instance comDriver
    instance cmdSeq

    # Manager instances
    instance systemManager
    instance radioManager
    instance imuManager
    instance navigationManager
    instance magnetometerManager
    instance tempManager

    # Bus driver instances
    instance imuI2cDriver
    instance magI2cDriver
    instance gpsI2cDriver
    instance tempI2cDriver
    instance radioSpiDriver

  # ----------------------------------------------------------------------
  # Pattern graph specifiers
  # ----------------------------------------------------------------------

    command connections instance CdhCore.cmdDisp
    event connections instance CdhCore.events
    telemetry connections instance CdhCore.tlmSend
    text event connections instance CdhCore.textLogger
    health connections instance CdhCore.$health
    param connections instance FileHandling.prmDb
    time connections instance chronoTime

  # ----------------------------------------------------------------------
  # Telemetry packets (only used when TlmPacketizer is used)
  # ----------------------------------------------------------------------

    # include "fprimecubesatPackets.fppi"

  # ----------------------------------------------------------------------
  # Direct graph specifiers
  # ----------------------------------------------------------------------
    connections fprimecubesat {
      # Rate Group 1 (1Hz cycle) output is connected to led's run input
      rateGroup1.RateGroupMemberOut[4] -> led.run
      rateGroup2.RateGroupMemberOut[4] -> led2.run
      # led's gpioSet output is connected to gpioDriver's gpioWrite input
      led.gpioSet -> gpioDriver.gpioWrite
      # led2's pin isn't confirmed yet - see gpioDriver3 note in instances.fpp
      led2.gpioSet -> gpioDriver3.gpioWrite
      # gpioDriver2 is driven by SystemManager state machine:
      #   HIGH (LED on)  = NOMINAL
      #   LOW  (LED off) = REBOOT / fault
      systemManager.statusLedSet -> gpioDriver2.gpioWrite
    }

    connections ComCcsds_CdhCore {
      # Core events and telemetry to communication queue
      CdhCore.events.PktSend -> ComCcsds.comQueue.comPacketQueueIn[ComCcsds.Ports_ComPacketQueue.EVENTS]
      CdhCore.tlmSend.PktSend -> ComCcsds.comQueue.comPacketQueueIn[ComCcsds.Ports_ComPacketQueue.TELEMETRY]

      # Router to Command Dispatcher
      ComCcsds.fprimeRouter.commandOut -> CdhCore.cmdDisp.seqCmdBuff
      CdhCore.cmdDisp.seqCmdStatus -> ComCcsds.fprimeRouter.cmdResponseIn

    }

    connections ComCcsds_FileHandling {
      # File Downlink to Communication Queue
      FileHandling.fileDownlink.bufferSendOut -> ComCcsds.comQueue.bufferQueueIn[ComCcsds.Ports_ComBufferQueue.FILE]
      ComCcsds.comQueue.bufferReturnOut[ComCcsds.Ports_ComBufferQueue.FILE] -> FileHandling.fileDownlink.bufferReturn

      # Router to File Uplink
      ComCcsds.fprimeRouter.fileOut -> FileHandling.fileUplink.bufferSendIn
      FileHandling.fileUplink.bufferSendOut -> ComCcsds.fprimeRouter.fileBufferReturnIn
    }

    connections Communications {
      # ComDriver buffer allocations
      comDriver.allocate      -> ComCcsds.commsBufferManager.bufferGetCallee
      comDriver.deallocate    -> ComCcsds.commsBufferManager.bufferSendIn

      # ComDriver <-> ComStub (Uplink)
      comDriver.$recv                     -> ComCcsds.comStub.drvReceiveIn
      ComCcsds.comStub.drvReceiveReturnOut -> comDriver.recvReturnIn

      # ComStub <-> ComDriver (Downlink)
      ComCcsds.comStub.drvSendOut      -> comDriver.$send
      comDriver.ready         -> ComCcsds.comStub.drvConnected
    }

    connections FileHandling_DataProducts {
      # Data Products to File Downlink
      DataProducts.dpCat.fileOut -> FileHandling.fileDownlink.SendFile
      FileHandling.fileDownlink.FileComplete -> DataProducts.dpCat.fileDone
    }

    connections RateGroups {
      # timer to drive rate group
      timer.CycleOut -> rateGroupDriver.CycleIn

      # Rate group 1 (1Hz - slow periodic)
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup1] -> rateGroup1.CycleIn
      rateGroup1.RateGroupMemberOut[0] -> CdhCore.tlmSend.Run
      rateGroup1.RateGroupMemberOut[1] -> FileHandling.fileDownlink.Run
      rateGroup1.RateGroupMemberOut[2] -> systemResources.run
      rateGroup1.RateGroupMemberOut[3] -> ComCcsds.comQueue.run
      rateGroup1.RateGroupMemberOut[5] -> systemManager.run
      rateGroup1.RateGroupMemberOut[6] -> radioManager.run
      rateGroup1.RateGroupMemberOut[7] -> tempManager.run
      # imu/gps run here, not on the 10Hz group - see note in instances.fpp
      rateGroup1.RateGroupMemberOut[8] -> navigationManager.run
      rateGroup1.RateGroupMemberOut[9] -> imuManager.run

      # Rate group 2 (10Hz - fast periodic)
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup2] -> rateGroup2.CycleIn
      rateGroup2.RateGroupMemberOut[0] -> cmdSeq.schedIn

      # Rate group 3 (2.5Hz - infrastructure)
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup3] -> rateGroup3.CycleIn
      rateGroup3.RateGroupMemberOut[0] -> CdhCore.$health.Run
      rateGroup3.RateGroupMemberOut[1] -> ComCcsds.commsBufferManager.schedIn
      rateGroup3.RateGroupMemberOut[2] -> DataProducts.dpBufferManager.schedIn
      rateGroup3.RateGroupMemberOut[3] -> DataProducts.dpWriter.schedIn
      rateGroup3.RateGroupMemberOut[4] -> DataProducts.dpMgr.schedIn
      # mag runs here, not on the 10Hz group - see note in instances.fpp
      rateGroup3.RateGroupMemberOut[5] -> magnetometerManager.run
    }

    connections CdhCore_cmdSeq {
      # Command Sequencer
      cmdSeq.comCmdOut -> CdhCore.cmdDisp.seqCmdBuff
      CdhCore.cmdDisp.seqCmdStatus -> cmdSeq.cmdResponseIn
    }

    connections BusDrivers {
      # IMU I2C bus connections
      imuManager.busWriteRead -> imuI2cDriver.writeRead
      imuManager.busWrite -> imuI2cDriver.write

      # Magnetometer I2C bus connections
      magnetometerManager.busWriteRead -> magI2cDriver.writeRead
      magnetometerManager.busWrite -> magI2cDriver.write

      # GPS I2C bus connections (same physical bus as mag, separate driver instance)
      navigationManager.busWriteRead -> gpsI2cDriver.writeRead
      navigationManager.busWrite -> gpsI2cDriver.write

      # TMP102 I2C bus connections
      tempManager.busWriteRead -> tempI2cDriver.writeRead
      tempManager.busWrite -> tempI2cDriver.write

      # Component health reporting to SystemManager
      imuManager.healthOut -> systemManager.sensorHealth
      tempManager.healthOut -> systemManager.tempHealth
      navigationManager.healthOut -> systemManager.gpsHealth
      magnetometerManager.healthOut -> systemManager.magHealth

      # Radio SPI bus connection
      radioManager.spiReadWrite -> radioSpiDriver.SpiReadWrite
    }

    connections fprimecubesat {

    }

  }

}
