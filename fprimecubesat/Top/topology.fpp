module fprimecubesat {

  # ----------------------------------------------------------------------
  # Symbolic constants for port numbers
  # ----------------------------------------------------------------------

  enum Ports_RateGroups {
    rateGroup1
    rateGroup2
    rateGroup3
  }

  deployment topology fprimecubesat {

  # ----------------------------------------------------------------------
  # Subtopology imports
  # ----------------------------------------------------------------------
    import CdhCore.Subtopology
    import ComCcsds.FramingSubtopology
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

    instance cmdSeq

    instance radioResetGpio

    # Manager instances
    instance systemManager
    instance imuManager
    instance navigationManager
    instance magnetometerManager
    instance tempManager
    instance pduManager
    instance radioManager
    instance packetBuilder

    # Bus driver instances
    instance imuI2cDriver
    instance magI2cDriver
    instance gpsI2cDriver
    instance tempI2cDriver
    instance pduI2cDriver
    instance radioSpi

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

    include "fprimecubesatPackets.fppi"

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
      rateGroup1.RateGroupMemberOut[6] -> tempManager.run
      rateGroup1.RateGroupMemberOut[7] -> magnetometerManager.run
      rateGroup1.RateGroupMemberOut[8] -> navigationManager.run
      rateGroup1.RateGroupMemberOut[9] -> imuManager.run
      rateGroup1.RateGroupMemberOut[10] -> packetBuilder.run

      # Rate group 2 (10Hz - fast periodic)
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup2] -> rateGroup2.CycleIn
      rateGroup2.RateGroupMemberOut[0] -> cmdSeq.schedIn
      rateGroup2.RateGroupMemberOut[1] -> radioManager.run

      # Rate group 3 (2.5Hz - infrastructure)
      rateGroupDriver.CycleOut[Ports_RateGroups.rateGroup3] -> rateGroup3.CycleIn
      rateGroup3.RateGroupMemberOut[0] -> CdhCore.$health.Run
      rateGroup3.RateGroupMemberOut[1] -> ComCcsds.commsBufferManager.schedIn
      rateGroup3.RateGroupMemberOut[2] -> DataProducts.dpBufferManager.schedIn
      rateGroup3.RateGroupMemberOut[3] -> DataProducts.dpWriter.schedIn
      rateGroup3.RateGroupMemberOut[4] -> DataProducts.dpMgr.schedIn
      rateGroup3.RateGroupMemberOut[5] -> pduManager.run
      rateGroup3.RateGroupMemberOut[6] -> ComCcsds.aggregator.timeout
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

      # Eddy PDU (2x ADS7828) I2C bus connections
      pduManager.busWriteRead -> pduI2cDriver.writeRead
      pduManager.busWrite -> pduI2cDriver.write

      # Component health reporting to SystemManager
      imuManager.healthOut -> systemManager.sensorHealth
      tempManager.healthOut -> systemManager.tempHealth
      navigationManager.healthOut -> systemManager.gpsHealth
      magnetometerManager.healthOut -> systemManager.magHealth

    }

    connections fprimecubesat {
      packetBuilder.rapOut -> radioManager.rapIn
      
      imuManager.dataOut -> packetBuilder.imuIn
      magnetometerManager.dataOut -> packetBuilder.magIn
      tempManager.dataOut -> packetBuilder.tempIn
      pduManager.dataOut -> packetBuilder.pduIn
      navigationManager.dataOut -> packetBuilder.gpsIn

      # RadioManager -> reset GPIO (RST pin, GPIO25)
      radioManager.resetPin -> radioResetGpio.gpioWrite

      # RadioManager -> SPI driver
      radioManager.spiWriteRead -> radioSpi.SpiWriteRead

      # RadioManager RX buffers
      radioManager.allocate   -> ComCcsds.commsBufferManager.bufferGetCallee
      radioManager.deallocate -> ComCcsds.commsBufferManager.bufferSendIn

      # Downlink: framer -> radio
      ComCcsds.FramingSubtopology.dataOut  -> radioManager.dataIn
      radioManager.dataReturnOut           -> ComCcsds.FramingSubtopology.dataReturnIn
      radioManager.comStatusOut            -> ComCcsds.FramingSubtopology.comStatusIn

      # Uplink: radio -> frame accumulator
      radioManager.dataOut                 -> ComCcsds.FramingSubtopology.dataIn
      ComCcsds.FramingSubtopology.dataReturnOut -> radioManager.dataReturnIn
    
    }

  }

}
