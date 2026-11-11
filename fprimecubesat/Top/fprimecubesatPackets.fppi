telemetry packets fprimecubesatPackets {

  # ---------- Group 1: flight essentials ----------
  packet Nav id 1 group 1 {
    navigationManager.Latitude
    navigationManager.Longitude
    navigationManager.Altitude
    navigationManager.GroundSpeed
    navigationManager.NumSatellites
  }

  packet Health id 2 group 1 {
    systemManager.SystemState
    systemManager.TotalComponentFaults
    CdhCore.cmdDisp.CommandsDispatched
    CdhCore.cmdDisp.CommandErrors
    pduManager.VBattVoltage
    pduManager.VBattCurrent
    radioManager.State
    radioManager.LastRssi
    radioManager.LastSnr
    radioManager.FramesReceived
    radioManager.FramesSent
    pduManager.Reg3v3Temp
    pduManager.Reg5v0Temp
    tempManager.Temperature
  }

  packet Attitude id 3 group 1 {
    imuManager.AccelX
    imuManager.AccelY
    imuManager.AccelZ
    imuManager.GyroX
    imuManager.GyroY
    imuManager.GyroZ
    imuManager.ImuTemp
    magnetometerManager.MagX
    magnetometerManager.MagY
    magnetometerManager.MagZ
  }

  # ---------- Group 2: engineering ----------
  packet Power id 10 group 2 {
    pduManager.Rail3v3Current
    pduManager.Rail3v3Voltage
    pduManager.Rail5v0Current
    pduManager.Rail5v0Voltage
    pduManager.VBattCurrent
    pduManager.VBattRawCurrent
    pduManager.VBattRawVoltage
    pduManager.VBattVoltage
  }

  packet RadioStats id 11 group 2 {
    radioManager.RxCrcErrors
    radioManager.RxDropped
    radioManager.SpiErrors
    radioManager.TxFailures
  }

  packet CdhErrors id 12 group 2 {
    CdhCore.$health.PingLateWarnings
    CdhCore.events.EventsDropped
    CdhCore.cmdDisp.CommandsDropped
    rateGroup1.RgCycleSlips
    rateGroup1.RgMaxTime
    rateGroup2.RgCycleSlips
    rateGroup2.RgMaxTime
    rateGroup3.RgCycleSlips
    rateGroup3.RgMaxTime
    ComCcsds.commsBufferManager.NoBuffs
    ComCcsds.comQueue.comQueueDepth
  }

  packet Files id 13 group 2 {
    FileHandling.fileDownlink.FilesSent
    FileHandling.fileDownlink.PacketsSent
    FileHandling.fileDownlink.Warnings
    FileHandling.fileManager.CommandsExecuted
    FileHandling.fileManager.Errors
    FileHandling.fileUplink.FilesReceived
    FileHandling.fileUplink.FilesReceivedFailed
    FileHandling.fileUplink.PacketsReceived
    FileHandling.fileUplink.Warnings
  }

  packet System id 14 group 2 {
    systemResources.CPU
    systemResources.MEMORY_USED
    systemResources.NON_VOLATILE_FREE
    systemManager.SystemUptime
    systemManager.DegradedTicks
  }

  # ---------- Group 3: on request (SEND_PKT) ----------
  packet Version id 20 group 3 {
    CdhCore.version.FrameworkVersion
    CdhCore.version.ProjectVersion
  }

} omit {
  # tutorial LEDs
  led.BlinkCommandCount
  led.BlinkingState
  led.LEDTransitions
  led2.BlinkCommandCount
  led2.BlinkingState
  led2.LEDTransitions

  # unused version slots
  CdhCore.version.CustomVersion01
  CdhCore.version.CustomVersion02
  CdhCore.version.CustomVersion03
  CdhCore.version.CustomVersion04
  CdhCore.version.CustomVersion05
  CdhCore.version.CustomVersion06
  CdhCore.version.CustomVersion07
  CdhCore.version.CustomVersion08
  CdhCore.version.CustomVersion09
  CdhCore.version.CustomVersion10
  CdhCore.version.LibraryVersion01
  CdhCore.version.LibraryVersion02
  CdhCore.version.LibraryVersion03
  CdhCore.version.LibraryVersion04
  CdhCore.version.LibraryVersion05
  CdhCore.version.LibraryVersion06
  CdhCore.version.LibraryVersion07
  CdhCore.version.LibraryVersion08
  CdhCore.version.LibraryVersion09
  CdhCore.version.LibraryVersion10

  # comms buffer bookkeeping (NoBuffs + comQueueDepth kept in CdhErrors)
  ComCcsds.comQueue.buffQueueDepth
  ComCcsds.commsBufferManager.CurrBuffs
  ComCcsds.commsBufferManager.EmptyBuffs
  ComCcsds.commsBufferManager.HiBuffs
  ComCcsds.commsBufferManager.TotalBuffs

  # data products (unused)
  DataProducts.dpBufferAccumulator.BA_NumQueuedBuffers
  DataProducts.dpBufferManager.CurrBuffs
  DataProducts.dpBufferManager.EmptyBuffs
  DataProducts.dpBufferManager.HiBuffs
  DataProducts.dpBufferManager.NoBuffs
  DataProducts.dpBufferManager.TotalBuffs
  DataProducts.dpCat.CatalogDps
  DataProducts.dpCat.DpsSent
  DataProducts.dpMgr.NumBytes
  DataProducts.dpMgr.NumDataProducts
  DataProducts.dpMgr.NumFailedAllocations
  DataProducts.dpMgr.NumSuccessfulAllocations
  DataProducts.dpWriter.NumBuffersReceived
  DataProducts.dpWriter.NumBytesWritten
  DataProducts.dpWriter.NumErrors
  DataProducts.dpWriter.NumFailedWrites
  DataProducts.dpWriter.NumSuccessfulWrites

  # command sequencer (add a packet if sequences get used)
  cmdSeq.CS_CancelCommands
  cmdSeq.CS_CommandsExecuted
  cmdSeq.CS_CurrentSequence
  cmdSeq.CS_Errors
  cmdSeq.CS_LoadCommands
  cmdSeq.CS_SequencesCompleted

  # packetizer's own config echo (large arrays)
  CdhCore.tlmSend.GroupConfigs
  CdhCore.tlmSend.SectionEnabled

  # misc
  radioSpi.SPI_Bytes
  systemManager.CpuUsage
  systemManager.MemUsage

  # per-core CPU (Pi 4 has 4 cores; total CPU kept in System) and static sizes
  systemResources.CPU_00
  systemResources.CPU_01
  systemResources.CPU_02
  systemResources.CPU_03
  systemResources.CPU_04
  systemResources.CPU_05
  systemResources.CPU_06
  systemResources.CPU_07
  systemResources.CPU_08
  systemResources.CPU_09
  systemResources.CPU_10
  systemResources.CPU_11
  systemResources.CPU_12
  systemResources.CPU_13
  systemResources.CPU_14
  systemResources.CPU_15
  systemResources.MEMORY_TOTAL
  systemResources.NON_VOLATILE_TOTAL
}
