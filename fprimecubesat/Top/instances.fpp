module fprimecubesat {

  # ----------------------------------------------------------------------
  # Base ID Convention
  # ----------------------------------------------------------------------
  #
  # All Base IDs follow the 8-digit hex format: 0xDSSCCxxx
  #
  # Where:
  #   D   = Deployment digit (1 for this deployment)
  #   SS  = Subtopology digits (00 for main topology, 01-05 for subtopologies)
  #   CC  = Component digits (00, 01, 02, etc.)
  #   xxx = Reserved for internal component items (events, commands, telemetry)
  #

  # ----------------------------------------------------------------------
  # Defaults
  # ----------------------------------------------------------------------

  module Default {
    constant QUEUE_SIZE = 10
    constant STACK_SIZE = 64 * 1024
  }

  # ----------------------------------------------------------------------
  # Active component instances
  # ----------------------------------------------------------------------
  instance gpioDriver: Drv.LinuxGpioDriver base id 0x10015000
  instance gpioDriver2: Drv.LinuxGpioDriver base id 0x10017000
  # led2's pin - placeholder BCM 27, not yet confirmed against real wiring
  instance gpioDriver3: Drv.LinuxGpioDriver base id 0x10018000

  instance led: Managers.Led base id 0x10005000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 95

  instance led2: Managers.Led base id 0x10008000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 95

  instance rateGroup1: Svc.ActiveRateGroup base id 0x10001000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 120

  instance rateGroup2: Svc.ActiveRateGroup base id 0x10002000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 119

  instance rateGroup3: Svc.ActiveRateGroup base id 0x10003000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 118

  instance cmdSeq: Svc.CmdSequencer base id 0x10004000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 117

  # ----------------------------------------------------------------------
  # Manager component instances (1Hz rate group)
  # ----------------------------------------------------------------------

  instance systemManager: Managers.SystemManager base id 0x10020000 \
    queue size 50

  instance tempManager: Managers.TempManager base id 0x10025000 \
    queue size 50 \
    stack size Default.STACK_SIZE \
    priority 90

  instance radioManager: Managers.RadioManager base id 0x10021000 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 90

  # ----------------------------------------------------------------------
  # Manager component instances (originally 10Hz rate group)
  #
  # imuManager and navigationManager actually run on rateGroup1 (1Hz), and
  # magnetometerManager on rateGroup3 (2.5Hz) - see connections RateGroups
  # in topology.fpp. They were moved off the 10Hz group because blocking
  # usleep() calls in the mag/gps read handlers were stalling everything
  # else scheduled on that rate group.
  # ----------------------------------------------------------------------

  instance imuManager: Managers.IMUManager base id 0x10022000 \
    queue size 50 \
    stack size Default.STACK_SIZE \
    priority 100

  instance navigationManager: Managers.NavigationManager base id 0x10023000 \
    queue size 50 \
    stack size Default.STACK_SIZE \
    priority 100

  instance magnetometerManager: Managers.MagnetometerManager base id 0x10024000 \
    queue size 50 \
    stack size Default.STACK_SIZE \
    priority 100

  # ----------------------------------------------------------------------
  # Queued component instances
  # ----------------------------------------------------------------------


  # ----------------------------------------------------------------------
  # Passive component instances
  # ----------------------------------------------------------------------

  instance chronoTime: Svc.ChronoTime base id 0x10010000

  instance rateGroupDriver: Svc.RateGroupDriver base id 0x10011000

  instance systemResources: Svc.SystemResources base id 0x10012000

  instance timer: Svc.LinuxTimer base id 0x10013000
  instance comDriver: Drv.TcpServer base id 0x10014000

  # Bus driver instances
  instance imuI2cDriver: Drv.LinuxI2cDriver base id 0x10030000
  instance magI2cDriver: Drv.LinuxI2cDriver base id 0x10031000
  instance gpsI2cDriver: Drv.LinuxI2cDriver base id 0x10033000
  instance tempI2cDriver: Drv.LinuxI2cDriver base id 0x10034000
  instance radioSpiDriver: Drv.LinuxSpiDriver base id 0x10032000

}
