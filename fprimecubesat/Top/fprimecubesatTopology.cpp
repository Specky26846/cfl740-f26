// ======================================================================
// \title  fprimecubesatTopology.cpp
// \brief cpp file containing the topology instantiation code
//
// ======================================================================
// Provides access to autocoded functions
#include <fprimecubesat/Top/fprimecubesatTopologyAc.hpp>
// Note: Uncomment when using Svc:TlmPacketizer
//#include <fprimecubesat/Top/fprimecubesatPacketsAc.hpp>

// Necessary project-specified types
#include <Fw/Types/MallocAllocator.hpp>
#include <Fw/Logger/Logger.hpp>

// Allows easy reference to objects in FPP/autocoder required namespaces
using namespace fprimecubesat;

// Instantiate a malloc allocator for cmdSeq buffer allocation
Fw::MallocAllocator mallocator;

// rate group divisors: timer ticks at 10hz (100ms)
//   rateGroup1: divider 10 -> 1hz  (system, radio, temp, nav, imu managers)
//   rateGroup2: divider 1  -> 10hz (cmdSeq)
//   rateGroup3: divider 4  -> 2.5hz (health, buffers, data products, magnetometer)
Svc::RateGroupDriver::DividerSet rateGroupDivisorsSet{{{10, 0}, {1, 0}, {4, 0}}};

// Rate groups may supply a context token to each of the attached children whose purpose is set by the project. The
// reference topology sets each token to zero as these contexts are unused in this project.
U32 rateGroup1Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};
U32 rateGroup2Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};
U32 rateGroup3Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};

enum TopologyConstants {
    COMM_PRIORITY = 100,
};

/**
 * \brief configure/setup components in project-specific way
 *
 * This is a *helper* function which configures/sets up each component requiring project specific input. This includes
 * allocating resources, passing-in arguments, etc. This function may be inlined into the topology setup function if
 * desired, but is extracted here for clarity.
 */
void configureTopology() {
    // Rate group driver needs a divisor list
    rateGroupDriver.configure(rateGroupDivisorsSet);

    // Rate groups require context arrays.
    rateGroup1.configure(rateGroup1Context, FW_NUM_ARRAY_ELEMENTS(rateGroup1Context));
    rateGroup2.configure(rateGroup2Context, FW_NUM_ARRAY_ELEMENTS(rateGroup2Context));
    rateGroup3.configure(rateGroup3Context, FW_NUM_ARRAY_ELEMENTS(rateGroup3Context));

    // Command sequencer needs to allocate memory to hold contents of command sequences
    cmdSeq.allocateBuffer(0, mallocator, 5 * 1024);

    // GPIO drivers for LEDs
    Os::File::Status status =
        gpioDriver.open("/dev/gpiochip4", 13, Drv::LinuxGpioDriver::GpioConfiguration::GPIO_OUTPUT);
    if (status != Os::File::Status::OP_OK) {
        Fw::Logger::log("[ERROR] Failed to open GPIO pin 13\n");
    }
    Os::File::Status status2 =
        gpioDriver2.open("/dev/gpiochip4", 17, Drv::LinuxGpioDriver::GpioConfiguration::GPIO_OUTPUT);
    if (status2 != Os::File::Status::OP_OK) {
        Fw::Logger::log("[ERROR] Failed to open GPIO pin 17\n");
    }
    // TODO: pin 27 is a placeholder for led2 - confirm against real wiring and update
    Os::File::Status status3 =
        gpioDriver3.open("/dev/gpiochip4", 27, Drv::LinuxGpioDriver::GpioConfiguration::GPIO_OUTPUT);
    if (status3 != Os::File::Status::OP_OK) {
        Fw::Logger::log("[ERROR] Failed to open GPIO pin 27\n");
    }

    // I2C driver for IMU (IMUManager) - Raspberry Pi 4 I2C bus 1
    bool i2cImuOk = imuI2cDriver.open("/dev/i2c-1");
    if (!i2cImuOk) {
        Fw::Logger::log("[ERROR] Failed to open I2C for IMU\n");
    }

    // I2C driver for Magnetometer - Raspberry Pi 4 I2C bus 1
    bool i2cMagOk = magI2cDriver.open("/dev/i2c-1");
    if (!i2cMagOk) {
        Fw::Logger::log("[ERROR] Failed to open I2C for Magnetometer\n");
    }

    // I2C driver for GPS (NEO-M9N) - same physical bus as Magnetometer
    bool i2cGpsOk = gpsI2cDriver.open("/dev/i2c-1");
    if (!i2cGpsOk) {
        Fw::Logger::log("[ERROR] Failed to open I2C for GPS\n");
    }

    // I2C driver for TMP102 temperature sensor
    bool i2cTempOk = tempI2cDriver.open("/dev/i2c-1");
    if (!i2cTempOk) {
        Fw::Logger::log("[ERROR] Failed to open I2C for temperature sensor\n");
    }

    // SPI driver for Radio - Raspberry Pi 4 SPI bus 0, chip select 0
    bool spiOk = radioSpiDriver.open(0, 0, Drv::SPI_FREQUENCY_1MHZ);
    if (!spiOk) {
        Fw::Logger::log("[ERROR] Failed to open SPI for Radio\n");
    }

    // Configure manager I2C addresses
    imuManager.configure(0x68);
    magnetometerManager.configure(0x20);
    navigationManager.configure();
}

// Public functions for use in main program are namespaced with deployment name fprimecubesat
namespace fprimecubesat {
void setupTopology(const TopologyState& state) {
    // Autocoded initialization. Function provided by autocoder.
    initComponents(state);
    // Autocoded id setup. Function provided by autocoder.
    setBaseIds();
    // Autocoded connection wiring. Function provided by autocoder.
    connectComponents();
    // Autocoded command registration. Function provided by autocoder.
    regCommands();
    // Autocoded configuration. Function provided by autocoder.
    configComponents(state);
    if (state.hostname != nullptr && state.port != 0) {
        comDriver.configure(state.hostname, state.port);
    }
    // Project-specific component configuration. Function provided above. May be inlined, if desired.
    configureTopology();
    // Autocoded parameter loading. Function provided by autocoder.
    loadParameters();
    // Autocoded task kick-off (active components). Function provided by autocoder.
    startTasks(state);
    // Initialize socket communication if and only if there is a valid specification
    if (state.hostname != nullptr && state.port != 0) {
        Os::TaskString name("ReceiveTask");
        // Uplink is configured for receive so a socket task is started
        comDriver.start(name, COMM_PRIORITY, Default::STACK_SIZE);
    }
}

void startRateGroups(Fw::TimeInterval interval) {
    // The timer component drives the fundamental tick rate of the system.
    // Svc::RateGroupDriver will divide this down to the slower rate groups.
    // This call will block until the stopRateGroups() call is made.
    // For this Linux demo, that call is made from a signal handler.
    // Set timer to 10Hz (100ms period)
    timer.startTimer(interval.getSeconds() * 1000 + interval.getUSeconds() / 1000); // 100 ms = 10Hz
}

void stopRateGroups() {
    timer.quit();
}

void teardownTopology(const TopologyState& state) {
    // Autocoded (active component) task clean-up. Functions provided by topology autocoder.
    stopTasks(state);
    freeThreads(state);

    // Other task clean-up.
    comDriver.terminate();
    comDriver.stop();
    (void)comDriver.join();

    // Resource deallocation
    cmdSeq.deallocateBuffer(mallocator);

    tearDownComponents(state);
}
};  // namespace fprimecubesat
