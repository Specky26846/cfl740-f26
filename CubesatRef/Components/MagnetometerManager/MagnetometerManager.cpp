// MagnetometerManager.cpp

#include <cmath>
#include <cstring>
#include <unistd.h>
#include "CubesatRef/Components/MagnetometerManager/MagnetometerManager.hpp"


namespace Managers {

MagnetometerManager::MagnetometerManager(const char* const compName)
    : MagnetometerManagerComponentBase(compName),
      m_simTick(0) {}

MagnetometerManager::~MagnetometerManager() {}

void MagnetometerManager::configure(U32 i2cAddress) {
    this->m_i2cAddress = i2cAddress;
}

void MagnetometerManager::run_handler(FwIndexType portNum, U32 context) {
    this->magSm_sendSignal_tick();
}

// state machine actions

// try a poll read to verify i2c works, retries 3 times for bus contention on startup
void MagnetometerManager::Managers_MagnetometerManagerStateMachine_action_doInit(
    SmId smId, Managers_MagnetometerManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::magSm);
    F32 mx = 0, my = 0, mz = 0;
    Drv::I2cStatus status = Drv::I2cStatus::I2C_OTHER_ERR;

    for (U32 i = 0; i < 3; i++) {
        status = this->readMagData(mx, my, mz);
        if (status == Drv::I2cStatus::I2C_OK) {
            break;
        }
        usleep(200000);  // 200ms between retries - bus can still be settling at startup with gps also coming up
    }

    if (status == Drv::I2cStatus::I2C_OK) {
        this->log_ACTIVITY_HI_StateChange(MagnetometerManager_SensorState::RUNNING);
        this->magSm_sendSignal_success();
    } else {
        this->log_ACTIVITY_HI_StateChange(MagnetometerManager_SensorState::FAULT);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, false);
        }
        this->magSm_sendSignal_fault();
    }
}

// reads mag data and writes telemetry, faults on i2c error
void MagnetometerManager::Managers_MagnetometerManagerStateMachine_action_doRead(
    SmId smId, Managers_MagnetometerManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::magSm);
    F32 mx = 0, my = 0, mz = 0;
    Drv::I2cStatus status = this->readMagData(mx, my, mz);

    if (status != Drv::I2cStatus::I2C_OK) {
        this->log_ACTIVITY_HI_StateChange(MagnetometerManager_SensorState::FAULT);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, false);
        }
        this->magSm_sendSignal_fault();
        return;
    }

    this->tlmWrite_MagX(mx);
    this->tlmWrite_MagY(my);
    this->tlmWrite_MagZ(mz);
}

// tries to read again, if it works we're back to running
void MagnetometerManager::Managers_MagnetometerManagerStateMachine_action_doFaultRecovery(
    SmId smId, Managers_MagnetometerManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::magSm);
    F32 mx = 0, my = 0, mz = 0;
    Drv::I2cStatus status = this->readMagData(mx, my, mz);
    if (status == Drv::I2cStatus::I2C_OK) {
        this->tlmWrite_MagX(mx);
        this->tlmWrite_MagY(my);
        this->tlmWrite_MagZ(mz);
        this->log_ACTIVITY_HI_StateChange(MagnetometerManager_SensorState::RUNNING);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, true);
        }
        this->magSm_sendSignal_success();
    }
}

// generates fake sinusoidal mag data for testing without hardware
void MagnetometerManager::Managers_MagnetometerManagerStateMachine_action_doSimRead(
    SmId smId, Managers_MagnetometerManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::magSm);
    F32 mx = 0, my = 0, mz = 0;
    this->simulateMagData(mx, my, mz);
    m_simTick++;

    this->tlmWrite_MagX(mx);
    this->tlmWrite_MagY(my);
    this->tlmWrite_MagZ(mz);
}

// command handlers
void MagnetometerManager::CALIBRATE_MAG_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_ACTIVITY_HI_MagCalibrationStarted();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void MagnetometerManager::ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_simTick = 0;
    this->log_ACTIVITY_HI_StateChange(MagnetometerManager_SensorState::SIM);
    this->magSm_sendSignal_enableSim();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void MagnetometerManager::DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_ACTIVITY_HI_StateChange(MagnetometerManager_SensorState::INIT);
    this->magSm_sendSignal_disableSim();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// rm3100 poll mode: write to POLL_REG, wait 25ms for conversion, read 9 bytes (3 per axis, big-endian 24-bit signed)
Drv::I2cStatus MagnetometerManager::readMagData(F32& mx, F32& my, F32& mz) {
    // 0x70 = measure all 3 axes in poll mode
    U8 pollCmd[2] = {POLL_REG, 0x70};
    Fw::Buffer pollBuffer(pollCmd, sizeof(pollCmd));
    Drv::I2cStatus status = this->busWrite_out(0, this->m_i2cAddress, pollBuffer);
    if (status != Drv::I2cStatus::I2C_OK) {
        // gps can hold the bus for a bit so retry once after 10ms
        usleep(10000);
        pollCmd[0] = POLL_REG;
        pollCmd[1] = 0x70;
        pollBuffer = Fw::Buffer(pollCmd, sizeof(pollCmd));
        status = this->busWrite_out(0, this->m_i2cAddress, pollBuffer);
        if (status != Drv::I2cStatus::I2C_OK) {
            return status;  // still failing, let the state machine know
        }
    }

    usleep(25000);  // rm3100 conversion time at default cycle count 200 - from datasheet table 4

    // set up the read from MX_REG - gives us 9 bytes: mx(3), my(3), mz(3)
    U8 regAddr = MX_REG;
    Fw::Buffer writeBuffer(&regAddr, sizeof(regAddr));
    U8 rawData[MAG_DATA_SIZE] = {};
    Fw::Buffer readBuffer(rawData, sizeof(rawData));

    status = this->busWriteRead_out(0, this->m_i2cAddress, writeBuffer, readBuffer);

    // retry read once if bus was still busy
    if (status != Drv::I2cStatus::I2C_OK) {
        usleep(10000);
        regAddr = MX_REG;
        writeBuffer = Fw::Buffer(&regAddr, sizeof(regAddr));
        memset(rawData, 0, sizeof(rawData));
        readBuffer = Fw::Buffer(rawData, sizeof(rawData));
        status = this->busWriteRead_out(0, this->m_i2cAddress, writeBuffer, readBuffer);
    }

    if (status == Drv::I2cStatus::I2C_OK) {
        // assemble 24-bit big-endian values (3 bytes per axis, MSB first)
        I32 rawMx = (static_cast<I32>(rawData[0]) << 16) |
                    (static_cast<I32>(rawData[1]) << 8) |
                     static_cast<I32>(rawData[2]);
        I32 rawMy = (static_cast<I32>(rawData[3]) << 16) |
                    (static_cast<I32>(rawData[4]) << 8) |
                     static_cast<I32>(rawData[5]);
        I32 rawMz = (static_cast<I32>(rawData[6]) << 16) |
                    (static_cast<I32>(rawData[7]) << 8) |
                     static_cast<I32>(rawData[8]);

        // sign extend from 24-bit to 32-bit so negative values work correctly
        if (rawMx & 0x800000) { rawMx |= 0xFF000000; }
        if (rawMy & 0x800000) { rawMy |= 0xFF000000; }
        if (rawMz & 0x800000) { rawMz |= 0xFF000000; }

        // 75 counts/uT is the RM3100 sensitivity at cycle count 200 (the default) per datasheet table 2
        mx = static_cast<F32>(rawMx) / RM3100_SENSITIVITY;
        my = static_cast<F32>(rawMy) / RM3100_SENSITIVITY;
        mz = static_cast<F32>(rawMz) / RM3100_SENSITIVITY;
    }

    return status;
}

// earth field sim - 20/20/-40 uT base, phase offsets so the axes aren't in sync
void MagnetometerManager::simulateMagData(F32& mx, F32& my, F32& mz) {
    float t = static_cast<float>(m_simTick) * 0.1f;
    mx = 20.0f  + 5.0f * sinf(t);
    my = 20.0f  + 5.0f * sinf(t + 1.0f);
    mz = -40.0f + 5.0f * sinf(t + 2.0f);  // z tends negative in northern hemisphere
}

}  // namespace Managers
