// TempManager.cpp

#include <cmath>
#include "CubesatRef/Components/TempManager/TempManager.hpp"


namespace Managers {

TempManager::TempManager(const char* const compName)
    : TempManagerComponentBase(compName),
      m_simTick(0) {}

TempManager::~TempManager() {}

void TempManager::run_handler(FwIndexType portNum, U32 context) {
    this->tempSm_sendSignal_tick();
}

// state machine actions

// kick off a read to make sure the tmp102 is actually there on the bus
void TempManager::Managers_TempManagerStateMachine_action_doInit(
    SmId smId, Managers_TempManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::tempSm);
    F32 temperature = 0.0f;
    Drv::I2cStatus status = this->readRawTemp(temperature);
    if (status == Drv::I2cStatus::I2C_OK) {
        this->log_ACTIVITY_HI_StateChange(TempManager_SensorState::RUNNING);
        this->tempSm_sendSignal_success();
    } else {
        this->log_ACTIVITY_HI_StateChange(TempManager_SensorState::FAULT);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, false);
        }
        this->tempSm_sendSignal_fault();
    }
}

// grab a fresh temp reading and push to telemetry, transition to FAULT on i2c error
void TempManager::Managers_TempManagerStateMachine_action_doRead(
    SmId smId, Managers_TempManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::tempSm);
    F32 temperature = 0.0f;
    Drv::I2cStatus status = this->readRawTemp(temperature);

    if (status == Drv::I2cStatus::I2C_OK) {
        this->tlmWrite_Temperature(temperature);
        if (++m_readCount % READ_LOG_INTERVAL == 0) {
            this->log_ACTIVITY_LO_TempReading(temperature);
        }
    } else {
        this->log_ACTIVITY_HI_StateChange(TempManager_SensorState::FAULT);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, false);
        }
        this->tempSm_sendSignal_fault();
    }
}

// retry the read, if it comes back we flip back to running
void TempManager::Managers_TempManagerStateMachine_action_doFaultRecovery(
    SmId smId, Managers_TempManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::tempSm);
    F32 temperature = 0.0f;
    Drv::I2cStatus status = this->readRawTemp(temperature);
    if (status == Drv::I2cStatus::I2C_OK) {
        this->tlmWrite_Temperature(temperature);
        this->log_ACTIVITY_HI_StateChange(TempManager_SensorState::RUNNING);
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, true);
        }
        this->tempSm_sendSignal_success();
    } else {
        if (this->isConnected_healthOut_OutputPort(0)) {
            this->healthOut_out(0, false);
        }
    }
}

// fake temp data wobbling around 25c when there's no hardware
void TempManager::Managers_TempManagerStateMachine_action_doSimRead(
    SmId smId, Managers_TempManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::tempSm);
    float t = static_cast<float>(m_simTick) * 0.1f;
    F32 temperature = 25.0f + 5.0f * sinf(t * 0.05f);
    m_simTick++;

    this->tlmWrite_Temperature(temperature);
}

// command handlers

void TempManager::READ_TEMP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    F32 temperature = 0.0f;
    Drv::I2cStatus status = this->readRawTemp(temperature);
    if (status == Drv::I2cStatus::I2C_OK) {
        this->tlmWrite_Temperature(temperature);
    }
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void TempManager::ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_simTick = 0;
    this->log_ACTIVITY_HI_StateChange(TempManager_SensorState::SIM);
    this->tempSm_sendSignal_enableSim();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void TempManager::DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_ACTIVITY_HI_StateChange(TempManager_SensorState::INIT);
    this->tempSm_sendSignal_disableSim();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// tmp102 returns a 12-bit signed value in the upper 12 bits of 2 bytes - shift right 4, sign-extend, scale by 0.0625
Drv::I2cStatus TempManager::readRawTemp(F32& temperature) {
    const U8 devAddr = 0x4a;  // ADD0 pin is tied to ground on our board, so address is 0x4a not the default 0x48
    const U8 tempReg = 0x00;  // temperature register

    U8 writeBuf[1] = {tempReg};
    U8 readBuf[2]  = {0};

    Fw::Buffer writeFwBuf(writeBuf, sizeof(writeBuf));
    Fw::Buffer readFwBuf(readBuf, sizeof(readBuf));

    Drv::I2cStatus status = this->busWriteRead_out(0, devAddr, writeFwBuf, readFwBuf);

    if (status == Drv::I2cStatus::I2C_OK) {
        // combine the two bytes big-endian style (MSB first from tmp102)
        I16 raw = (readBuf[0] << 8) | readBuf[1];

        // top 12 bits are the actual value, bottom 4 are flag bits we don't need
        raw >>= 4;

        // if bit 11 is set the value is negative - sign extend to full 16 bits
        if (raw & 0x800) {
            raw |= 0xF000;
        }

        // 0.0625C per count = 1/16 degree, which is the tmp102's resolution in normal 12-bit mode
        temperature = raw * 0.0625f;
    }

    return status;
}

}  // namespace Managers
