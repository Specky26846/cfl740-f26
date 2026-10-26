// PduManager.cpp
//
// port of the eddy_eps.py / ads7828.py scripts (Justin Schachter) to an fprime component.
// the eddy pdu has two ads7828 8-channel 12-bit adcs; the schematic numbers their inputs
// ADC_0..ADC_15 where 0-7 live on the adc at 0x48 and 8-15 on the adc at 0x49.

#include <cmath>
#include "CubesatRef/Components/PduManager/PduManager.hpp"

namespace {

// i2c addresses of the two ads7828s (A1/A0 = LL and LH)
constexpr U8 ADC0_ADDR = 0x48;
constexpr U8 ADC1_ADDR = 0x49;
constexpr U8 CHANNELS_PER_ADC = 8;

// command byte bits - see ads7828 datasheet page 11
constexpr U8 CMD_SINGLE_ENDED = 0x80;
constexpr U8 CMD_INTERNAL_REF_ON_AD_ON = 0x0C;

// internal reference, 12-bit converter
constexpr F32 VREF = 2.5f;
constexpr F32 COUNTS = 4096.0f;

// eps channel assignments for rev A (ADC_XX on the output regulation schematic)
constexpr U8 CH_I_5V0 = 0;
constexpr U8 CH_V_5V0 = 1;
constexpr U8 CH_I_3V3 = 2;
constexpr U8 CH_V_3V3 = 3;
constexpr U8 CH_LM20_5V0_REG = 4;
constexpr U8 CH_V_VBATT_RAW = 5;
constexpr U8 CH_I_VBATT_RAW = 6;
constexpr U8 CH_LM20_3V3_REG = 7;
constexpr U8 CH_V_VBATT = 13;
constexpr U8 CH_I_VBATT = 14;

// undo the resistor divider in front of the adc pin
F32 dividerReversal(F32 v2, F32 r1, F32 r2) {
    return v2 * ((r1 + r2) / r2);
}

// max9634 current sense amp: v_out = i * r_sense * gain
F32 max9634VtoI(F32 vSense, F32 rSense, F32 gain) {
    return vSense / (rSense * gain);
}

// lm20 transfer function from the datasheet, returns degrees c
F32 lm20DegC(F32 voltage) {
    return -1481.96f + sqrtf(2.1962e6f + ((1.8639f - voltage) / 3.88e-6f));
}

// single-ended channel select bits aren't in channel order: C2 is the low bit of the
// channel, C1:C0 are the upper two (CH1 = 0b100, CH2 = 0b001, ...)
U8 singleEndedCommand(U8 adcChannel) {
    U8 select = static_cast<U8>((adcChannel >> 1) | ((adcChannel & 0x1) << 2));
    return static_cast<U8>(CMD_SINGLE_ENDED | (select << 4) | CMD_INTERNAL_REF_ON_AD_ON);
}

}  // namespace

namespace Managers {

PduManager::PduManager(const char* const compName)
    : PduManagerComponentBase(compName),
      m_simTick(0) {}

PduManager::~PduManager() {}

void PduManager::run_handler(FwIndexType portNum, U32 context) {
    this->pduSm_sendSignal_tick();
}

// state machine actions

// send a command byte to each adc - this both checks they ack on the bus and turns the
// internal reference on so it has a tick to settle before the first real conversion
void PduManager::Managers_PduManagerStateMachine_action_doInit(
    SmId smId, Managers_PduManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::pduSm);
    U8 cmd[1] = {singleEndedCommand(0)};
    Fw::Buffer cmdBuf(cmd, sizeof(cmd));

    Drv::I2cStatus status = this->busWrite_out(0, ADC0_ADDR, cmdBuf);
    if (status == Drv::I2cStatus::I2C_OK) {
        cmdBuf.setSize(sizeof(cmd));
        status = this->busWrite_out(0, ADC1_ADDR, cmdBuf);
    }

    if (status == Drv::I2cStatus::I2C_OK) {
        this->log_ACTIVITY_HI_StateChange(PduManager_SensorState::RUNNING);
        this->pduSm_sendSignal_success();
    } else {
        this->log_ACTIVITY_HI_StateChange(PduManager_SensorState::FAULT);
        this->reportHealth(false);
        this->pduSm_sendSignal_fault();
    }
}

// read the whole board and push to telemetry, transition to FAULT on i2c error
void PduManager::Managers_PduManagerStateMachine_action_doRead(
    SmId smId, Managers_PduManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::pduSm);
    PduReadings readings;
    Drv::I2cStatus status = this->readAll(readings);

    if (status == Drv::I2cStatus::I2C_OK) {
        this->writeTelemetry(readings);
        if (++m_readCount % READ_LOG_INTERVAL == 0) {
            this->log_ACTIVITY_LO_PduReading(readings.vbattRawV, readings.i3v3, readings.i5v0);
        }
    } else {
        this->log_ACTIVITY_HI_StateChange(PduManager_SensorState::FAULT);
        this->reportHealth(false);
        this->pduSm_sendSignal_fault();
    }
}

// retry the read, if it comes back we flip back to running
void PduManager::Managers_PduManagerStateMachine_action_doFaultRecovery(
    SmId smId, Managers_PduManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::pduSm);
    PduReadings readings;
    Drv::I2cStatus status = this->readAll(readings);
    if (status == Drv::I2cStatus::I2C_OK) {
        this->writeTelemetry(readings);
        this->log_ACTIVITY_HI_StateChange(PduManager_SensorState::RUNNING);
        this->reportHealth(true);
        this->pduSm_sendSignal_success();
    } else {
        this->reportHealth(false);
    }
}

// fake rail data when there's no hardware - battery slowly discharging, small load ripple
void PduManager::Managers_PduManagerStateMachine_action_doSimRead(
    SmId smId, Managers_PduManagerStateMachine::Signal signal)
{
    FW_ASSERT(smId == SmId::pduSm);
    float t = static_cast<float>(m_simTick) * 0.1f;
    float ripple = sinf(t * 0.5f);
    m_simTick++;

    PduReadings readings;
    readings.vbattRawV = 8.2f - 0.0005f * static_cast<float>(m_simTick % 2000);
    readings.vbattRawI = 0.45f + 0.05f * ripple;
    readings.v3v3 = 3.30f + 0.01f * ripple;
    readings.i3v3 = 0.25f + 0.03f * ripple;
    readings.v5v0 = 5.00f + 0.01f * ripple;
    readings.i5v0 = 0.15f + 0.02f * ripple;
    readings.vbattV = readings.vbattRawV - 0.05f;
    readings.vbattI = 0.05f;
    readings.temp3v3Reg = 30.0f + 2.0f * sinf(t * 0.05f);
    readings.temp5v0Reg = 32.0f + 2.0f * sinf(t * 0.05f);

    this->writeTelemetry(readings);
}

// command handlers

void PduManager::READ_PDU_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    PduReadings readings;
    Drv::I2cStatus status = this->readAll(readings);
    if (status == Drv::I2cStatus::I2C_OK) {
        this->writeTelemetry(readings);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
    }
}

void PduManager::ENABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_simTick = 0;
    this->log_ACTIVITY_HI_StateChange(PduManager_SensorState::SIM);
    this->pduSm_sendSignal_enableSim();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void PduManager::DISABLE_SIM_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->log_ACTIVITY_HI_StateChange(PduManager_SensorState::INIT);
    this->pduSm_sendSignal_disableSim();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// helpers

// resistor values and sense amp gains are straight from eddy_eps.py (rev A)
Drv::I2cStatus PduManager::readAll(PduReadings& readings) {
    F32 v = 0.0f;
    Drv::I2cStatus status;

    if ((status = this->readChannel(CH_V_VBATT_RAW, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.vbattRawV = dividerReversal(v, 100e3f, 12e3f);
    if ((status = this->readChannel(CH_I_VBATT_RAW, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.vbattRawI = max9634VtoI(v, 12e-3f, 100.0f);

    if ((status = this->readChannel(CH_V_3V3, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.v3v3 = dividerReversal(v, 100e3f, 60.4e3f);
    if ((status = this->readChannel(CH_I_3V3, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.i3v3 = max9634VtoI(v, 5e-3f, 100.0f);

    if ((status = this->readChannel(CH_V_5V0, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.v5v0 = dividerReversal(v, 100e3f, 33.2e3f);
    if ((status = this->readChannel(CH_I_5V0, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.i5v0 = max9634VtoI(v, 12e-3f, 100.0f);

    if ((status = this->readChannel(CH_V_VBATT, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.vbattV = dividerReversal(v, 100e3f, 12e3f);
    if ((status = this->readChannel(CH_I_VBATT, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.vbattI = max9634VtoI(v, 12e-3f, 100.0f);

    if ((status = this->readChannel(CH_LM20_3V3_REG, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.temp3v3Reg = lm20DegC(v);
    if ((status = this->readChannel(CH_LM20_5V0_REG, v)) != Drv::I2cStatus::I2C_OK) return status;
    readings.temp5v0Reg = lm20DegC(v);

    return Drv::I2cStatus::I2C_OK;
}

// writing the command byte starts the conversion, then 2 bytes come back with the
// 12-bit count in the low nibble of byte 0 + all of byte 1
Drv::I2cStatus PduManager::readChannel(U8 epsChannel, F32& voltage) {
    FW_ASSERT(epsChannel < 2 * CHANNELS_PER_ADC, epsChannel);
    const U8 devAddr = (epsChannel < CHANNELS_PER_ADC) ? ADC0_ADDR : ADC1_ADDR;
    const U8 adcChannel = epsChannel % CHANNELS_PER_ADC;

    U8 writeBuf[1] = {singleEndedCommand(adcChannel)};
    U8 readBuf[2]  = {0};

    Fw::Buffer writeFwBuf(writeBuf, sizeof(writeBuf));
    Fw::Buffer readFwBuf(readBuf, sizeof(readBuf));

    Drv::I2cStatus status = this->busWriteRead_out(0, devAddr, writeFwBuf, readFwBuf);

    if (status == Drv::I2cStatus::I2C_OK) {
        U16 counts = static_cast<U16>(((readBuf[0] & 0x0F) << 8) | readBuf[1]);
        voltage = static_cast<F32>(counts) * (VREF / COUNTS);
    }

    return status;
}

void PduManager::writeTelemetry(const PduReadings& readings) {
    this->tlmWrite_VBattRawVoltage(readings.vbattRawV);
    this->tlmWrite_VBattRawCurrent(readings.vbattRawI);
    this->tlmWrite_Rail3v3Voltage(readings.v3v3);
    this->tlmWrite_Rail3v3Current(readings.i3v3);
    this->tlmWrite_Rail5v0Voltage(readings.v5v0);
    this->tlmWrite_Rail5v0Current(readings.i5v0);
    this->tlmWrite_VBattVoltage(readings.vbattV);
    this->tlmWrite_VBattCurrent(readings.vbattI);
    this->tlmWrite_Reg3v3Temp(readings.temp3v3Reg);
    this->tlmWrite_Reg5v0Temp(readings.temp5v0Reg);
}

void PduManager::reportHealth(bool healthy) {
    if (this->isConnected_healthOut_OutputPort(0)) {
        this->healthOut_out(0, healthy);
    }
}

}  // namespace Managers
