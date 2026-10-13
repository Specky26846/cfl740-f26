// ======================================================================
// \title  RadioManager.cpp
// \author tyler
// \brief  cpp file for RadioManager component implementation class
// ======================================================================

#include "Cfl740_f26/Components/RadioManager/RadioManager.hpp"

namespace Cfl740_f26 {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

RadioManager ::RadioManager(const char* const compName) : RadioManagerComponentBase(compName) {}

RadioManager ::~RadioManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void RadioManager ::dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    // TODO
}

void RadioManager ::dataReturnIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) {
    // TODO
}

void RadioManager ::run_handler(FwIndexType portNum, U32 context) {
    // TODO
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void RadioManager ::READ_REGISTER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 addr) {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void RadioManager ::WRITE_REGISTER_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, U8 addr, U8 value) {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void RadioManager ::RESET_RADIO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined internal interfaces
// ----------------------------------------------------------------------

void RadioManager ::transmitFrame_internalInterfaceHandler(const Fw::Buffer& data,
                                                           const ComCfg::FrameContext& context) {
    // TODO
}

}  // namespace Cfl740_f26
