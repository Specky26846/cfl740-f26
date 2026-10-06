// ======================================================================
// \title  HubTest.cpp
// \author lauraf26846
// \brief  cpp file for HubTest component implementation class
// ======================================================================

#include "Cfl740_f26/Components/HubTest/HubTest.hpp"

namespace hubTest {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

HubTest ::HubTest(const char* const compName) : HubTestComponentBase(compName) {}

HubTest ::~HubTest() {}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void HubTest ::TODO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace hubTest
