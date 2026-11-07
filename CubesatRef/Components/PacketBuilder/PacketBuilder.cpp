// ======================================================================
// \title  PacketBuilder.cpp
// \author lauraf26846
// \brief  cpp file for PacketBuilder component implementation class
// ======================================================================

#include "CubesatRef/Components/PacketBuilder/PacketBuilder.hpp"

namespace CubesatRef {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

PacketBuilder ::PacketBuilder(const char* const compName) : PacketBuilderComponentBase(compName) {}

PacketBuilder ::~PacketBuilder() {}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void PacketBuilder ::TODO_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace CubesatRef
