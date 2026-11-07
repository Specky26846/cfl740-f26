// ======================================================================
// \title  PacketBuilder.hpp
// \author lauraf26846
// \brief  hpp file for PacketBuilder component implementation class
// ======================================================================

#ifndef CubesatRef_PacketBuilder_HPP
#define CubesatRef_PacketBuilder_HPP

#include "CubesatRef/Components/PacketBuilder/PacketBuilderComponentAc.hpp"

namespace CubesatRef {

class PacketBuilder final : public PacketBuilderComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct PacketBuilder object
    PacketBuilder(const char* const compName  //!< The component name
    );

    //! Destroy PacketBuilder object
    ~PacketBuilder();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command TODO
    //!
    //! TODO
    void TODO_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                         U32 cmdSeq            //!< The command sequence number
                         ) override;
};

}  // namespace CubesatRef

#endif
