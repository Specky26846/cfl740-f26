// ======================================================================
// \title  HubTest.hpp
// \author lauraf26846
// \brief  hpp file for HubTest component implementation class
// ======================================================================

#ifndef hubTest_HubTest_HPP
#define hubTest_HubTest_HPP

#include "Cfl740_f26/Components/HubTest/HubTestComponentAc.hpp"

namespace hubTest {

class HubTest final : public HubTestComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct HubTest object
    HubTest(const char* const compName  //!< The component name
    );

    //! Destroy HubTest object
    ~HubTest();

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

}  // namespace hubTest

#endif
