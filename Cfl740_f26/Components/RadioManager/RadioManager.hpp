// ======================================================================
// \title  RadioManager.hpp
// \author tyler
// \brief  hpp file for RadioManager component implementation class
// ======================================================================

#ifndef Cfl740_f26_RadioManager_HPP
#define Cfl740_f26_RadioManager_HPP

#include "Cfl740_f26/Components/RadioManager/RadioManagerComponentAc.hpp"

namespace Cfl740_f26 {

class RadioManager final : public RadioManagerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct RadioManager object
    RadioManager(const char* const compName  //!< The component name
    );

    //! Destroy RadioManager object
    ~RadioManager();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for dataIn
    //!
    //! Data to be sent on the wire (coming in to the component)
    void dataIn_handler(FwIndexType portNum,  //!< The port number
                        Fw::Buffer& data,
                        const ComCfg::FrameContext& context) override;

    //! Handler implementation for dataReturnIn
    //!
    //! Port receiving back ownership of buffer sent out on dataOut
    void dataReturnIn_handler(FwIndexType portNum,  //!< The port number
                              Fw::Buffer& data,
                              const ComCfg::FrameContext& context) override;

    //! Handler implementation for run
    //!
    //! Port receiving calls from the rate group
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command READ_REGISTER
    //!
    //! Read one radio register and report as an event
    void READ_REGISTER_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                  U32 cmdSeq,           //!< The command sequence number
                                  U8 addr) override;

    //! Handler implementation for command WRITE_REGISTER
    //!
    //! Write one radio register
    void WRITE_REGISTER_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                   U32 cmdSeq,           //!< The command sequence number
                                   U8 addr,
                                   U8 value) override;

    //! Handler implementation for command RESET_RADIO
    //!
    //! Reset radio
    void RESET_RADIO_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                U32 cmdSeq            //!< The command sequence number
                                ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for user-defined internal interfaces
    // ----------------------------------------------------------------------

    //! Handler implementation for transmitFrame
    //!
    //! Moves frame from dataIn onto our thread
    void transmitFrame_internalInterfaceHandler(const Fw::Buffer& data, const ComCfg::FrameContext& context) override;
};

}  // namespace Cfl740_f26

#endif
