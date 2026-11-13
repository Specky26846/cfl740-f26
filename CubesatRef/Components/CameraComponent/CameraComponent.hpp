// ======================================================================
// \title  CameraComponent.hpp
// \author lauraf26846
// \brief  hpp file for CameraComponent component implementation class
// ======================================================================

#ifndef CubesatRef_CameraComponent_HPP
#define CubesatRef_CameraComponent_HPP

#include "CubesatRef/Components/CameraComponent/CameraComponentComponentAc.hpp"

namespace Managers {

class CameraComponent final : public CameraComponentComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct CameraComponent object
    CameraComponent(const char* const compName);

    //! Destroy CameraComponent object
    ~CameraComponent();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command TAKE_PHOTO
    //! Takes a picture using the specified camera and directory
    void TAKE_PHOTO_cmdHandler(
        FwOpcodeType opCode,                 //!< The opcode
        U32 cmdSeq,                          //!< Command sequence number
        U32 cameraId,                        //!< Camera index
        const Fw::CmdStringArg& filePath     //!< Image storage directory
    ) override;

  private:
    U32 m_photoCount = 0;
};

}  // namespace ProjectZero

#endif