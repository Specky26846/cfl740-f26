// ======================================================================
// \title  RapEncoder.hpp
// \author lauraf26846
// \brief  RAP (MXL Radio Application Packet) framing - port of encode.py
// ======================================================================

#ifndef CubesatRef_RapEncoder_HPP
#define CubesatRef_RapEncoder_HPP

#include <Fw/FPrimeBasicTypes.hpp>

namespace CubesatRef {
namespace Rap {

// rap flags - lower nibble is packet type, upper nibble is action
static constexpr U8 FLAG_TAP = 0x00;
static constexpr U8 FLAG_DAP = 0x01;
static constexpr U8 FLAG_BEACON = 0x03;
static constexpr U8 FLAG_ACK_REQ = 0x80;

static constexpr U8 SYNC_0 = 0xAB;
static constexpr U8 SYNC_1 = 0xCD;

// spacecraft ids from encode.py (PID = 495, SID = 28 for team alpha)
static constexpr U16 PRIMARY_ID = 495;
static constexpr U16 SECONDARY_ID = 28;

static constexpr FwSizeType HEAD_LENGTH = 11;  // sync(2) pid(2) sid(2) flags(1) len(2) hdr cksum(2)
static constexpr FwSizeType FOOT_LENGTH = 6;   // data cksum(2) hmac(4)
static constexpr FwSizeType MAX_DATA_LENGTH = 238;

//! fletcher-16 exactly as encode.py's checksum(): out[0] = s0, out[1] = s1
void fletcher16(const U8* data, FwSizeType len, U8 out[2]);

//! wrap data in a rap header/footer, same bytes as encode.py's encode_rap()
//! \return total packet length, or 0 if data is too long or out is too small
FwSizeType encode(U8 flag, const U8* data, FwSizeType dataLen, U8* out, FwSizeType outCapacity);

}  // namespace Rap
}  // namespace CubesatRef

#endif