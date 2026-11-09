#include "CubesatRef/Components/PacketBuilder/RapEncoder.hpp"

#include <cstring>

namespace CubesatRef {
namespace Rap {

    // this is checksum() from encode.py
    void fletcher16(const U8* data, FwSizeType len, U8 out[2]) {
        U32 s0 = 0xff; // sums go past 255 before being folded, like python ints
        U32 s1 = 0xff;
        FwSizeType i = 0;
        while (len > 0) {
            FwSizeType tlen = (len > 21) ? 21 : len; // input data is split into 8 bit blocks
            len -= tlen;
            while (tlen > 0) {
                s0 += data[i++];
                s1 += s0;
                tlen--;
            }
            s0 = (s0 & 0xff) + (s0 >> 8);
            s1 = (s1 & 0xff) + (s1 >> 8);
        }
        s0 = (s0 & 0xff) + (s0 >> 8);
        s1 = (s1 & 0xff) + (s1 >> 8);
        out[0] = static_cast<U8>(s0);
        out[1] = static_cast<U8>(s1);
    }

    namespace { // start of anonymous namespace for private helpers

        // this class is similar to hashlib from Python, standard algorithm that F Prime does not have built-in
        class Sha1 {
        public:
            static constexpr FwSizeType DIGEST_LENGTH = 20;

            Sha1() : m_h{0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0}, m_blockLen(0), m_totalLen(0) {}

            void update(const U8* data, FwSizeType len) {
                for (FwSizeType i = 0; i < len; i++) {
                    m_block[m_blockLen++] = data[i];
                    if (m_blockLen == 64) {
                        this->processBlock();
                        m_blockLen = 0;
                    }
                }
                m_totalLen += len;
            }

            void update(U8 byte) { this->update(&byte, 1); }

            void digest(U8 out[DIGEST_LENGTH]) {
                const U64 bitLen = m_totalLen * 8;
                // pad with 0x80 then zeros until 8 bytes short of a block, then the 64-bit big-endian length
                U8 pad = 0x80;
                this->update(&pad, 1);
                pad = 0x00;
                while (m_blockLen != 56) {
                    this->update(&pad, 1);
                }
                for (I32 i = 7; i >= 0; i--) {
                    m_block[m_blockLen++] = static_cast<U8>(bitLen >> (i * 8));
                }
                this->processBlock();
                for (U32 i = 0; i < 5; i++) {
                    out[i * 4 + 0] = static_cast<U8>(m_h[i] >> 24);
                    out[i * 4 + 1] = static_cast<U8>(m_h[i] >> 16);
                    out[i * 4 + 2] = static_cast<U8>(m_h[i] >> 8);
                    out[i * 4 + 3] = static_cast<U8>(m_h[i]);
                }
            }

        private:
            static U32 rol(U32 x, U32 n) { return (x << n) | (x >> (32 - n)); }

            void processBlock() {
                U32 w[80];
                for (U32 i = 0; i < 16; i++) {
                    w[i] = (static_cast<U32>(m_block[i * 4]) << 24) | (static_cast<U32>(m_block[i * 4 + 1]) << 16) |
                        (static_cast<U32>(m_block[i * 4 + 2]) << 8) | static_cast<U32>(m_block[i * 4 + 3]);
                }
                for (U32 i = 16; i < 80; i++) {
                    w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
                }
                U32 a = m_h[0], b = m_h[1], c = m_h[2], d = m_h[3], e = m_h[4];
                for (U32 i = 0; i < 80; i++) {
                    U32 f, k;
                    if (i < 20) {
                        f = (b & c) | (~b & d);
                        k = 0x5A827999;
                    } else if (i < 40) {
                        f = b ^ c ^ d;
                        k = 0x6ED9EBA1;
                    } else if (i < 60) {
                        f = (b & c) | (b & d) | (c & d);
                        k = 0x8F1BBCDC;
                    } else {
                        f = b ^ c ^ d;
                        k = 0xCA62C1D6;
                    }
                    const U32 temp = rol(a, 5) + f + e + k + w[i];
                    e = d;
                    d = c;
                    c = rol(b, 30);
                    b = a;
                    a = temp;
                }
                m_h[0] += a;
                m_h[1] += b;
                m_h[2] += c;
                m_h[3] += d;
                m_h[4] += e;
            }

            U32 m_h[5];
            U8 m_block[64];
            FwSizeType m_blockLen;
            U64 m_totalLen;
        };

        // calculate_hmac() from encode.py (17 bytes, trailing 0x00 is part of it)
        const U8 KEY_BYTES[] = {0x56, 0xAF, 0x55, 0x1a, 0x30, 0x56, 0xf0, 0x13, 0xfa,
                                0x85, 0x82, 0x14, 0xac, 0x91, 0x81, 0x77, 0x00};

        // not standard hmac - the inner digest is cut to 4 bytes and reversed before the outer
        // hash, and the sid is hashed little-endian even though the packet sends it big-endian.
        // kept identical to encode.py so the ground side verifies it.
        void calculateHmac(U8 flag, U16 length, const U8 headerCksum[2], const U8* data, FwSizeType dataLen,
                        const U8 dataCksum[2], U8 out[4]) {
            U8 ipad[64];
            U8 opad[64];
            memset(ipad, 0x36, sizeof(ipad));
            memset(opad, 0x5c, sizeof(opad));
            for (FwSizeType i = 0; i < sizeof(KEY_BYTES); i++) {
                ipad[i] ^= KEY_BYTES[i];
                opad[i] ^= KEY_BYTES[i];
            }

            Sha1 inner;
            inner.update(ipad, sizeof(ipad));
            inner.update(SYNC_0);
            inner.update(SYNC_1);
            inner.update(static_cast<U8>(PRIMARY_ID & 0xFF));    // little-endian
            inner.update(static_cast<U8>(PRIMARY_ID >> 8));
            inner.update(static_cast<U8>(SECONDARY_ID & 0xFF));  // little-endian here!
            inner.update(static_cast<U8>(SECONDARY_ID >> 8));
            inner.update(flag);
            inner.update(static_cast<U8>(length & 0xFF));
            inner.update(static_cast<U8>(length >> 8));
            inner.update(headerCksum[0]);
            inner.update(headerCksum[1]);
            inner.update(data, dataLen);
            inner.update(dataCksum[0]);
            inner.update(dataCksum[1]);

            U8 innerDigest[Sha1::DIGEST_LENGTH];
            inner.digest(innerDigest);
            const U8 innerShort[4] = {innerDigest[3], innerDigest[2], innerDigest[1], innerDigest[0]};

            Sha1 outer;
            outer.update(opad, sizeof(opad));
            outer.update(innerShort, sizeof(innerShort));

            U8 outerDigest[Sha1::DIGEST_LENGTH];
            outer.digest(outerDigest);
            for (U32 i = 0; i < 4; i++) {
                out[i] = outerDigest[3 - i];  // python: list(reversed(hmac[0:4]))
            }
        }

    } // anonymous namespace (private to this file)

    // mirrors encode_rap() from encode.py, returns total packet length or 0 if 'data' is too long or 'out' is too small
    FwSizeType encode(U8 flag, const U8* data, FwSizeType dataLen, U8* out, FwSizeType outCapacity) {
        const FwSizeType length = HEAD_LENGTH + dataLen + FOOT_LENGTH;
        if (dataLen > MAX_DATA_LENGTH || length > outCapacity) {
            return 0;
        }

        // header - everything little-endian except the secondary id
        out[0] = SYNC_0;
        out[1] = SYNC_1;
        out[2] = static_cast<U8>(PRIMARY_ID & 0xFF);
        out[3] = static_cast<U8>(PRIMARY_ID >> 8);
        out[4] = static_cast<U8>(SECONDARY_ID >> 8);
        out[5] = static_cast<U8>(SECONDARY_ID & 0xFF);
        out[6] = flag;
        out[7] = static_cast<U8>(length & 0xFF);
        out[8] = static_cast<U8>(length >> 8);
        fletcher16(out, HEAD_LENGTH - 2, &out[9]);  // covers bytes 0-8

        memcpy(&out[HEAD_LENGTH], data, dataLen);

        // footer: data checksum then hmac
        U8* foot = &out[HEAD_LENGTH + dataLen];
        fletcher16(data, dataLen, &foot[0]);
        calculateHmac(flag, static_cast<U16>(length), &out[9], data, dataLen, &foot[0], &foot[2]);

        return length;
    }

} // namespace Rap
} // namespace CubesatRef