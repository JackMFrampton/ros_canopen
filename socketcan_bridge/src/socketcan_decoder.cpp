/*
 * Copyright (c) 2021, DeepX, Inc.
 * Adapted from: https://github.com/reinzor/libcan-encode-decode
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <socketcan_bridge/socketcan_decoder.hpp>
#include <stdint.h>  // uint typedefinitions, non-rtw!
#include <assert.h>
#include <math.h>

#define MASK64(nbits) ((0xffffffffffffffff) >> (64 - nbits))

namespace socketcan_bridge
{


    float toPhysicalValue(uint64_t target, float factor, float offset, bool is_signed)
    {
        if (is_signed)
        {
            return ((int64_t)target) * factor + offset;
        }else{
            return target * factor + offset;
        }
    }

    uint64_t fromPhysicalValue(float physical_value, float factor, float offset)
    {
        return (int64_t)((physical_value - offset) / factor);
    }

    void clearBits(uint8_t* target_byte, uint8_t* bits_to_clear,
                        const uint8_t startbit, const uint8_t length)
    {
        for (uint8_t i = startbit; i < length + startbit; ++i)
        {
            *target_byte &= ~(1UL << i);
            *bits_to_clear -= 1;
        }
    }

    void storeSignal(uint8_t* frame, uint64_t value, const uint8_t startbit,
                            const uint8_t length, bool is_big_endian)
    {
        uint8_t start_byte = startbit / 8;
        uint8_t startbit_in_byte = startbit % 8;
        uint8_t end_byte = 0;
        int8_t count = 0;
        uint8_t current_target_length = (8 - startbit_in_byte);
        uint8_t bits_to_clear = length;

        // Mask the value
        value &= MASK64(length);

        // Write bits of startbyte
        clearBits(&frame[start_byte], &bits_to_clear, startbit_in_byte,
                current_target_length > length ? length : current_target_length);
        frame[start_byte] |= value << startbit_in_byte;

        // Write residual bytes
        if (is_big_endian)  // Motorola (big endian)
        {
            end_byte = (start_byte * 8 + 8 - startbit_in_byte - length) / 8;

            for (count = start_byte - 1; count >= end_byte; count--)
            {
            clearBits(&frame[count], &bits_to_clear, 0, bits_to_clear >= 8 ? 8 : bits_to_clear);
            frame[count] |= value >> current_target_length;
            current_target_length += 8;
            }
        }else{  // Intel (little endian)
            end_byte = (startbit + length - 1) / 8;

            for (count = start_byte + 1; count <= end_byte; count++)
            {
                clearBits(&frame[count], &bits_to_clear, 0, bits_to_clear >= 8 ? 8 : bits_to_clear);
                frame[count] |= value >> current_target_length;
                current_target_length += 8;
            }
        }
    }

    uint64_t extractSignal(const uint8_t* frame, const uint8_t startbit,
                                const uint8_t length, bool is_big_endian, bool is_signed)
    {
        uint8_t start_byte = startbit / 8;
        uint8_t startbit_in_byte = startbit % 8;
        uint8_t current_target_length = (8 - startbit_in_byte);
        uint8_t end_byte = 0;
        int8_t count = 0;

        // Write first bits to target
        uint64_t target = frame[start_byte] >> startbit_in_byte;

        // Write residual bytes
        if (is_big_endian)  // Motorola (big endian)
        {
            end_byte = (start_byte * 8 + 8 - startbit_in_byte - length) / 8;

            for (count = start_byte - 1; count >= end_byte; count--)
            {
                target |= frame[count] << current_target_length;
                current_target_length += 8;
            }
        }else{  // Intel (little endian)
            end_byte = (startbit + length - 1) / 8;

            for (count = start_byte + 1; count <= end_byte; count++)
            {
                target |= frame[count] << current_target_length;
                current_target_length += 8;
            }
        }

        // Mask value
        target &= MASK64(length);

        // perform sign extension
        if (is_signed)
        {
            int64_t msb_sign_mask = 1 << (length - 1);
            target = ((int64_t)target ^ msb_sign_mask) - msb_sign_mask;
        }

        return target;
    }

    // For Vector CAN DB files https://vector.com/vi_candb_en.html

    float decode(const uint8_t* frame, const socketcan_bridge::SocketCANSignal &signal)
    {
    return socketcan_bridge::toPhysicalValue(socketcan_bridge::extractSignal(frame,
                                            signal.start_bit_, signal.bit_length_,
                                            signal.is_big_endian_, signal.is_signed_),
                                            signal.factor_, signal.offset_, signal.is_signed_);
    }

    void encode(uint8_t* frame, const socketcan_bridge::SocketCANSignal &signal)
    {
    socketcan_bridge::storeSignal(frame, socketcan_bridge::fromPhysicalValue(signal.value_,
                                signal.factor_, signal.offset_),
                                signal.start_bit_, signal.bit_length_, signal.is_big_endian_);
    }

}  // namespace socketcan_bridge
