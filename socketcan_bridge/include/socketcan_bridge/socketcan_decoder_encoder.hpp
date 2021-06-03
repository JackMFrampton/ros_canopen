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

#ifndef SOCKETCAN_BRIDGE__SOCKETCAN_DECODER_ENCODER_HPP_
#define SOCKETCAN_BRIDGE__SOCKETCAN_DECODER_ENCODER_HPP_

#include <socketcan_bridge/socketcan_signal.hpp>
#include <can_msgs/msg/frame.hpp>
#include <stdint.h>
#include <vector>

namespace socketcan_bridge
{

    float toPhysicalValue(uint64_t target, float factor, float offset,
                                bool is_signed);

    uint64_t fromPhysicalValue(float physical_value, float factor,
                                    float offset);

    void clearBits(uint8_t* target_byte, uint8_t* bits_to_clear,
                        const uint8_t startbit, const uint8_t length);

    void storeSignal(uint8_t* frame, uint64_t value, const uint8_t startbit,
                            const uint8_t length, bool is_big_endian, bool is_signed);

    uint64_t extractSignal(const uint8_t* frame, const uint8_t startbit,
                                const uint8_t length, bool is_big_endian, bool is_signed);

    float decode(const uint8_t* frame,
                        const socketcan_bridge::SocketCANSignal &signal);

    void encode(uint8_t* frame,
                        const socketcan_bridge::SocketCANSignal &signal);

}  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE__SOCKETCAN_DECODER_ENCODER_HPP_
