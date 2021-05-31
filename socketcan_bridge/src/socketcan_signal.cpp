/*
 * Copyright (c) 2021, DeepX, Inc.
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

#include <socketcan_bridge/socketcan_signal.hpp>
#include <string>

namespace socketcan_bridge
{
    SocketCANSignal::SocketCANSignal(const uint16_t &bit_length,
                                    const float &factor,
                                    const bool &is_big_endian,
                                    const bool &is_signed,
                                    const float &max,
                                    const float &min,
                                    const std::string &signal_name,
                                    const float &offset,
                                    const uint16_t &start_bit,
                                    const std::string &topic_name) :
                                    bit_length_{ bit_length },
                                    factor_{ factor },
                                    is_big_endian_{ is_big_endian },
                                    is_signed_{ is_signed },
                                    max_{ max },
                                    min_{ min },
                                    signal_name_{ signal_name },
                                    offset_{ offset },
                                    start_bit_{ start_bit },
                                    topic_name_{ topic_name }
                                    {}

}  // namespace socketcan_bridge
