/*
 * Copyright (c) 2016, Ivor Wanders
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

#ifndef SOCKETCAN_BRIDGE__SOCKETCAN_CONVERTER_HPP_
#define SOCKETCAN_BRIDGE__SOCKETCAN_CONVERTER_HPP_

#include <socketcan_interface/socketcan.hpp>
#include <socketcan_bridge/socketcan_signal.hpp>
#include <can_msgs/msg/frame.hpp>
#include <map>
#include <vector>

namespace socketcan_bridge
{

void convertSocketCANToMessage(const can::Frame& f, can_msgs::msg::Frame& m,
    std::map<int, std::vector<socketcan_bridge::SocketCANSignal>>& map);

void convertMessageToSocketCAN(const can_msgs::msg::Frame& m, can::Frame& f,
    std::map<int, std::vector<socketcan_bridge::SocketCANSignal>>& map);

}  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE__SOCKETCAN_CONVERTER_HPP_
