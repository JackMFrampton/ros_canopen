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

#include <socketcan_bridge/socketcan_converter.hpp>
#include <socketcan_bridge/socketcan_decoder_encoder.hpp>
#include <stdint.h>
#include <map>
#include <vector>
#include <algorithm>

namespace socketcan_bridge
{

  void convertSocketCANToMessage(const can::Frame& f, can_msgs::msg::Frame& m,
      std::map<int, std::vector<socketcan_bridge::SocketCANSignal>>& map)
  {
    m.id = f.id;
    m.dlc = f.dlc;
    m.is_error = f.is_error;
    m.is_rtr = f.is_rtr;
    m.is_extended = f.is_extended;

    // map iterator to key value pair of matching can id and vector of signals
    auto tmp_signal_iter = map.find(f.id);

    // the message associated with the can id may not have any signals
    // proceed only if there are signals to process
    if (tmp_signal_iter != map.end())
    {
      if (tmp_signal_iter->second.size() > 0)
      {
        std::array<uint8_t, 8> data;
        std::copy(begin(f.data), end(f.data), begin(data));

        // iterate through each signal in the message
        // decode each signal and push them to the ros msg arrays
        for (auto &signal : tmp_signal_iter->second)
        {
          // break if the signal has no bit length
          // a CAN msg that spans the full 64 bits
          // will have a dummy signal with 0 bit length
          // some dummy signals will have a bit length
          // this is so that the original data is preserved
          if (signal.bit_length_ == 0)
          {
            signal.value_ = 0.0;
            m.signal_names.push_back(signal.signal_name_);
            m.signal_values.push_back(signal.value_);
            break;
          }

          signal.value_ = decode(data.data(), signal);

          m.signal_names.push_back(signal.signal_name_);
          m.signal_values.push_back(signal.value_);
        }
      }
    }
  }

  void convertMessageToSocketCAN(const can_msgs::msg::Frame& m, can::Frame& f,
      std::map<int, std::vector<socketcan_bridge::SocketCANSignal>>& map)
  {
    f.id = m.id;
    f.dlc = m.dlc;
    f.is_error = m.is_error;
    f.is_rtr = m.is_rtr;
    f.is_extended = m.is_extended;

    auto tmp_signal_names = m.signal_names;
    auto tmp_signal_values = m.signal_values;
    // map iterator to key value pair of matching can id and vector of signals
    auto tmp_signal_iter = map.find(m.id);

    // can id may be valid
    // but the message may not have any signals
    if (tmp_signal_iter != map.end())
    {
      if (tmp_signal_iter->second.size() > 0)
      {
        // iterate through each signal in the message
        for (auto &signal : tmp_signal_iter->second)
        {
          // break if the signal has no bit length
          // a CAN msg that spans the full 64 bits
          // will have a dummy signal with 0 bit length
          if (signal.bit_length_ == 0)
          {
            break;
          }

          // loop through the ros msg vectors until it matches current signal
          for (size_t i = 0; i < tmp_signal_names.size(); i++)
          {
            // once a match is found:
            // assign signal value
            // erase here so each subsequent iteration is more efficient
            // break here to prevent reiteration
            if (tmp_signal_names[i] == signal.signal_name_)
            {
              signal.value_ = tmp_signal_values[i];

              tmp_signal_names.erase(tmp_signal_names.begin()+i);
              tmp_signal_values.erase(tmp_signal_values.begin()+i);

              break;
            }
          }
          // signals fed into the encode function slowly build up data[8] array
          encode(f.data.data(), signal);
        }
      }
    }
  }

}  // namespace socketcan_bridge
