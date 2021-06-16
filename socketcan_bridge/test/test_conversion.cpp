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
#include <socketcan_bridge/topic_to_socketcan.hpp>
#include <socketcan_bridge/socketcan_to_topic.hpp>
#include <socketcan_bridge/socketcan_signal.hpp>

#include <can_msgs/msg/frame.hpp>
#include <socketcan_interface/socketcan.hpp>

// Bring in gtest
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <boost/algorithm/string.hpp>
#include <stdint.h>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>

using json = nlohmann::json;

class MessageReference
{
  public:
    // constructor, creates a map of msg id with a vector of its signals
    // only one signal needed for testing socketcan_conversion
    MessageReference()
    {
      std::vector<socketcan_bridge::SocketCANSignal> tmp_vector_signals;

      uint32_t tmp_id = 1;
      std::string tmp_topic_str = "test_topic";

      uint16_t tmp_bit_length = 64;
      float tmp_factor = 1.0;
      bool tmp_is_big_endian = false;
      bool tmp_is_signed = false;
      float tmp_max = 255.0;
      float tmp_min = 0.0;
      std::string tmp_name = "test_signal_name";
      float tmp_offset = 0;
      uint16_t tmp_start_bit = 0;

      // Create a signal object to contain each value
      socketcan_bridge::SocketCANSignal tmp_signal(tmp_bit_length,
                                                    tmp_factor,
                                                    tmp_is_big_endian,
                                                    tmp_is_signed,
                                                    tmp_max,
                                                    tmp_min,
                                                    tmp_name,
                                                    tmp_offset,
                                                    tmp_start_bit,
                                                    tmp_topic_str);

      tmp_vector_signals.push_back(tmp_signal);
      id_signal_map_.emplace(tmp_id, tmp_vector_signals);
    }

    // msg id mapped with a vector of its signals, public for access
    // not best practice but beats using global variable
    std::map<int, std::vector<socketcan_bridge::SocketCANSignal>> id_signal_map_;
};

// test whether the content of a conversion from a SocketCAN frame
// to a ROS message correctly maintains the data.
TEST(ConversionTest, socketCANToTopicStandard)
{
  MessageReference ref;
  can::Frame f;
  can_msgs::msg::Frame m;

  // create a dummy can::Frame
  f.id = 1;
  f.dlc = 8;
  f.is_error = false;
  f.is_rtr = false;
  f.is_extended = false;
  for (uint8_t i = 0; i < f.dlc; ++i)
  {
    f.data[i] = 0;
  }
  // compare the created can::Frame, f, to the converted can_msgs::msg::Frame message, m
  socketcan_bridge::convertSocketCANToMessage(f, m, ref.id_signal_map_);
  EXPECT_EQ(1, m.id);
  EXPECT_EQ(8, m.dlc);
  EXPECT_EQ(false, m.is_error);
  EXPECT_EQ(false, m.is_rtr);
  EXPECT_EQ(false, m.is_extended);
  EXPECT_EQ("test_signal_name", m.signal_names[0]);
  EXPECT_EQ(0, m.signal_values[0]);
}

// test all three flags seperately.
TEST(ConversionTest, socketCANToTopicFlags)
{
  MessageReference ref;
  can::Frame f;
  can_msgs::msg::Frame m;

  // create a dummy can::Frame
  f.id = 1;
  f.dlc = 8;
  f.is_error = false;
  f.is_rtr = false;
  f.is_extended = false;
  for (uint8_t i = 0; i < f.dlc; ++i)
  {
    f.data[i] = 0;
  }

  f.is_error = true;
  // compare the created can::Frame, f, to the converted can_msgs::msg::Frame message, m
  socketcan_bridge::convertSocketCANToMessage(f, m, ref.id_signal_map_);
  EXPECT_EQ(true, m.is_error);
  f.is_error = false;

  f.is_rtr = true;
  // compare the created can::Frame, f, to the converted can_msgs::msg::Frame message, m
  socketcan_bridge::convertSocketCANToMessage(f, m, ref.id_signal_map_);
  EXPECT_EQ(true, m.is_rtr);
  f.is_rtr = false;

  f.is_extended = true;
  // compare the created can::Frame, f, to the converted can_msgs::msg::Frame message, m
  socketcan_bridge::convertSocketCANToMessage(f, m, ref.id_signal_map_);
  EXPECT_EQ(true, m.is_extended);
  f.is_extended = false;
}

// idem, but the other way around.
TEST(ConversionTest, topicToSocketCANStandard)
{
  MessageReference ref;
  can::Frame f;
  can_msgs::msg::Frame m;

  // create a dummy can_msgs::msg::Frame
  m.id = 1;
  m.dlc = 8;
  m.is_error = false;
  m.is_rtr = false;
  m.is_extended = false;
  m.signal_names.push_back("test_signal_name");
  m.signal_values.push_back(0);

  // compare the created can_msgs::msg::Frame message, m, to the converted can::Frame, f
  socketcan_bridge::convertMessageToSocketCAN(m, f, ref.id_signal_map_);
  EXPECT_EQ(1, f.id);
  EXPECT_EQ(8, f.dlc);
  EXPECT_EQ(false, f.is_error);
  EXPECT_EQ(false, f.is_rtr);
  EXPECT_EQ(false, f.is_extended);

  for (uint8_t i=0; i < 8; i++)
  {
    EXPECT_EQ(0, f.data[i]);
  }
}

TEST(ConversionTest, topicToSocketCANFlags)
{
  MessageReference ref;
  can::Frame f;
  can_msgs::msg::Frame m;

  // create a dummy can_msgs::msg::Frame
  m.id = 1;
  m.dlc = 8;
  m.is_error = false;
  m.is_rtr = false;
  m.is_extended = true;
  m.signal_names.push_back("test_signal_name");
  m.signal_values.push_back(0);

  m.is_error = true;
  // compare the created can_msgs::msg::Frame message, m, to the converted can::Frame, f
  socketcan_bridge::convertMessageToSocketCAN(m, f, ref.id_signal_map_);
  EXPECT_EQ(true, f.is_error);
  m.is_error = false;

  m.is_rtr = true;
  // compare the created can_msgs::msg::Frame message, m, to the converted can::Frame, f
  socketcan_bridge::convertMessageToSocketCAN(m, f, ref.id_signal_map_);
  EXPECT_EQ(true, f.is_rtr);
  m.is_rtr = false;

  m.is_extended = true;
  // compare the created can_msgs::msg::Frame message, m, to the converted can::Frame, f
  socketcan_bridge::convertMessageToSocketCAN(m, f, ref.id_signal_map_);
  EXPECT_EQ(true, f.is_extended);
  m.is_extended = false;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
