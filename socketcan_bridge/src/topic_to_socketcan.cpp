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
#include <socketcan_bridge/socketcan_signal.hpp>
#include <socketcan_bridge/socketcan_decoder_encoder.hpp>
#include <socketcan_interface/string.hpp>
#include <nlohmann/json.hpp>
#include <boost/algorithm/string.hpp>
#include <stdint.h>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>

using json = nlohmann::json;

namespace socketcan_bridge
{
  TopicToSocketCAN::TopicToSocketCAN(can::DriverInterfaceSharedPtr driver)
                                    : Node("topic_to_socketcan_node",
                                    rclcpp::NodeOptions()
                                    .allow_undeclared_parameters(true)
                                    .automatically_declare_parameters_from_overrides(true))
    {
      auto flag_can_device = get_parameter_or("can_device",
                                              can_device_,
                                              rclcpp::Parameter("can_device", "can0"));
      if (!flag_can_device)
      {
          RCLCPP_WARN_ONCE(get_logger(),
                          "Could not get can device, setting: %s",
                          can_device_.as_string().c_str());
      }

      auto flag_json_file = get_parameter_or("json_file",
            json_file_,
            rclcpp::Parameter("json_file", "/home/ros2/foxy/deepx/json_example.json"));
      if (!flag_json_file)
      {
          RCLCPP_WARN_ONCE(get_logger(),
                          "Could not get JSON file: %s",
                          json_file_.as_string().c_str());
      }

      std::ifstream jsonFile(json_file_.as_string().c_str());
      if (jsonFile)
      {
        json j = json::parse(jsonFile);
        if (j.is_discarded())
        {
          RCLCPP_ERROR(this->get_logger(),
                      "JSON File could not be parsed\n");
          RCLCPP_ERROR(this->get_logger(),
                      strerror(errno));
        }

        // Iterates through the "messages" array in the json file
        for (const auto& ele : j["messages"])
        {
          uint32_t tmp_id;
          rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr tmp_sub;
          std::vector<socketcan_bridge::SocketCANSignal> tmp_vector_signals;
          std::string tmp_topic_str;

          if (ele.contains("id"))
          {
            tmp_id = ele["id"].get<uint32_t>();
          }
          if (ele.contains("name"))
          {
            std::string tmp_topic_str = ele["name"].get<std::string>();
            boost::to_lower(tmp_topic_str);

            tmp_sub = this->create_subscription<can_msgs::msg::Frame>
                            (tmp_topic_str.c_str(), 10,
                            std::bind(&TopicToSocketCAN::msgCallback,
                            this, std::placeholders::_1));
          }
          if (ele.contains("signals"))
          {
            if (ele["signals"].size() > 0)
            {
              for (const auto& sig : ele["signals"])
              {
                uint16_t tmp_bit_length;
                float tmp_factor;
                bool tmp_is_big_endian;
                bool tmp_is_signed;
                float tmp_max;
                float tmp_min;
                std::string tmp_name;
                float tmp_offset;
                uint16_t tmp_start_bit;

                if (sig.contains("bit_length"))
                {
                  tmp_bit_length = sig["bit_length"].get<uint16_t>();
                }
                if (sig.contains("factor"))
                {
                  tmp_factor = std::stof(sig["factor"].get<std::string>());
                }
                if (sig.contains("is_big_endian"))
                {
                  tmp_is_big_endian = sig["is_big_endian"].get<bool>();
                }
                if (sig.contains("is_signed"))
                {
                  tmp_is_signed = sig["is_signed"].get<bool>();
                }
                if (sig.contains("max"))
                {
                  tmp_max = std::stof(sig["max"].get<std::string>());
                }
                if (sig.contains("min"))
                {
                  tmp_min = std::stof(sig["min"].get<std::string>());
                }
                if (sig.contains("name"))
                {
                  tmp_name = sig["name"].get<std::string>();
                  boost::to_lower(tmp_name);
                }
                if (sig.contains("offset"))
                {
                  tmp_offset = std::stof(sig["offset"].get<std::string>());
                }
                if (sig.contains("start_bit"))
                {
                  tmp_start_bit = sig["start_bit"].get<uint16_t>();
                }

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
              }
            }
          }

          t_to_s_id_signal_map_.emplace(tmp_id, tmp_vector_signals);
          t_to_s_topic_vector_.push_back(tmp_sub);
        }
      }else{
        RCLCPP_ERROR(this->get_logger(),
                    "JSON File could not be opened\n");
        RCLCPP_ERROR(this->get_logger(),
                    strerror(errno));
      }

      driver_ = driver;
    }

  void TopicToSocketCAN::setup()
    {
      state_listener_ = driver_->createStateListener(
              std::bind(&TopicToSocketCAN::stateCallback, this, std::placeholders::_1));
    }

  void TopicToSocketCAN::msgCallback(const can_msgs::msg::Frame::SharedPtr msg)
    {
      // ROS_DEBUG("Message came from sent_messages topic");

      // translate it to the socketcan frame type.

      can_msgs::msg::Frame m = *msg.get();  // ROS message
      can::Frame f;  // socketcan type

      auto tmp_signal_names = m.signal_names;
      auto tmp_signal_values = m.signal_values;
      auto tmp_signal_iter = t_to_s_id_signal_map_.find(m.id);

      if (tmp_signal_iter != t_to_s_id_signal_map_.end())
      {
        // converts the can_msgs::Frame (ROS msg) to can::Frame (socketcan.h)
        convertMessageToSocketCAN(m, f);

        if (tmp_signal_iter->second.size() > 0)
        {
          std::array<uint8_t, 8> data;

          for (auto &signal : tmp_signal_iter->second)
          {
            for (size_t i = 0; i < tmp_signal_names.size(); ++i)
            {
              if (tmp_signal_names[i] == signal.signal_name_)
              {
                signal.value_ = tmp_signal_values[i];

                tmp_signal_names.erase(tmp_signal_names.begin()+i);
                tmp_signal_values.erase(tmp_signal_values.begin()+i);

                break;
              }
            }
            encode(data.data(), signal);
          }

          std::copy(begin(data), end(data), begin(f.data));
        }
      }

      if (!f.isValid())  // check if the id and flags are appropriate.
      {
        // ROS_WARN("Refusing to send invalid frame: %s.", can::tostring(f, true).c_str());
        // can::tostring cannot be used for dlc > 8 frames. It causes an crash
        // due to usage of boost::array for the data array. The should always work.
        RCLCPP_ERROR(this->get_logger(),
                    "Invalid frame from topic: id: %#04x, length: %d, is_extended: %d",
                    m.id, m.dlc, m.is_extended);
        return;
      }

      bool res = driver_->send(f);
      if (!res)
      {
        RCLCPP_ERROR(this->get_logger(),
                    "Failed to send message: %s.",
                    can::tostring(f, true).c_str());
      }
    }



  void TopicToSocketCAN::stateCallback(const can::State & s)
    {
      std::string err;
      driver_->translateError(s.internal_error, err);
      if (!s.internal_error)
      {
        RCLCPP_INFO(this->get_logger(),
                    "State: %s, asio: %s",
                    err.c_str(),
                    s.error_code.message().c_str());
      }else{
        RCLCPP_ERROR(this->get_logger(),
                    "Error: %s, asio: %s",
                    err.c_str(),
                    s.error_code.message().c_str());
      }
    }
}  // namespace socketcan_bridge
