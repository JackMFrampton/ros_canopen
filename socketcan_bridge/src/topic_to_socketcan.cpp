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
      // Attempts to collect can_device parameter
      // If executed from launch script, parameter is assigned via .yaml file
      // else parameter is set to can0 by default
      auto flag_can_device = get_parameter_or("can_device",
                                              can_device_,
                                              rclcpp::Parameter("can_device", "vcan0"));
      if (!flag_can_device)
      {
          RCLCPP_WARN_ONCE(get_logger(),
                          "Could not get can device, setting: %s",
                          can_device_.as_string().c_str());
      }

      // Attempts to collect json_file parameter
      // If executed from launch script, parameter is assigned via .yaml file
      // else parameter is set to a default file path
      auto flag_json_file = get_parameter_or("json_file",
            json_file_,
            rclcpp::Parameter("json_file", "/home/ros2/foxy/deepx/json_example.json"));
      if (!flag_json_file)
      {
          RCLCPP_WARN_ONCE(get_logger(),
                          "Could not get JSON file, setting: %s",
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

          // Takes first instances of id and name
          // May be a better implementation
          if (ele.contains("id"))
          {
            tmp_id = ele["id"].get<uint32_t>();
          }

          if (ele.contains("name"))
          {
            tmp_topic_str = ele["name"].get<std::string>();
            boost::to_lower(tmp_topic_str);

            tmp_sub = this->create_subscription<can_msgs::msg::Frame>
                            (tmp_topic_str.c_str(), 10,
                            std::bind(&TopicToSocketCAN::msgCallback,
                            this, std::placeholders::_1));
          }

          // Takes first instance of signals
          if (ele.contains("signals"))
          {
            // Some messages have no signals in the json array
            if (ele["signals"].size() > 0)
            {
              // Iterate through each element in the json array
              for (const auto& sig : ele["signals"])
              {
                uint16_t tmp_bit_length;
                double tmp_factor;
                bool tmp_is_big_endian;
                bool tmp_is_signed;
                double tmp_max;
                double tmp_min;
                std::string tmp_name;
                double tmp_offset;
                uint16_t tmp_start_bit;

                // Iterate through the first instance of each relevent variable
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

                // Create a signal object that contains each value
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
          // msg id mapped with a vector of its signals, public for access
          t_to_s_id_signal_map_.emplace(tmp_id, tmp_vector_signals);
          t_to_s_topic_vector_.push_back(tmp_sub);
          t_to_s_id_name_map_.emplace(tmp_id, tmp_topic_str);
        }
      }else{
        RCLCPP_ERROR(this->get_logger(), "JSON File could not be opened\n");
        RCLCPP_ERROR(this->get_logger(), strerror(errno));
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
      // translate it to the socketcan frame type.
      can_msgs::msg::Frame m = *msg.get();  // ROS message
      can::Frame f;  // socketcan type
      if (!m.is_extended)
      {
        f = can::Frame(can::Header(m.id, m.is_extended, m.is_rtr, m.is_error));
      }else{
        f = can::Frame(can::ExtendedHeader(m.id));
      }

      // map iterator to key value pair of matching can id and vector of signals
      auto tmp_signal_iter = t_to_s_id_signal_map_.find(m.id);

      // convert if the can id is valid
      // (according to the json file)
      // this is the subscriber msgCallback which should only recieve valid msgs
      // just as a precaution, maybe not needed
      if (tmp_signal_iter != t_to_s_id_signal_map_.end())
      {
        convertMessageToSocketCAN(m, f, t_to_s_id_signal_map_);
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
