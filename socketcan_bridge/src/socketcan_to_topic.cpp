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

#include <socketcan_bridge/socketcan_to_topic.hpp>
#include <socketcan_bridge/socketcan_signal.hpp>
#include <socketcan_bridge/socketcan_decoder_encoder.hpp>
#include <socketcan_interface/string.hpp>
#include <can_msgs/msg/frame.hpp>
#include <nlohmann/json.hpp>
#include <boost/algorithm/string.hpp>
#include <stdint.h>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <algorithm>

using json = nlohmann::json;

// namespace can
// {
// template<> can::FrameFilterSharedPtr tofilter(const XmlRpc::XmlRpcValue  &ct)
// {
//   XmlRpc::XmlRpcValue t(ct);
//   try  // try to read as integer
//   {
//     uint32_t id = static_cast<int>(t);
//     return tofilter(id);
//   }
//   catch(...)  // else read as string
//   {
//     return  tofilter(static_cast<std::string>(t));
//   }
// }
// }  // namespace can

namespace socketcan_bridge
{
  SocketCANToTopic::SocketCANToTopic(can::DriverInterfaceSharedPtr driver)
                                    : Node("socketcan_to_topic_node",
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
          rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr tmp_pub;
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

            tmp_pub = this->create_publisher<can_msgs::msg::Frame>
                                (tmp_topic_str.c_str(), 10);
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
          // dummy signal needed for every CAN msg
          // this will help preserve data
          if (!tmp_vector_signals.empty())
          {
            auto end_signal = tmp_vector_signals.back();

            double dummy_tmp_factor = 1.0;
            bool dummy_tmp_is_big_endian = end_signal.is_big_endian_;
            bool dummy_tmp_is_signed = false;
            std::string dummy_tmp_name = "dummy_signal";
            double dummy_tmp_offset = 0.0;
            uint16_t dummy_tmp_start_bit = end_signal.start_bit_
                                            + end_signal.bit_length_;
            uint16_t dummy_tmp_bit_length = 64 - dummy_tmp_start_bit;
            std::string dummy_tmp_topic_str = tmp_topic_str;
            double dummy_tmp_max;
            double dummy_tmp_min;
            if (dummy_tmp_bit_length == 0)
            {
              dummy_tmp_max = 1;
              dummy_tmp_min = -dummy_tmp_max;
            }else{
              dummy_tmp_max = (1 << uint64_t(dummy_tmp_bit_length)) - 1;
              dummy_tmp_min = -dummy_tmp_max;
            }

            socketcan_bridge::SocketCANSignal dummy_tmp_signal(
                                              dummy_tmp_bit_length,
                                              dummy_tmp_factor,
                                              dummy_tmp_is_big_endian,
                                              dummy_tmp_is_signed,
                                              dummy_tmp_max,
                                              dummy_tmp_min,
                                              dummy_tmp_name,
                                              dummy_tmp_offset,
                                              dummy_tmp_start_bit,
                                              dummy_tmp_topic_str);

            tmp_vector_signals.push_back(dummy_tmp_signal);
          }else{
            // This will be for edge cases where the CAN msg has no signals
            // Again, this is for preservation of data
            double dummy_tmp_factor = 1.0;
            bool dummy_tmp_is_big_endian = false;
            bool dummy_tmp_is_signed = false;
            std::string dummy_tmp_name = "dummy_signal";
            double dummy_tmp_offset = 0;
            uint16_t dummy_tmp_start_bit = 0;
            uint16_t dummy_tmp_bit_length = 64;
            std::string dummy_tmp_topic_str = tmp_topic_str;
            double dummy_tmp_max = (1 << uint64_t(dummy_tmp_bit_length)) - 1;
            double dummy_tmp_min = -dummy_tmp_max;

            socketcan_bridge::SocketCANSignal dummy_tmp_signal(
                                              dummy_tmp_bit_length,
                                              dummy_tmp_factor,
                                              dummy_tmp_is_big_endian,
                                              dummy_tmp_is_signed,
                                              dummy_tmp_max,
                                              dummy_tmp_min,
                                              dummy_tmp_name,
                                              dummy_tmp_offset,
                                              dummy_tmp_start_bit,
                                              dummy_tmp_topic_str);

            tmp_vector_signals.push_back(dummy_tmp_signal);
          }
          // msg id mapped with a vector of its signals, public for access
          s_to_t_id_signal_map_.emplace(tmp_id, tmp_vector_signals);
          s_to_t_id_pub_map_.emplace(tmp_id, tmp_pub);
          s_to_t_id_name_map_.emplace(tmp_id, tmp_topic_str);
        }
      }else{
        RCLCPP_ERROR(this->get_logger(),
                    "JSON File could not be opened\n");
        RCLCPP_ERROR(this->get_logger(),
                    strerror(errno));
      }

      driver_ = driver;
    }

  void SocketCANToTopic::setup()
    {
      // register handler for frames and state changes.
      frame_listener_ = driver_->createMsgListenerM(this, &SocketCANToTopic::frameCallback);
      state_listener_ = driver_->createStateListenerM(this, &SocketCANToTopic::stateCallback);
    }

  void SocketCANToTopic::setup(const can::FilteredFrameListener::FilterVector &filters)
  {
    frame_listener_.reset(new can::FilteredFrameListener(driver_,
                                                        std::bind(&SocketCANToTopic::frameCallback,
                                                        this,
                                                        std::placeholders::_1),
                                                        filters));

    state_listener_ = driver_->createStateListenerM(this, &SocketCANToTopic::stateCallback);
  }

  // void SocketCANToTopic::setup(XmlRpc::XmlRpcValue filters)
  // {
  //     setup(can::tofilters(filters));
  // }
  // void SocketCANToTopic::setup(std::shared_ptr<rclcpp::Node> nh)
  // {
  //      XmlRpc::XmlRpcValue filters;
  //      if (nh->get_parameter("can_ids", filters)) return setup(filters);
  //      return setup();
  // }

  void SocketCANToTopic::frameCallback(const can::Frame& f)
    {
      RCLCPP_INFO(this->get_logger(), "Frame recieved");
      if (!f.isValid())
      {
        std::string errStr  = "Invalid frame from SocketCAN: ";
        std::string errStr2 = "id: %#04x, length: %d, is_extended: %d, is_error: %d, is_rtr: %d";
        RCLCPP_ERROR(this->get_logger(),
        errStr.append(errStr2),
        f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
        return;
      }else{
        if (f.is_error)
        {
          // can::tostring cannot be used for dlc > 8 frames. It causes an crash
          // due to usage of boost::array for the data array. The should always work.
          RCLCPP_WARN(this->get_logger(),
                      "Received frame is error: %s",
                      can::tostring(f, true).c_str());
        }
      }

      // map iterator to key value pair of matching can id and publisher object
      auto tmp_pub_iter = s_to_t_id_pub_map_.find(f.id);

      // convert and publish if the can id is valid
      // (according to the json file)
      if (tmp_pub_iter != s_to_t_id_pub_map_.end())
      {
        can_msgs::msg::Frame msg;
        convertSocketCANToMessage(f, msg, s_to_t_id_signal_map_);

        auto tmp_name_iter = s_to_t_id_name_map_.find(f.id);
        if (tmp_name_iter != s_to_t_id_name_map_.end())
        {
          msg.name = tmp_name_iter->second;
        }
        msg.header.frame_id = "";
        msg.header.stamp = this->get_clock()->now();

        // publish the ros msg to the topic matching the can id
        tmp_pub_iter->second->publish(msg);
      }
    }


  void SocketCANToTopic::stateCallback(const can::State & s)
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
