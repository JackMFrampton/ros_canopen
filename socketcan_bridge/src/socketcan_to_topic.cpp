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
#include <socketcan_interface/string.hpp>
#include <can_msgs/msg/frame.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <map>

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
      auto flag_can_device = get_parameter_or("can_device",
                                              can_device_,
                                              rclcpp::Parameter("can_device", "can0"));
      if (!flag_can_device)
      {
          RCLCPP_WARN_ONCE(get_logger(),
                          "Could not get can device, setting: %s",
                          can_device_.as_string().c_str());
      }

      std::ifstream jsonFile("/home/ros2/foxy/deepx/json_example.json");
      if (jsonFile)
      {
        json j = json::parse(jsonFile);
        if (j.is_discarded())
        {
          std::cerr << "JSON File could not be parsed\n";
          std::cerr << "Error code: " << strerror(errno);
        }

        // Iterates through the "messages" array in the json file
        for (const auto& ele : j["messages"])
        {
          int tmp_id;
          rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr tmp_pub;

          // Takes first instances of id and name
          // May be a better implementation
          if (ele.contains("id"))
          {
            tmp_id = ele["id"].get<int>();
          }
          if (ele.contains("name"))
          {
            tmp_pub = this->create_publisher<can_msgs::msg::Frame>
                                            (ele["name"].get<std::string>().c_str(), 10);
          }

          s_to_t_id_map_.emplace(tmp_id, tmp_pub);
        }
      }else{
        std::cerr << "JSON File could not be opened\n";
        std::cerr << "Error code: " << strerror(errno);
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
      // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
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

      std::map<int, rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr>::iterator tmp_iter;
      tmp_iter = s_to_t_id_map_.find(f.id);

      if (tmp_iter != s_to_t_id_map_.end())
      {
         RCLCPP_INFO(this->get_logger(),
                    "Message published to ID %i",
                    f.id);
        can_msgs::msg::Frame msg;
        // converts the can::Frame (socketcan.h) to can_msgs::Frame (ROS msg)
        convertSocketCANToMessage(f, msg);

        msg.header.frame_id = "";  // empty frame is the de-facto standard for no frame.
        msg.header.stamp = this->get_clock()->now();

        tmp_iter->second->publish(msg);
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
