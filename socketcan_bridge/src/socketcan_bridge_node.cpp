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

#include "rclcpp/rclcpp.hpp"
// #include <socketcan_bridge/topic_to_socketcan.hpp>
// #include <socketcan_bridge/socketcan_to_topic.hpp>
#include <socketcan_bridge/socketcan_bridge_driver.hpp>
#include <socketcan_interface/threading.hpp>
// #include <socketcan_interface/xmlrpc_settings.hpp>
#include <memory>
#include <string>


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executers::SingleThreadedExecuter exec;

  driver = std::make_shared<can::ThreadedSocketCANInterface>();

  auto socketcan_to_topic = std::make_shared<socketcan_bridge::SocketCANToTopic>(driver);
  auto topic_to_socketcan = std::make_shared<socketcan_bridge::TopicToSocketCAN>(driver);
  auto driver_node_shared_ptr = socketcan_to_topic->shared_from_this();

  std::string can_device;
  driver_node_shared_ptr->get_parameter("can_device", can_device);

  if (!driver->init(can_device.as_string(), 0, can::NoSettings::create()))
  {
    RCLCPP_FATAL(driver_node_shared_ptr->get_logger(), "Failed to initialize can_device at %s", can_device.as_string().c_str());
  }
  else
  {
    RCLCPP_INFO(driver_node_shared_ptr->get_logger(), "Successfully connected to %s.", can_device.as_string().c_str());
  }

  socketcan_to_topic->setup();
  topic_to_socketcan->setup();

  exec.add_node(socketcan_to_topic);
  exec.add_node(topic_to_socketcan);

  exec.spin();

  rclcpp::shutdown();

  driver->shutdown();
  driver.reset();
}
