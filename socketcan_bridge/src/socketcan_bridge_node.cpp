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
#include <socketcan_interface/threading.hpp>
// #include <socketcan_interface/xmlrpc_settings.hpp>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto driver_s_to_t = std::make_shared<can::ThreadedSocketCANInterface>();
  auto driver_t_to_s = std::make_shared<can::ThreadedSocketCANInterface>();

  auto socketcan_to_topic = std::make_shared<socketcan_bridge::SocketCANToTopic>(driver_s_to_t);
  auto topic_to_socketcan = std::make_shared<socketcan_bridge::TopicToSocketCAN>(driver_t_to_s);

  auto s_to_t_shared_ptr = socketcan_to_topic->shared_from_this();
  auto t_to_s_shared_ptr = topic_to_socketcan->shared_from_this();

  std::string s_to_t_can_device;
  std::string t_to_s_can_device;

  s_to_t_shared_ptr->get_parameter("can_device", s_to_t_can_device);
  t_to_s_shared_ptr->get_parameter("can_device", t_to_s_can_device);

  if (!driver_s_to_t->init(s_to_t_can_device.c_str(), 0, can::NoSettings::create()))
  {
    RCLCPP_FATAL(s_to_t_shared_ptr->get_logger(),
                "Failed to initialize can_device at %s",
                s_to_t_can_device.c_str());
  }else{
    RCLCPP_INFO(s_to_t_shared_ptr->get_logger(),
                "Successfully connected to %s.",
                s_to_t_can_device.c_str());
  }

  if (!driver_t_to_s->init(t_to_s_can_device.c_str(), 0, can::NoSettings::create()))
  {
    RCLCPP_FATAL(t_to_s_shared_ptr->get_logger(),
                "Failed to initialize can_device at %s",
                t_to_s_can_device.c_str());
  }else{
    RCLCPP_INFO(t_to_s_shared_ptr->get_logger(),
                "Successfully connected to %s.",
                t_to_s_can_device.c_str());
  }

  socketcan_to_topic->setup();
  topic_to_socketcan->setup();

  exec.add_node(socketcan_to_topic);
  exec.add_node(topic_to_socketcan);

  exec.spin();

  rclcpp::shutdown();

  driver_s_to_t->shutdown();
  driver_s_to_t.reset();

  driver_t_to_s->shutdown();
  driver_t_to_s.reset();
}
