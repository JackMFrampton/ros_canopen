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

  auto node_name = std::string("socketcan_bridge_node");

  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  // Type = std::shared_ptr<rclcpp::Node>
  auto socketcan_bridge_driver = std::make_shared<socketcan_bridge_driver::SocketCANDriver>(node_name, options);
  auto driver_node_shared_ptr = socketcan_bridge_driver->shared_from_this();
  socketcan_bridge_driver->init_param();
  socketcan_bridge_driver->init_can_driver();

  // initialize the bridge both ways.
  socketcan_bridge_driver->init_topic_to_socket_can(driver_node_shared_ptr);
  socketcan_bridge_driver->init_socket_can_to_topic(driver_node_shared_ptr);

  rclcpp::spin(socketcan_bridge_driver);
  rclcpp::shutdown();
}
