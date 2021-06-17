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

#include <can_msgs/msg/frame.hpp>
#include <socketcan_interface/socketcan.hpp>
#include <socketcan_interface/dummy.hpp>
#include <socketcan_bridge/topic_to_socketcan.hpp>

#include <gtest/gtest.h>
#include <list>
#include <memory>
#include <string>
#include <chrono>
#include <map>
#include <vector>
#include "rclcpp/rclcpp.hpp"

// Need to create a separate ROS2 node just to capture incoming msgs in its callback
class GTestSubscriber : public rclcpp::Node
{
  public:
    explicit GTestSubscriber(const std::string &topic_name)
    : Node("gtest_subscriber")
    {
      subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
      topic_name.c_str(), 10, std::bind(&GTestSubscriber::topic_callback,
                                      this, std::placeholders::_1));
    }

    std::list<can_msgs::msg::Frame> messages;

  private:
    void topic_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "ros msg recieved");
      can_msgs::msg::Frame m = *msg.get();
      messages.push_back(m);
    }
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscriber_;
};

std::string convertMessageToString(const can_msgs::msg::Frame &msg,
std::map<int, std::vector<socketcan_bridge::SocketCANSignal>>& map, bool lc = true)
{
  can::Frame f;
  socketcan_bridge::convertMessageToSocketCAN(msg, f, map);
  return can::tostring(f, lc);
}

TEST(SocketCANToTopicTest, checkCorrectData)
{
  // rclcpp::executors::SingleThreadedExecutor exec;
  can::DummyBus bus("checkCorrectData");

  // create the dummy interface
  auto dummy = std::make_shared<can::ThreadedDummyInterface>();

  // start the to topic bridge.
  auto socketcan_to_topic = std::make_shared<socketcan_bridge::SocketCANToTopic>(dummy);
  socketcan_to_topic->setup();  // initiate the message callbacks
  auto signal_map = socketcan_to_topic->s_to_t_id_signal_map_;
  auto name_map = socketcan_to_topic->s_to_t_id_name_map_;

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // id is an explicitly specified valid id
  // will implement method so that gtest will cycle
  // through all valid ids created from the json
  int id = 835;
  auto iter = signal_map.find(id);
  auto iter2 = name_map.find(id);
  // register for messages
  std::string topic_name = iter2->second;
  auto subscriber = std::make_shared<GTestSubscriber>(topic_name);

  // create a can::frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = iter->first;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = 0;
  }

  // send the can frame to the driver
  dummy->send(f);

  // give some time for the interface some time to process the message
  rclcpp::Rate sleepRate(std::chrono::seconds(1));
  sleepRate.sleep();

  // exec.add_node(socketcan_to_topic);
  // exec.add_node(subscriber);
  // exec.spin_some();

  rclcpp::spin_some(subscriber);

  ASSERT_EQ(1, subscriber->messages.size());

  // compare the received can_msgs::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::msg::Frame msg = subscriber->messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received, signal_map);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  EXPECT_EQ(received.data, f.data);

  dummy->shutdown();
  dummy.reset();
}

TEST(SocketCANToTopicTest, checkInvalidFrameHandling)
{
  // - tries to send a non-extended frame with an id larger than 11 bits.
  //   that should not be sent.
  // - verifies that sending one larger than 11 bits actually works.
  // sending a message with a dlc > 8 is not possible as the DummyInterface
  // causes a crash then.

  // rclcpp::executors::SingleThreadedExecutor exec;
  can::DummyBus bus("checkInvalidFrameHandling");

  // create the dummy interface
  auto dummy = std::make_shared<can::ThreadedDummyInterface>();

  // start the to topic bridge.
  auto socketcan_to_topic = std::make_shared<socketcan_bridge::SocketCANToTopic>(dummy);
  socketcan_to_topic->setup();  // initiate the message callbacks
  auto signal_map = socketcan_to_topic->s_to_t_id_signal_map_;
  auto name_map = socketcan_to_topic->s_to_t_id_name_map_;

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // id is an explicitly specified valid id
  // will implement method so that gtest will cycle
  // through all valid ids created from the json
  int id = 835;
  auto iter = signal_map.find(id);
  auto iter2 = name_map.find(id);
  // register for messages
  std::string topic_name = iter2->second;
  auto subscriber = std::make_shared<GTestSubscriber>(topic_name);

  // create a message
  can::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.is_error = false;
  f.id = (1<<11)+1;  // this is an illegal CAN packet... should not be sent.
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can::Frame over the driver.
  // dummy->send(f);

  // give some time for the interface some time to process the message
  rclcpp::Rate sleepRate(std::chrono::seconds(1));
  sleepRate.sleep();

  // exec.add_node(socketcan_to_topic);
  // exec.add_node(subscriber);
  // exec.spin_some();

  rclcpp::spin_some(subscriber);

  EXPECT_EQ(subscriber->messages.size(), 0);

  f.is_extended = true;
  f.id = iter->first;  // now a valid id

  dummy->send(f);
  sleepRate.sleep();
  rclcpp::spin_some(subscriber);
  EXPECT_EQ(subscriber->messages.size(), 1);

  dummy->shutdown();
  dummy.reset();
}

TEST(SocketCANToTopicTest, checkCorrectCanIdFilter)
{
  // rclcpp::executors::SingleThreadedExecutor exec;
  can::DummyBus bus("checkCorrectCanIdFilter");

  // create the dummy interface
  auto dummy = std::make_shared<can::ThreadedDummyInterface>();

  // create can_id vector with id that should be passed and published to ros
  std::vector<unsigned int> pass_can_ids;
  pass_can_ids.push_back(0x343);

  // start the to topic bridge.
  auto socketcan_to_topic = std::make_shared<socketcan_bridge::SocketCANToTopic>(dummy);
  socketcan_to_topic->setup(can::tofilters(pass_can_ids));  // initiate the message callbacks
  auto signal_map = socketcan_to_topic->s_to_t_id_signal_map_;
  auto name_map = socketcan_to_topic->s_to_t_id_name_map_;

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // id is an explicitly specified valid id
  // will implement method so that gtest will cycle
  // through all valid ids created from the json
  int id = 835;
  auto iter = signal_map.find(id);
  auto iter2 = name_map.find(id);
  // register for messages
  std::string topic_name = iter2->second;
  auto subscriber = std::make_shared<GTestSubscriber>(topic_name);

  // create a can::frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = iter->first;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  dummy->send(f);

  // give some time for the interface some time to process the message
  rclcpp::Rate sleepRate(std::chrono::seconds(1));
  sleepRate.sleep();

  // exec.add_node(socketcan_to_topic);
  // exec.add_node(subscriber);
  // exec.spin_some();

  rclcpp::spin_some(subscriber);

  ASSERT_EQ(1, subscriber->messages.size());

  // compare the received can_msgs::Frame message to the sent can::Frame.
  can::Frame received;
  can_msgs::msg::Frame msg = subscriber->messages.back();
  socketcan_bridge::convertMessageToSocketCAN(msg, received,
                    socketcan_to_topic->s_to_t_id_signal_map_);

  EXPECT_EQ(received.id, f.id);
  EXPECT_EQ(received.dlc, f.dlc);
  EXPECT_EQ(received.is_extended, f.is_extended);
  EXPECT_EQ(received.is_rtr, f.is_rtr);
  EXPECT_EQ(received.is_error, f.is_error);
  EXPECT_EQ(received.data, f.data);

  dummy->shutdown();
  dummy.reset();
}

TEST(SocketCANToTopicTest, checkInvalidCanIdFilter)
{
  // rclcpp::executors::SingleThreadedExecutor exec;
  can::DummyBus bus("checkInvalidCanIdFilter");

  // create the dummy interface
  auto dummy = std::make_shared<can::ThreadedDummyInterface>();

  // create can_id vector with id that should not be received on can bus
  std::vector<unsigned int> pass_can_ids;
  pass_can_ids.push_back(0x300);

  // start the to topic bridge.
  auto socketcan_to_topic = std::make_shared<socketcan_bridge::SocketCANToTopic>(dummy);
  socketcan_to_topic->setup(can::tofilters(pass_can_ids));  // initiate the message callbacks
  auto signal_map = socketcan_to_topic->s_to_t_id_signal_map_;
  auto name_map = socketcan_to_topic->s_to_t_id_name_map_;

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // id is an explicitly specified valid id
  // will implement method so that gtest will cycle
  // through all valid ids created from the json
  int id = 835;
  auto iter = signal_map.find(id);
  auto iter2 = name_map.find(id);
  // register for messages
  std::string topic_name = iter2->second;
  auto subscriber = std::make_shared<GTestSubscriber>(topic_name);

  // create a can::frame
  can::Frame f;
  f.is_extended = true;
  f.is_rtr = false;
  f.is_error = false;
  f.id = iter->first;
  f.dlc = 8;
  for (uint8_t i=0; i < f.dlc; i++)
  {
    f.data[i] = i;
  }

  // send the can frame to the driver
  dummy->send(f);

  // give some time for the interface some time to process the message
  rclcpp::Rate sleepRate(std::chrono::seconds(1));
  sleepRate.sleep();

  // exec.add_node(socketcan_to_topic);
  // exec.add_node(subscriber);
  // exec.spin_some();

  rclcpp::spin_some(subscriber);

  EXPECT_EQ(0, subscriber->messages.size());

  dummy->shutdown();
  dummy.reset();
}

TEST(SocketCANToTopicTest, checkMaskFilter)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  can::DummyBus bus("checkMaskFilter");

  // create the dummy interface
  auto dummy = std::make_shared<can::ThreadedDummyInterface>();

  // setup filter
  can::FilteredFrameListener::FilterVector filters;
  filters.push_back(can::tofilter("300:ffe"));

  // start the to topic bridge.
  auto socketcan_to_topic = std::make_shared<socketcan_bridge::SocketCANToTopic>(dummy);
  socketcan_to_topic->setup(filters);  // initiate the message callbacks
  auto signal_map = socketcan_to_topic->s_to_t_id_signal_map_;
  auto name_map = socketcan_to_topic->s_to_t_id_name_map_;

  // init the driver to test stateListener (not checked automatically).
  dummy->init(bus.name, true, can::NoSettings::create());

  // id is an explicitly specified valid id
  // will implement method so that gtest will cycle
  // through all valid ids created from the json
  int id = 835;
  auto iter = signal_map.find(id);
  auto iter2 = name_map.find(id);
  // register for messages
  std::string topic_name = iter2->second;
  auto subscriber = std::make_shared<GTestSubscriber>(topic_name);

  const std::string pass1("300#1234"), nopass1("302#9999"), pass2("301#5678");

  // send the can frame to the driver with valid id
  can::Frame p1 = can::toframe(pass1);
  p1.id = iter->first;

  can::Frame np1 = can::toframe(nopass1);
  np1.id = iter->first;

  can::Frame p2 = can::toframe(pass2);
  p2.id = iter->first;

  dummy->send(p1);
  dummy->send(np1);
  dummy->send(p2);

  // give some time for the interface some time to process the message
  rclcpp::Rate sleepRate(std::chrono::seconds(1));
  sleepRate.sleep();

  exec.add_node(socketcan_to_topic);
  exec.add_node(subscriber);
  exec.spin_some();

  // rclcpp::spin_some(subscriber);

  // compare the received can_msgs::Frame message to the sent can::Frame.
  auto map = socketcan_to_topic->s_to_t_id_signal_map_;
  ASSERT_EQ(2, subscriber->messages.size());
  EXPECT_EQ(pass1, convertMessageToString(subscriber->messages.front(), map));
  EXPECT_EQ(pass2, convertMessageToString(subscriber->messages.back(), map));

  dummy->shutdown();
  dummy.reset();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Rate sleepRate(std::chrono::seconds(1));
  sleepRate.sleep();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
