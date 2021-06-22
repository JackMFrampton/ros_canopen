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
#include <socketcan_bridge/socketcan_decoder_encoder.hpp>

#include <can_msgs/msg/frame.hpp>
#include <socketcan_interface/socketcan.hpp>
#include <socketcan_interface/dummy.hpp>

#include <boost/random.hpp>

#include <gtest/gtest.h>
#include <list>
#include <memory>
#include <string>
#include <chrono>
#include <map>
#include <vector>
#include "rclcpp/rclcpp.hpp"

class GTestPublisher : public rclcpp::Node
{
  public:
    explicit GTestPublisher(const std::string &topic_name)
    : Node("gtest_publisher")
    {
      publisher_ = this->create_publisher<can_msgs::msg::Frame>(topic_name.c_str(), 10);
    }

    void PublishMsg(const can_msgs::msg::Frame &msg)
    {
      publisher_->publish(msg);
      // RCLCPP_INFO(this->get_logger(), "ros msg published");  // debug
    }
  private:
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
};

class frameCollector
{
  public:
    std::list<can::Frame> frames;

    frameCollector() {}

    void frameCallback(const can::Frame& f)
    {
      frames.push_back(f);
    }
};

TEST(TopicToSocketCANTest, checkCorrectData)
{
  can::DummyBus bus("checkCorrectData");
  // create the dummy interface
  can::ThreadedDummyInterfaceSharedPtr dummy = std::make_shared<can::ThreadedDummyInterface>();

  // start the to topic bridge.
  auto topic_to_socketcan = std::make_shared<socketcan_bridge::TopicToSocketCAN>(dummy);
  auto signal_map = topic_to_socketcan->t_to_s_id_signal_map_;
  auto name_map = topic_to_socketcan->t_to_s_id_name_map_;

  // id is an explicitly specified valid id
  // will implement method so that gtest will cycle
  // through all valid ids created from the json
  for(auto iter = name_map.begin(); iter != name_map.end(); ++iter)
  {
    uint32_t valid_id = iter->first;
    auto signal_iter = signal_map.find(valid_id);

    topic_to_socketcan->setup();

    // init the driver to test stateListener (not checked automatically).
    dummy->init(bus.name, true, can::NoSettings::create());

    // register for messages
    std::string topic_name = iter->second;
    auto publisher = std::make_shared<GTestPublisher>(topic_name);

    // create a frame collector.
    frameCollector frame_collector_;

    //  driver->createMsgListener(&frameCallback);
    can::FrameListenerConstSharedPtr frame_listener_ = dummy->createMsgListener(
              std::bind(&frameCollector::frameCallback, &frame_collector_, std::placeholders::_1));

    // create a can_msgs::msg::frame message
    can_msgs::msg::Frame msg;
    msg.is_extended = true;
    msg.is_rtr = false;
    msg.is_error = false;
    msg.id = valid_id;
    msg.dlc = 8;

    boost::random::mt19937 rng;
    rng.seed(time(NULL));

    if (signal_iter->second.size() > 0)
    {
      // create random dummy frame data
      // from this we will get random signal values
      std::array<uint8_t, 8> data;
      for (uint8_t i=0; i < data.size(); i++)
      {
        boost::random::uniform_int_distribution<> gen(0, 255);
        data[i] = gen(rng);
      }

      for (auto &signal : signal_iter->second)
      {
        // break if the signal has no bit length
        // a CAN msg that spans the full 64 bits
        // will have a dummy signal with 0 bit length
        // some dummy signals will have a bit length
        // this is so that the original data is preserved
        if (signal.bit_length_ == 0)
        {
          signal.value_ = 0.0;
          msg.signal_names.push_back(signal.signal_name_);
          msg.signal_values.push_back(signal.value_);
          break;
        }

        signal.value_ = decode(data.data(), signal);

        msg.signal_names.push_back(signal.signal_name_);
        msg.signal_values.push_back(signal.value_);
      }
    }

    msg.header.frame_id = "0";  // "0" for no frame.
    msg.header.stamp = topic_to_socketcan->get_clock()->now();
    // send the can_frame::Frame message to the sent_messages topic.
    publisher->PublishMsg(msg);

    // give some time for the interface some time to process the message
    rclcpp::Rate sleepRate(std::chrono::seconds(1));
    sleepRate.sleep();

    rclcpp::spin_some(topic_to_socketcan);

    dummy->flush();

    can_msgs::msg::Frame received;
    can::Frame f = frame_collector_.frames.back();
    RCLCPP_INFO(topic_to_socketcan->get_logger(), "CAN name: %s | id: %i",
                                                  iter->second.c_str(), valid_id);
    socketcan_bridge::convertSocketCANToMessage(f, received, signal_map);

    EXPECT_EQ(received.id, msg.id);
    EXPECT_EQ(received.dlc, msg.dlc);
    EXPECT_EQ(received.is_extended, msg.is_extended);
    EXPECT_EQ(received.is_rtr, msg.is_rtr);
    EXPECT_EQ(received.is_error, msg.is_error);

    for (uint8_t i=0; i < signal_iter->second.size(); i++)
    {
      EXPECT_EQ(received.signal_names[i], msg.signal_names[i]);
      EXPECT_FLOAT_EQ(received.signal_values[i], msg.signal_values[i]);
    }
  }
  dummy->shutdown();
  dummy.reset();
}

TEST(TopicToSocketCANTest, checkInvalidFrameHandling)
{
  // - tries to send a non-extended frame with an id larger than 11 bits.
  //   that should not be sent.
  // - verifies that sending one larger than 11 bits actually works.
  // - tries sending a message with a dlc > 8 bytes, this should not be sent.
  //   sending with 8 bytes is verified by the checkCorrectData testcase.
  can::DummyBus bus("checkInvalidFrameHandling");

  // create the dummy interface
  can::ThreadedDummyInterfaceSharedPtr dummy = std::make_shared<can::ThreadedDummyInterface>();

  // start the to topic bridge.
  auto topic_to_socketcan = std::make_shared<socketcan_bridge::TopicToSocketCAN>(dummy);
  auto signal_map = topic_to_socketcan->t_to_s_id_signal_map_;
  auto name_map = topic_to_socketcan->t_to_s_id_name_map_;

  // id is an explicitly specified valid id
  // will implement method so that gtest will cycle
  // through all valid ids created from the json
  for(auto iter = name_map.begin(); iter != name_map.end(); ++iter)
  {
    uint32_t valid_id = iter->first;

    topic_to_socketcan->setup();

    dummy->init(bus.name, true, can::NoSettings::create());

    // register for messages
    std::string topic_name = iter->second;
    auto publisher = std::make_shared<GTestPublisher>(topic_name.c_str());

    // create a frame collector.
    frameCollector frame_collector_;

    //  add callback to the dummy interface.
    can::FrameListenerConstSharedPtr frame_listener_ = dummy->createMsgListener(
            std::bind(&frameCollector::frameCallback, &frame_collector_, std::placeholders::_1));

    // create a message
    can_msgs::msg::Frame msg;

    msg.id = (1<<11)+1;  // this is an illegal CAN packet... should not be sent.
    msg.dlc = 8;
    msg.is_error = false;
    msg.is_rtr = false;
    msg.is_extended = false;
    msg.header.frame_id = "0";  // "0" for no frame.
    msg.header.stamp = topic_to_socketcan->get_clock()->now();

    // send the can_frame::Frame message to the sent_messages topic.
    publisher->PublishMsg(msg);

    // give some time for the interface some time to process the message
    rclcpp::Rate sleepRate(std::chrono::seconds(1));
    sleepRate.sleep();

    rclcpp::spin_some(topic_to_socketcan);

    dummy->flush();
    RCLCPP_INFO(topic_to_socketcan->get_logger(), "CAN name: %s | id: %i",
                                                  iter->second.c_str(), valid_id);

    EXPECT_EQ(frame_collector_.frames.size(), 0);

    msg.is_extended = true;
    msg.id = valid_id;  // now it should be alright
    // send the can_frame::Frame message to the sent_messages topic.
    publisher->PublishMsg(msg);
    sleepRate.sleep();
    rclcpp::spin_some(topic_to_socketcan);
    dummy->flush();

    EXPECT_EQ(frame_collector_.frames.size(), 1);

    // wipe the frame queue.
    frame_collector_.frames.clear();

    // finally, check if frames with a dlc > 8 are discarded.
    msg.dlc = 10;
    publisher->PublishMsg(msg);
    sleepRate.sleep();
    rclcpp::spin_some(topic_to_socketcan);
    dummy->flush();

    EXPECT_EQ(frame_collector_.frames.size(), 0);
  }
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
