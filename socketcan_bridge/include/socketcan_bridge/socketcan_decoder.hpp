#ifndef SOCKETCAN_BRIDGE__SOCKETCAN_DECODER_HPP_
#define SOCKETCAN_BRIDGE__SOCKETCAN_DECODER_HPP_

#include <socketcan_bridge/socketcan_signal.hpp>
#include <can_msgs/msg/frame.hpp>

namespace socketcan_bridge
{
    void decodeSocketCANMessage(const std::array<unsigned char, 8> &data, 
                                const std::vector<socketcan_bridge::SocketCANSignal> &signals,
                                can_msgs::msg::Frame &msg);

}  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE__SOCKETCAN_DECODER_HPP_
