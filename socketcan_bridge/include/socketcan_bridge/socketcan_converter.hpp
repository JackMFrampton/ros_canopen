#ifndef SOCKETCAN_BRIDGE_SOCKETCAN_CONVERTER_H
#define SOCKETCAN_BRIDGE_SOCKETCAN_CONVERTER_H

#include <socketcan_interface/socketcan.hpp>
#include <can_msgs/msg/frame.hpp>

namespace socketcan_bridge
{

void convertSocketCANToMessage(const can::Frame& f, can_msgs::msg::Frame& m);

void convertMessageToSocketCAN(const can_msgs::msg::Frame& m, can::Frame& f);

}  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE_SOCKETCAN_CONVERTER_H