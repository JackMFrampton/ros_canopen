#ifndef SOCKETCAN_BRIDGE_SOCKETCAN_BRIDGE_DRIVER_H
#define SOCKETCAN_BRIDGE_SOCKETCAN_BRIDGE_DRIVER_H

#include <socketcan_bridge/topic_to_socketcan.hpp>
#include <socketcan_bridge/socketcan_to_topic.hpp>
#include <socketcan_interface/threading.hpp>
#include <socketcan_interface/xmlrpc_settings.hpp>

namespace socketcan_bridge_driver
{
class SocketCANDriver : public rclcpp::Node
{
  public:
    SocketCANDriver(const std::string &node_name, const rclcpp::NodeOptions &options);  // Constructor
    ~SocketCANDriver(); // Destructor

    void init_param();  // Initiate node parameters
    void init_can_driver(); // Initiate can device
    void init_topic_to_socket_can(std::shared_ptr<rclcpp::Node> nh);  // Subscriber
    void init_socket_can_to_topic(std::shared_ptr<rclcpp::Node> nh);  // Publisher

  private:
    rclcpp::Parameter can_device;
    can::ThreadedSocketCANInterfaceSharedPtr driver;
};

}  // namespace socketcan_bridge_driver


#endif  // SOCKETCAN_BRIDGE_SOCKETCAN_BRIDGE_DRIVER_H