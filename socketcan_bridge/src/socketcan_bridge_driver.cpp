#include "rclcpp/rclcpp.hpp"
#include <socketcan_bridge/topic_to_socketcan.hpp>
#include <socketcan_bridge/socketcan_to_topic.hpp>
#include <socketcan_bridge/socketcan_bridge_driver.hpp>
#include <socketcan_interface/threading.hpp>
#include <socketcan_interface/xmlrpc_settings.hpp>
#include <memory>
#include <string>

namespace socketcan_bridge_driver
{

SocketCANDriver::SocketCANDriver(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name, options) 
{

}

SocketCANDriver::~SocketCANDriver()
{
    driver->shutdown();
    driver.reset();
}

void SocketCANDriver::init_param()
{
    auto flag_can_device = get_parameter_or("can_device", can_device, rclcpp::Parameter("can_device", "can0"))
    if (!flag_can_device){
        RCLCPP_WARN_ONCE(get_logger(), "Could not get can device, setting: %s", can_device.as_string().c_str());)
    }

void SocketCANDriver::init_can()
{
    driver = std::make_shared<can::ThreadedSocketCANInterface> ();

    if (!driver->init(can_device.as_string().c_str(), 0, XmlRpcSettings::create(this))){
        ROS_FATAL("Failed to initialize can_device at %s", can_device.c_str());
        return 1;
    }
    else{
        ROS_INFO("Successfully connected to %s.", can_device.c_str());
    }
}
}

}; // namespace socketcan_bridge_driver