#include <class_loader/class_loader.hpp> 
#include <socketcan_interface/socketcan.hpp>

CLASS_LOADER_REGISTER_CLASS(can::SocketCANInterface, can::DriverInterface);
