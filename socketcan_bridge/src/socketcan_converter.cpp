#include <socketcan_bridge/socketcan_converter.hpp>

namespace socketcan_bridge
{

  void convertSocketCANToMessage(const can::Frame& f, can_msgs::msg::Frame& m)
  {
    m.id = f.id;
    m.dlc = f.dlc;
    m.is_error = f.is_error;
    m.is_rtr = f.is_rtr;
    m.is_extended = f.is_extended;

    for (int i = 0; i < 8; i++)  // always copy all data, regardless of dlc.
    {
      m.data[i] = f.data[i];
    }
  }

  void convertMessageToSocketCAN(const can_msgs::msg::Frame& m, can::Frame& f)
  {
    f.id = m.id;
    f.dlc = m.dlc;
    f.is_error = m.is_error;
    f.is_rtr = m.is_rtr;
    f.is_extended = m.is_extended;

    for (int i = 0; i < 8; i++)  // always copy all data, regardless of dlc.
    {
      f.data[i] = m.data[i];
    }
  }

} // namespace socketcan_bridge
