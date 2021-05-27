#ifndef SOCKETCAN_BRIDGE__SOCKETCAN_SIGNAL_HPP_
#define SOCKETCAN_BRIDGE__SOCKETCAN_SIGNAL_HPP_

#include <string>

namespace socketcan_bridge
{
class SocketCANSignal
{
  public:
    SocketCANSignal(const int &bit_length,
                    const float &factor,
                    const bool &is_big_endian,
                    const bool &is_signed,
                    const int &max,
                    const int &min,
                    const std::string &name,
                    const float &offset,
                    const int &start_bit);

    int end_index_bit_;
    int start_byte_;
    int end_byte_;
    int bitset_width_factor_;
    int start_bit_;
    int end_bit_;
    int bit_length_;
    float factor_;
    bool is_big_endian_;
    bool is_signed_;
    int max_;
    int min_;
    std::string name_;
    float offset_;
    int start_index_bit_;
};

}  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE__SOCKETCAN_SIGNAL_HPP_
