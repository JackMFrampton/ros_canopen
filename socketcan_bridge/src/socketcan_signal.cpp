#include <socketcan_bridge/socketcan_signal.hpp>

namespace socketcan_bridge
{
    SocketCANSignal::SocketCANSignal(const int &bit_length,
                                    const float &factor,
                                    const bool &is_big_endian,
                                    const bool &is_signed,
                                    const int &max,
                                    const int &min,
                                    const std::string &name,
                                    const float &offset,
                                    const int &start_bit) :
                                    bit_length_{ bit_length },
                                    factor_{ factor },
                                    is_big_endian_{ is_big_endian },
                                    is_signed_{ is_signed },
                                    max_{ max },
                                    min_{ min },
                                    name_{ name },
                                    offset_{ offset },
                                    start_index_bit_{ start_bit }
                                    
    {
        end_index_bit_ = start_index_bit_ + bit_length_ - 1;
        start_byte_ = start_index_bit_ / 8;
        end_byte_ = end_index_bit_ / 8;
        bitset_width_factor_ = end_byte_ - start_byte_ + 1;
        start_bit_ = (7 + (8 * (bitset_width_factor_ - 1))) - start_index_bit_;
        end_bit_ = start_bit_ - bit_length_ + 1;
    }
}  // namespace socketcan_bridge
