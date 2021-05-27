#include <socketcan_bridge/socketcan_decoder.hpp>
#include <vector>
#include <array>
#include <algorithm>
#include <bitset>

namespace socketcan_bridge
{

    void decodeSocketCANMessage(const std::array<unsigned char, 8> &data, 
                                const std::vector<socketcan_bridge::SocketCANSignal> &signals,
                                can_msgs::msg::Frame &msg)
        {
            for (const auto &sig : signals)
            {
                float tmp_signal_val;
                std::string tmp_signal_name;

                std::vector<int> tmp_bytes;
                for (int i = sig.start_byte_; i < sig.end_byte_ + 1; i++)
                {
                    tmp_bytes.push_back(data[i]);

                    if (!sig.is_big_endian_ && tmp_bytes.size() >= 2)
                    {
                        std::reverse(tmp_bytes.begin(), tmp_bytes.end());
                    }
                }

                std::string tmp_byte_string = "";
                for (const auto &byte : tmp_bytes) 
                {
                    tmp_byte_string = tmp_byte_string + std::bitset<8>(byte).to_string();
                }
                
                if (tmp_bytes.size() == 1)
                {
                    auto tmp_bits = std::bitset<8>(tmp_byte_string);
                    tmp_signal_val = tmp_bits.to_ulong();
                    if (sig.bit_length_ < 8)
                    {
                        tmp_byte_string = "";
                        for (int i = sig.start_bit_; i > sig.end_bit_ - 1; i--)
                        {
                            std::bitset<1> tmp_bit;
                            tmp_bit[0] = tmp_bits[i];
                            tmp_byte_string = tmp_byte_string + tmp_bit.to_string();
                        }
                        auto tmp_bits = std::bitset<8>(tmp_byte_string);
                        tmp_signal_val = tmp_bits.to_ulong();
                    }
                }else if (tmp_bytes.size() == 2){
                    auto tmp_bits = std::bitset<16>(tmp_byte_string);
                    tmp_signal_val = tmp_bits.to_ulong();
                }else if (tmp_bytes.size() == 3){
                    auto tmp_bits = std::bitset<24>(tmp_byte_string);
                    tmp_signal_val = tmp_bits.to_ulong();
                }else if (tmp_bytes.size() == 4){
                    auto tmp_bits = std::bitset<32>(tmp_byte_string);
                    tmp_signal_val = tmp_bits.to_ulong();
                }else if (tmp_bytes.size() == 5){
                    auto tmp_bits = std::bitset<40>(tmp_byte_string);
                    tmp_signal_val = tmp_bits.to_ulong();
                }else if (tmp_bytes.size() == 6){
                    auto tmp_bits = std::bitset<48>(tmp_byte_string);
                    tmp_signal_val = tmp_bits.to_ulong();
                }else if (tmp_bytes.size() == 7){
                    auto tmp_bits = std::bitset<56>(tmp_byte_string);
                    tmp_signal_val = tmp_bits.to_ulong();
                }else if (tmp_bytes.size() == 8){
                    auto tmp_bits = std::bitset<64>(tmp_byte_string);
                    tmp_signal_val = tmp_bits.to_ulong();
                }
                
                tmp_signal_val = (tmp_signal_val*sig.factor_) + sig.offset_;
                tmp_signal_name = sig.name_;

                msg.signal_names.push_back(tmp_signal_name);
                msg.signal_values.push_back(tmp_signal_val);
            }
        }

}  // namespace socketcan_bridge
