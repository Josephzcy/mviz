
#ifndef TYPE_CONVERSION_HPP_
#define TYPE_CONVERSION_HPP_
#include <sstream>
#include <string>
#include <iomanip>
namespace mviz_record::type_conversion {

// transfer string to hex-string
std::string string_to_hex(const std::string& str) {
  // std::string result = "0x";
  std::string result;
  std::string tmp;
  std::stringstream ss;
  for (size_t i = 0; i < str.size(); i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(static_cast<uint8_t>(str[i])) << std::endl;
    ss >> tmp;
    result += tmp;
  }
  return result;
}

std::string int_to_hex(const int& can_id) {
  std::string result = "0x";
  // std::string result;
  std::string tmp;
  std::stringstream ss;
  ss << std::hex << can_id << std::endl;
  ss >> tmp;
  return result += tmp;
}

void SplitFloat(double& timestamp, std::string& timestamp_s, std::string& timestamp_ns) {
  std::string timestamp_str = std::to_string(timestamp);
  int dot_index = timestamp_str.find('.');
  timestamp_s = timestamp_str.substr(0, dot_index);
  timestamp_ns = timestamp_str.substr(dot_index + 1, timestamp_str.size() - dot_index - 1);
}

}  // namespace mviz_record::type_conversion
#endif