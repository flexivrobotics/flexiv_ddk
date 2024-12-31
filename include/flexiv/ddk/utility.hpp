/**
 * @file utility.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DDK_UTILITY_HPP_
#define FLEXIV_DDK_UTILITY_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
namespace flexiv {
namespace ddk {
namespace utility {

/**
 * @brief Convert an std::vector to a string.
 * @param[in] vec std::vector of any type and size.
 * @param[in] decimal Decimal places to keep for each number in the vector.
 * @param[in] trailing_space Whether to include a space after the last element.
 * @param[in] separator Character to separate between numbers.
 * @return A string with format "vec[0] vec[1] ... vec[n] ", i.e. each element
 * followed by a space, including the last one if trailing_space = true.
 */
template <typename T>
inline std::string Vec2Str(const std::vector<T> &vec, size_t decimal = 3,
                           bool trailing_space = true,
                           const std::string &separator = " ") {
  if (vec.empty()) {
    return "";
  }

  std::stringstream ss;
  ss.precision(decimal);
  ss << std::fixed;

  for (size_t i = 0; i < vec.size(); ++i) {
    if (i > 0) {
      ss << separator;
    }
    ss << vec[i];
  }

  if (trailing_space) {
    ss << " ";
  }
  return ss.str();
}

/**
 * @brief Convert an std::array to a string.
 * @param[in] arr std::array of any type and size.
 * @param[in] decimal Decimal places to keep for each number in the array.
 * @param[in] trailing_space Whether to include a space after the last element.
 * @param[in] separator Character to separate between numbers.
 * @return A string with format "arr[0] arr[1] ... arr[n] ", i.e. each element
 * followed by a space, including the last one if trailing_space = true.
 */
template <typename T, size_t N>
inline std::string Arr2Str(const std::array<T, N> &arr, size_t decimal = 3,
                           bool trailing_space = true,
                           const std::string &separator = " ") {
  std::vector<T> vec(arr.begin(), arr.end());
  return Vec2Str(vec, decimal, trailing_space, separator);
}

/**
 * @brief Check if any provided strings exist in the program arguments.
 * @param[in] argc Argument count passed to main() of the program.
 * @param[in] argv Argument vector passed to main() of the program, where
 * argv[0] is the program name.
 * @param[in] ref_strings Reference strings to check against.
 * @return True if the program arguments contain one or more reference strings.
 */
inline bool ProgramArgsExistAny(int argc, char **argv,
                                const std::vector<std::string> &ref_strings) {
  for (int i = 0; i < argc; i++) {
    for (const auto &v : ref_strings) {
      if (v == std::string(argv[i])) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Check if one specific string exists in the program arguments.
 * @param[in] argc Argument count passed to main() of the program.
 * @param[in] argv Argument vector passed to main() of the program, with argv[0]
 * being the program name.
 * @param[in] ref_strings Reference string to check against.
 * @return True if the program arguments contain this specific reference string.
 */
inline bool ProgramArgsExist(int argc, char **argv,
                             const std::string &ref_strings) {
  return ProgramArgsExistAny(argc, argv, {ref_strings});
}

/**
 * @brief Convert the server time into format data time string.
 * @param sec_since_epoch Current seconds since epoch.
 * @param nano_sec_since_full_sec Number of nanoseconds since last full second.
 * @param format The format of target date time string.
 * @return Converted date time string.
 */
inline std::string
convertToDateTimeString(int sec_since_epoch, int nano_sec_since_full_sec,
                        const std::string &format = "%Y-%m-%d %H:%M:%S") {
  // Convert sec_since_epoch to a time_point
  std::chrono::system_clock::time_point tp =
      std::chrono::system_clock::time_point(
          std::chrono::seconds(sec_since_epoch));

  // Convert time_point to time_t for conversion to tm structure
  std::time_t time = std::chrono::system_clock::to_time_t(tp);

  // Convert time_t to tm (local time)
  std::tm *local = std::localtime(&time);

  // Create a stringstream to format the output
  std::stringstream ss;

  // Format the time using std::put_time and the provided format
  ss << std::put_time(local, format.c_str());

  // Add nanoseconds part, if needed
  ss << "." << std::setw(9) << std::setfill('0') << nano_sec_since_full_sec;

  return ss.str();
}

} /* namespace utility */
} /* namespace ddk */
} /* namespace flexiv */

#endif /* FLEXIV_DDK_UTILITY_HPP_ */
