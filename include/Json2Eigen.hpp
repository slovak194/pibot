#pragma once

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

struct type {
  template<typename T>
  using base_type = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
  template<typename T>
  static constexpr const char *dtype() {
    if constexpr(std::is_same<base_type<T>, char>::value) { return "u1"; }
    if constexpr(std::is_same<base_type<T>, std::uint8_t>::value) { return "u1"; }
    else if constexpr(std::is_same<base_type<T>, std::uint16_t>::value) { return "u2"; }
    else if constexpr(std::is_same<base_type<T>, std::uint32_t>::value) { return "u4"; }
    else if constexpr(std::is_same<base_type<T>, std::uint64_t>::value) { return "u8"; }
    else if constexpr(std::is_same<base_type<T>, std::int8_t>::value) { return "i1"; }
    else if constexpr(std::is_same<base_type<T>, std::int16_t>::value) { return "i2"; }
    else if constexpr(std::is_same<base_type<T>, std::int32_t>::value) { return "i4"; }
    else if constexpr(std::is_same<base_type<T>, std::int64_t>::value) { return "i8"; }
    else if constexpr(std::is_same<base_type<T>, float>::value) { return "f4"; }
    else if constexpr(std::is_same<base_type<T>, double>::value) { return "f8"; }
    else if constexpr(std::is_same<base_type<T>, long double>::value) { return "f8"; }
    else { return "unknown type"; }
  }
};

namespace Eigen {
template<typename T>
void to_json(nlohmann::json &j, const T &matrix) {
  typename T::Index rows = matrix.rows(), cols = matrix.cols();
  auto data_begin = const_cast<typename T::Scalar *>(matrix.data());
  std::vector<typename T::Scalar> v(data_begin, data_begin + rows * cols);

  bool is_row_major = matrix.IsRowMajor;

  j = {
      {"dtype", type::dtype<typename T::Scalar>()},
      {"shape", {rows, cols}},
      {"data", v},
      {"order", is_row_major ? "C" : "F"}
  };
}
}
