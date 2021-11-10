#pragma once

#include <limits>

constexpr float FLOAT_MAX = std::numeric_limits<float>::max();
//notice: std::numeric_limits<float>::min() is e-38(near to zero not negative)
constexpr float FLOAT_MIN = -FLOAT_MAX;
constexpr float FLOAT_EPSILON = 0.5f*std::numeric_limits<float>::epsilon();
constexpr int MAX_INT = std::numeric_limits<int>::max();
constexpr int MIN_INT = std::numeric_limits<int>::min();

const float CUSTOM_EPSILON = 1e-9;
const float PI = 3.1415926536;
const float PI2 = 6.2831853072;
const float PI4 = 12.5663706144;
const float INV_PI = 0.3183098862;
const float INV_PI2 = 0.1591549431; // 1/(2pi)

inline constexpr float _Gamma(int n) {
  return n*FLOAT_EPSILON / (1.0f - n*FLOAT_EPSILON);
}