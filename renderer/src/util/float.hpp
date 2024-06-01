/**
 * \file util/float.hpp
 **/
#ifndef PBR_FLOAT_HPP
#define PBR_FLOAT_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <bit>

namespace pbr {

constexpr static float kDoubleOneMinusEpsilon = 0x1.fffffffffffffp-1;
constexpr static float kFloatOneMinusEpsilon = float(0x1.fffffep-1);

#if 0 // PBR_USE_FLOAT_AS_DOUBLE
  constexpr static double kOneMinusEpsilon = kDoubleOneMinusEpsilon; 
#else
  constexpr static float kOneMinusEpsilon = kFloatOneMinusEpsilon; 
#endif
 
constexpr static float infinity = std::numeric_limits<float>::infinity();
constexpr static float kMachineEpsilon = std::numeric_limits<float>::epsilon() * 0.5f;

template <typename T>
inline typename std::enable_if_t<std::is_floating_point_v<T>, bool> IsNaN(T v) {
#ifdef PBRT_IS_GPU_CODE
  return isnan(v);
#else
  return std::isnan(v);
#endif
}

template <typename T>
inline typename std::enable_if_t<std::is_integral_v<T>, bool> IsNaN(T v) {
  return false;
}

template <typename T>
inline typename std::enable_if_t<std::is_floating_point_v<T>, bool> IsInf(T v) {
#ifdef PBRT_IS_GPU_CODE
  return isinf(v);
#else
  return std::isinf(v);
#endif
}

template <typename T>
inline typename std::enable_if_t<std::is_integral_v<T>, bool> IsInf(T v) {
  return false;
}

template <typename T>
inline typename std::enable_if_t<std::is_floating_point_v<T>, bool> IsFinite(T v) {
#ifdef PBRT_IS_GPU_CODE
  return isfinite(v);
#else
  return std::isfinite(v);
#endif
}
template <typename T>
inline typename std::enable_if_t<std::is_integral_v<T>, bool> IsFinite(T v) {
  return true;
}

inline float FMA(float a, float b, float c) {
  return std::fma(a, b, c);
}

inline double FMA(double a, double b, double c) {
  return std::fma(a, b, c);
}


inline uint32_t FloatToBits(float f) {
  return std::bit_cast<uint32_t>(f);
}

inline float BitsToFloat(uint32_t b) {
  return std::bit_cast<float>(b);
}

inline int32_t Exponent(float v) {
  return (FloatToBits(v) >> 23) - 127;
}

inline int32_t Significand(float v) {
  return FloatToBits(v) & ((1 << 23) - 1);
}

inline uint32_t SignBit(float v) {
  return FloatToBits(v) & 0x80000000;
}

inline uint64_t DoubleToBits(double d) {
  return std::bit_cast<uint64_t>(d);
}

inline double BitsToDouble(uint64_t b) {
  return std::bit_cast<double>(b);
}

inline float NextFloatUp(float v) {
  if (IsInf(v) && v > 0.f) {
    return v;
  }

  if (v == -0.f) {
    v = 0.f;
  }

  uint32_t ui = FloatToBits(v);
  if (v >= 0) {
    ++ui;
  } else {
    --ui;
  }

  return BitsToFloat(ui);
}

inline float NextFloatDown(float v) {
  if (IsInf(v) && v < 0.f) {
    return v;
  }

  if (v == 0.f) {
    v = -0.f;
  }

  uint32_t ui = FloatToBits(v);
  if (v > 0) {
    --ui;
  } else {
    ++ui;
  }

  return BitsToFloat(ui);
}

inline constexpr float Gamma(int32_t n) {
  return (n * kMachineEpsilon) / (1 - n * kMachineEpsilon);
}

} // namespace pbr

#endif // !PBR_FLOAT_HPP
