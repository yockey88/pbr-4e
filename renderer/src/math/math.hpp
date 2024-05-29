/**
 * \file math/math.hpp
 **/
#ifndef PBR_MATH_HPP
#define PBR_MATH_HPP

#include <span>

#include <glm/glm.hpp>

#include "util/float.hpp"

namespace pbr {

  constexpr static float pi = 3.14159265358979323846f;

  template <typename T , typename U , typename V>
  constexpr T Clamp(T val , U low , V high) {
    if (val < low) {
      return T(low);
    } else if (val > high) {
      return T(high);
    } else {
      return val;
    }
  }

  template <typename Pred>
  size_t FindInterval(size_t sz , const Pred& pred) {
    using ssize_t = std::make_signed_t<size_t>;
    ssize_t size = static_cast<ssize_t>(sz) - 2;
    ssize_t first = 1;
    while (size > 0) {
      size_t half = static_cast<size_t>(size) >> 1;
      size_t middle = first + half;
      bool pred_res = pred(middle);
      first = pred_res ? 
        middle + 1 : first;
      size = pred_res ? 
        size - (half + 1) : half;
    }

    return static_cast<size_t>(Clamp(static_cast<ssize_t>(first) - 1 , 0 , sz - 2));
  }

  inline float SafeAsin(float x) {
    return glm::asin(Clamp(x , -1 , 1));
  }

  inline float SafeAcos(float x) {
    return glm::acos(Clamp(x , -1 , 1));
  }

  inline float SafeSqrt(float x) {
    return glm::sqrt(glm::max(0.f , x));
  }

  template <typename F , typename C>
  constexpr F EvaluatePolynomial(F t , C c) {
    return c;
  }

  template <typename F , typename C , typename... Args>
  F EvaluatePolynomial(F t , C c , Args... args) {
    return FMA(t , EvaluatePolynomial(t , args...) , c);
  }

  float FastExp(float x);

  float Lerp(float x , float a , float b);
  
  float Bilerp(const glm::vec2& p , const std::span<const float>& w);

} // namespace pbr

#endif // !PBR_MATH_HPP
