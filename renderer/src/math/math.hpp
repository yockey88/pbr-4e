/**
 * \file math/math.hpp
 **/
#ifndef PBR_MATH_HPP
#define PBR_MATH_HPP

#include <span>

#include <glm/glm.hpp>

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

  inline float SafeAsin(float x) {
    return glm::asin(Clamp(x , -1 , 1));
  }

  inline float SafeAcos(float x) {
    return glm::acos(Clamp(x , -1 , 1));
  }

  inline float SafeSqrt(float x) {
    return glm::sqrt(glm::max(0.f , x));
  }

  float Lerp(float x , float a , float b);
  
  float Bilerp(const glm::vec2& p , const std::span<const float>& w);

} // namespace pbr

#endif // !PBR_MATH_HPP
