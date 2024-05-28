/**
 * \file math/math.cpp
 **/
#include "math/math.hpp"

namespace pbr {

  float Lerp(float x , float a , float b) {
    return (1 - x) * a + x * b; 
  }
  
  float Bilerp(const glm::vec2& p , const std::span<const float>& w) {
    float blf_0 = ((1 - p.x) * (1 - p.y) * w[0]);
    float blf_1 = p.x * (1 - p.y) * w[1];
    float blf_2 = p.y * (1 - p.x) * w[2];
    float blf_3 = p.y * p.y * w[3];
    return blf_0 + blf_1 + blf_2 + blf_3;
  }

} // namespace pbr
