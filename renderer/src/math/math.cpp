/**
 * \file math/math.cpp
 **/
#include "math/math.hpp"
#include "math/vecmath.hpp"
#include "util/float.hpp"

namespace pbr {
  
  float FastExp(float x) {
    float xp = x * 1.442695041f;
    float fxp = glm::floor(xp);
    float f = xp - fxp;
    int32_t i = static_cast<int32_t>(fxp);
    float two_to_f = EvaluatePolynomial(f , 1.f , 0.695556856f , 0.226173572f , 0.0781455737f);
    int32_t exp = Exponent(two_to_f) + i;
    if (exp < -126) {
      return 0;
    }

    if (exp > 127) {
      return infinity;
    }

    uint32_t bits = FloatToBits(two_to_f);
    bits |= (exp + 127) << 23;
    return BitsToFloat(bits);
  }

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
