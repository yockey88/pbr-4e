/**
 * \file octahedral_vector.cpp
 **/
#include "octahedral_vector.hpp"

namespace pbr {

  OctahedralVector::OctahedralVector(const glm::vec3& v) {
    auto r_v = v / glm::abs(v.x) + glm::abs(v.y) + glm::abs(v.z);
    if (r_v.z >= 0) {
      x = Encode(v.x);
      y = Encode(v.y);
    } else {
      x = Encode((1 - glm::abs(r_v.y)) * Sign(v.x));
      y = Encode((1 - glm::abs(r_v.x)) * Sign(v.y));
    }
  }
      
  uint16_t OctahedralVector::Encode(float f) {
    return std::round(Clamp((f + 1) / 2 , 0 , 1) * 65535.f);
  }
      
  float OctahedralVector::Sign(float v) {
    return std::copysign(1.f , v);
  }

} // namespace pbr
