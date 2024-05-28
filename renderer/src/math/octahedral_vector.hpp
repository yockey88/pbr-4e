/**
 * \file math/octahedral_vector.hpp
 **/
#ifndef PBR_OCTAHEDRAL_VECTOR_HPP
#define PBR_OCTAHEDRAL_VECTOR_HPP

#include <glm/glm.hpp>

#include "math/vecmath.hpp"

namespace pbr {

  class OctahedralVector {
    public:
      OctahedralVector(const glm::vec3& vec);

      explicit operator glm::vec3() const {
        glm::vec3 v;
        v.x = -1 + 2 * (x / 65535.f);
        v.y = -1 + 2 * (y / 65535.f);
        v.z = 1 - (glm::abs(v.x) + glm::abs(v.y));
        if (v.z < 0) {
          float xo = v.x;
          v.x = (1 - glm::abs(v.y)) * Sign(xo);
          v.y = (1 - glm::abs(xo)) * Sign(v.y);
        }
        return Normalize(v);
      }

    private:
      uint16_t x;
      uint16_t y;

      static uint16_t Encode(float f); 
      static float Sign(float v);
  };

} // namespace pbr

#endif // !PBR_OCTAHEDRAL_VECTOR_HPP
