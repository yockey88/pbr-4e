/**
 * \file math/frame.hpp
 **/
#ifndef PBR_FRAME_HPP
#define PBR_FRAME_HPP

#include <glm/glm.hpp>

namespace pbr {

  class Frame {
    public:
      Frame()
        : x(1 , 0 , 0) , y(0 , 1 , 0) , z(0 , 0 , 1) {}
      Frame(const glm::vec3& x , const glm::vec3& y , const glm::vec3& z)
        : x(x) , y(y) , z(z) {}

      glm::vec3 Local(const glm::vec3& n) const;
      
      glm::vec3 FromLocal(const glm::vec3& v) const;

      static Frame FromXZ(const glm::vec3& x , const glm::vec3& z);
      
      static Frame FromXY(const glm::vec3& x , const glm::vec3& y);

      static Frame FromZ(const glm::vec3& z);

      glm::vec3 x , y , z;
  };

} // namespace pbr

#endif // !PBR_FRAME_HPP
