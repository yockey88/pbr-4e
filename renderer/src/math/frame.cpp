/**
 * \file math/frame.cpp
 **/
#include "math/frame.hpp"

#include "math/vecmath.hpp"

namespace pbr {
      
  glm::vec3 Frame::Local(const glm::vec3& n) const {
    return glm::vec3(glm::dot(n , x) , glm::dot(n , y) , glm::dot(n , z));
  }
      
  glm::vec3 Frame::FromLocal(const glm::vec3& v) const {
    return v.x * x + v.y * y + v.z * z;
  }

  Frame Frame::FromXZ(const glm::vec3& x , const glm::vec3& z) {
    return Frame(x , glm::cross(z , x) , z);
  }
  
  Frame Frame::FromXY(const glm::vec3& x , const glm::vec3& y) {
    return Frame(x , y , glm::cross(x , y));
  }

  Frame Frame::FromZ(const glm::vec3& z) {
    glm::vec3 x , y;
    BuildOrthonormalBasis(z , x , y);
    return Frame(x , y , z);
  }


} // namespace pbr
