/**
 * \file math/transform.hpp
 **/
#ifndef PBR_TRANSFORM_HPP
#define PBR_TRANSFORM_HPP

#include <glm/glm.hpp>

namespace pbr {

  class Transform {
    public:
      Transform(const glm::mat4& m);

    private:
      glm::mat4 model;
      glm::mat4 inverse_model;
  }; 

} // namespace pbr

#endif // !PBR_TRANSFORM_HPP
