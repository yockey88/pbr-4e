/**
 * \file camera/filter.hpp
 **/
#ifndef PBR_FILTER_HPP
#define PBR_FILTER_HPP

#include <glm/glm.hpp>

#include "util/ref.hpp"

namespace pbr {

  class Filter : public RefCounted {
    public:
      glm::vec2 Radius() const {
        return glm::vec2{ 0 , 0 }; 
      }

      float Integral() const {
        return 0.f;
      }
  };

  class BoxFilter : public Filter {};
  
  class GaussianFilter : public Filter {};

  class MitchellFilter : public Filter {};

  class LanczosFilter : public Filter {};

  class TriangleFilter : public Filter {};

} // namespace pbr

#endif // !PBR_FILTER_HPP
