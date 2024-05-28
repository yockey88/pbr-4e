/**
 * \file math/direction_cone.hpp
 **/
#ifndef PBR_DIRECTION_CONE_HPP
#define PBR_DIRECTION_CONE_HPP

#include "util/float.hpp"
#include "math/vecmath.hpp"
#include "math/bounds.hpp"

namespace pbr {

  class DirectionCone {
    public:
      DirectionCone() {}
      DirectionCone(const glm::vec3& w , float cos_theta) 
        : w(w) , cos_theta(cos_theta) {}
      explicit DirectionCone(const glm::vec3& w)
        : DirectionCone(w , 1) {}

      bool IsEmpty() const;
      bool Inside(const glm::vec3& w);
      
      static DirectionCone BoundSubtendedDirections(const Bounds3& b , const Point3& p);

      static DirectionCone EntireSphere();
      
      static DirectionCone Union(const DirectionCone& a , const DirectionCone& b);

    private:
      glm::vec3 w;
      float cos_theta = infinity;
  };

} // namespace pbr

#endif // !PBR_DIRECTION_CONE_HPP
