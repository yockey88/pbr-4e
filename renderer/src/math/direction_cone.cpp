/**
 * \file math/direction_cone.cpp
 **/
#include "math/direction_cone.hpp"

#include <glm/gtx/vector_angle.hpp>

#include "util/float.hpp"
#include "math/math.hpp"
#include "math/vecmath.hpp"

namespace pbr {

  bool DirectionCone::IsEmpty() const {
    return cos_theta == infinity;
  }
      
  bool DirectionCone::Inside(const glm::vec3& w) {
    return !IsEmpty() && glm::dot(w , Normalize(w)) >= cos_theta;
  }
      
  DirectionCone DirectionCone::BoundSubtendedDirections(const Bounds3& b , const Point3& p) {
    float radius;
    Point3 center;

    b.BoundSphere(center , radius);
    if (DistanceSquared(p , center) < glm::pow(radius , 2)) {
      return DirectionCone::EntireSphere();
    }

    glm::vec3 w = Normalize(center - p);
    float sin2_theta_max = glm::pow(radius , 2) / DistanceSquared(center , p);
    float cos_theta_max = SafeSqrt(1 - sin2_theta_max);
    return DirectionCone(w , cos_theta_max);
  }

  DirectionCone DirectionCone::EntireSphere() {
    return DirectionCone(glm::vec3(0 , 0 , 1) , -1);
  }
      
  DirectionCone DirectionCone::Union(const DirectionCone& a , const DirectionCone& b) {
    if (a.IsEmpty()) {
      return b;
    }

    if (b.IsEmpty()) {
      return a;
    }

    float theta_a = SafeAcos(a.cos_theta);
    float theta_b = SafeAcos(b.cos_theta);
    float theta_d = AngleBetween(a.w , b.w);

    if (glm::min(theta_d + theta_b , pi) <= theta_a) {
      return a;
    }

    if (glm::min(theta_d + theta_a , pi) <= theta_b) {
      return b;
    }

    float theta_o = (theta_a + theta_d + theta_b) / 2;
    if (theta_o >= pi) {
      return DirectionCone::EntireSphere();
    }

    float theta_r = theta_o - theta_a;
    glm::vec3 wr = glm::cross(a.w , b.w);
    if (LengthSquared(wr) == 0) {
      return DirectionCone::EntireSphere();
    }

    glm::vec3 w; //  = Rotate(glm::degrees(theta_r) , wr)(a.w);
    return DirectionCone(w , glm::cos(theta_o));
  }

} // namespace pbr
