/**
 * \file math/ray.cpp
 **/
#include "math/ray.hpp"

namespace pbr {

  const Point3& Ray::Origin() const {
    return origin;
  }

  const glm::vec3& Ray::Direction() const {
    return direction;
  }

  float Ray::Time() const {
    return time;
  }

  Ref<Medium> Ray::GetMedium() const {
    return medium;
  }

  Point3 Ray::At(float t) const {
    return origin + t * direction;
  }

  Point3 Ray::operator()(float t) const {
    return At(t);
  }

  const bool RayDifferential::HasDifferentials() const {
    return has_differentials;
  }

  const Point3& RayDifferential::RxOrigin() const {
    return rx_origin;
  }

  const Point3& RayDifferential::RyOrigin() const {
    return ry_origin;
  }

  const glm::vec3& RayDifferential::RxDirection() const {
    return rx_dir;
  }

  const glm::vec3& RayDifferential::RyDirection() const {
    return ry_dir;
  }
      
  void RayDifferential::ScaleDifferentials(float s) {
    rx_origin = s * (rx_origin - origin);
    ry_origin = s * (ry_origin - origin);

    rx_dir = direction + s * (rx_dir - direction);
    ry_dir = direction + s * (ry_dir - direction);
  }

} // namespace pbr
