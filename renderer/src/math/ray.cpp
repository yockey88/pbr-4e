/**
 * \file math/ray.cpp
 **/
#include "math/ray.hpp"

namespace pbr {

  void Ray::SetOrigin(const Point3& o) {
    origin = o;
  }

  void Ray::SetDirection(const glm::vec3& dir) {
    direction = dir;
  }
  
  void Ray::SetTime(float time) {
    this->time = time;
  }

  void Ray::SetMedium(const Ref<Medium>& medium) {
    this->medium = medium;
  }

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

  void RayDifferential::SetRxOrigin(const Point3& o) {
    rx_origin = o;
  }

  void RayDifferential::SetRyOrigin(const Point3& o) {
    ry_origin = o;
  }

  void RayDifferential::SetRxDirection(const glm::vec3& dir) {
    rx_dir = dir;
  }

  void RayDifferential::SetRyDirection(const glm::vec3& dir) {
    ry_dir = dir;
  }
      
  void RayDifferential::SetHasDifferentials(bool has) {
    has_differentials = has;
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
