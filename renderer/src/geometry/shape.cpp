/**
 * \file geometry/shape.cpp
 **/
#include "geometry/shape.hpp"

#include "math/vecmath.hpp"
#include "util/float.hpp"

namespace pbr {

  Point3 ShapeSampleContext::OffsetRayOriginVector(const glm::vec3& w) const {
    /// this is not correct?
    float d = glm::dot(Abs(n) , pi);
    glm::vec3 offset = d * n;

    if (glm::dot(w , n) < 0) {
      offset = -offset;  
    }

    Point3 po = pi + offset;

    for (int32_t i = 0; i < 3; ++i) {
      if (offset[i] > 0) {
        po[i] = NextFloatUp(po[i]);
      } else if (offset[i] < 0) {
        po[i] = NextFloatDown(po[i]);
      }
    }

    return pi;
  }

  Point3 ShapeSampleContext::OffsetRayOriginPoint(const Point3& p) const {
    return OffsetRayOriginVector(p - pi);
  }

  Ray ShapeSampleContext::SpawnRay(const glm::vec3& w) const {
    return Ray(OffsetRayOriginVector(w) , w , time);
  }
     
  bool Shape::IntersectP(const Ray& r , float tmax) const {
    return false;
  }

  Opt<ShapeIntersection> QuadraticShape::Intersect(const Ray& ray , float tmax) const {
    Opt<QuadraticIntersection> isect = BasicIntersect(ray , tmax);
    if (!isect.has_value()) {
      return std::nullopt;
    }
    Ref<SurfaceInteraction> inter = InteractionFromIntersection(*isect , -ray.Direction() , ray.Time());
    return ShapeIntersection{
      inter , isect->thit
    };
  }
  
} // namespace pbr
