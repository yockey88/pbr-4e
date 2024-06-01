/**
 * \file geometry/shape.hpp
 **/
#ifndef PBR_SHAPE_HPP
#define PBR_SHAPE_HPP

#include "util/defines.hpp"
#include "math/bounds.hpp"
#include "math/ray.hpp"
#include "math/direction_cone.hpp"
#include "geometry/interaction.hpp"

namespace pbr {

  struct ShapeIntersection {
    Ref<SurfaceInteraction> inter;
    float thit;
  };

  struct QuadraticIntersection {
    float thit;
    Point3 obj_point;
    float phi;
  };

  struct ShapeSample {
    Ref<Interaction> inter;
    float pdf;
  };

  struct ShapeSampleContext {
    ShapeSampleContext(const Point3& pi , const glm::vec3& n , const glm::vec3& ns , float time)
      : pi(pi) , n(n) , ns(ns) , time(time) {}
    ShapeSampleContext(const SurfaceInteraction& si)
      : pi(si.point) , n(si.normal) , ns(si.shading.normal) , time(si.time) {}
    ShapeSampleContext(const MediumInteraction& mi)
      : pi(mi.point) , time(mi.time) {}

    Point3 OffsetRayOriginVector(const glm::vec3& w) const;
    Point3 OffsetRayOriginPoint(const Point3& pt) const;
    Ray SpawnRay(const glm::vec3& w) const;

    Point3 pi;
    glm::vec3 n , ns;
    float time;
  };

  class Shape {
    public:
      virtual float Area() const = 0;
      virtual float PDF(const Interaction& interaction) const = 0;
      virtual float PDF(const ShapeSampleContext& ctx , const glm::vec3& wi) const = 0;

      virtual Bounds3 Bounds() const = 0;
      virtual DirectionCone NormalBounds() const = 0;

      virtual Opt<ShapeIntersection> Intersect(const Ray& ray , float tmax = infinity) const = 0;
      virtual Opt<ShapeSample> Sample(const Point2& u) const = 0;
      virtual Opt<ShapeSample> Sample(const ShapeSampleContext& ctx , const glm::vec2& u) const = 0;
      
      virtual bool IntersectP(const Ray& ray , float tmax = infinity) const;

    protected:
  }; 

  class QuadraticShape : public Shape {
    public:
      Opt<ShapeIntersection> Intersect(const Ray& ray , float tmax = infinity) const override;

      virtual Opt<QuadraticIntersection> BasicIntersect(const Ray& r , float tmax) const = 0;
      virtual Ref<SurfaceInteraction> InteractionFromIntersection(const QuadraticIntersection& isect , const glm::vec3& wo , float time) const = 0;

    private:
  };

} // namespace pbr

#endif // !PBR_SHAPE_HPP
