/**
 * \file util/triangle.hpp
 **/
#ifndef PBR_TRIANGLE_HPP
#define PBR_TRIANGLE_HPP

#include "geometry/shape.hpp"

namespace pbr {

  class Triangle : public Shape {
    public:
      virtual float Area() const override;
      virtual float PDF(const Interaction& interaction) const override;
      virtual float PDF(const ShapeSampleContext& ctx , const glm::vec3& wi) const override;

      virtual Bounds3 Bounds() const override;
      virtual DirectionCone NormalBounds() const override;

      virtual Opt<ShapeIntersection> Intersect(const Ray& ray , float tmax = infinity) const override;
      virtual Opt<ShapeSample> Sample(const Point2& u) const override;
      virtual Opt<ShapeSample> Sample(const ShapeSampleContext& ctx , const glm::vec2& u) const override;

    private:
  };  

} // namespace pbr

#endif // !PBR_TRIANGLE_HPP
