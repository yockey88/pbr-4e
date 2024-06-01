/**
 * \file geometry/disk.hpp
 **/
#ifndef PBR_DISK_HPP
#define PBR_DISK_HPP

#include "math/transform.hpp"
#include "geometry/shape.hpp"

namespace pbr {

  class Disk : public QuadraticShape {
    public:
      Disk(const Transform* render_from_object , const Transform* object_from_render , bool reverse_orientation ,
           float height , float radius , float inner_radius);

      virtual float Area() const override;
      virtual float PDF(const Interaction& interaction) const override;
      virtual float PDF(const ShapeSampleContext& ctx , const glm::vec3& wi) const override;

      virtual Bounds3 Bounds() const override;
      virtual DirectionCone NormalBounds() const override;

      virtual Opt<ShapeSample> Sample(const Point2& u) const override;
      virtual Opt<ShapeSample> Sample(const ShapeSampleContext& ctx , const glm::vec2& u) const override;

      virtual bool IntersectP(const Ray& ray , float tmax = infinity) const override;

      virtual Opt<QuadraticIntersection> BasicIntersect(const Ray& r , float tmax) const override;
      virtual Ref<SurfaceInteraction> InteractionFromIntersection(const QuadraticIntersection& isect , const glm::vec3& wo , float time) const override;

    private:
      const Transform* render_from_object;
      const Transform* object_from_render;
      bool reverse_orientation , transform_swaps_handedness;
      float height , radius , inner_radius , phi_max;
  };

} // namespace pbr

#endif // !PBR_DISK_HPP
