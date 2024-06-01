/**
 * \file util/triangle.hpp
 **/
#ifndef PBR_TRIANGLE_HPP
#define PBR_TRIANGLE_HPP

#include "util/mesh.hpp"
#include "geometry/shape.hpp"

namespace pbr {

  struct TriangleIntersection {
    float b0 , b1 , b2;
    float t;
  };

  class Triangle : public Shape {
    public:
      Triangle(int32_t mesh_idx , int32_t tri_idx)
        : mesh_idx(mesh_idx) , tri_idx(tri_idx) {}

      virtual float Area() const override;
      virtual float PDF(const Interaction& interaction) const override;
      virtual float PDF(const ShapeSampleContext& ctx , const glm::vec3& wi) const override;

      virtual Bounds3 Bounds() const override;
      virtual DirectionCone NormalBounds() const override;

      virtual Opt<ShapeIntersection> Intersect(const Ray& ray , float tmax = infinity) const override;
      virtual Opt<ShapeSample> Sample(const Point2& u) const override;
      virtual Opt<ShapeSample> Sample(const ShapeSampleContext& ctx , const glm::vec2& u) const override;

      float SolidAngle(const Point3& p) const;

      Ref<SurfaceInteraction> InteractionFromIntersection(const TriangleMesh* mesh , uint32_t tri_idx , const TriangleIntersection& intersect ,
                                                          float time , const glm::vec3& direction) const;
      
      static Opt<TriangleIntersection> IntersectTriangle(const Ray& ray , float tmax , const Point3& p0 , const Point3& p1 , const Point3& p2);

    private:
      int32_t mesh_idx = -1 , tri_idx = -1;
      static std::vector<const TriangleMesh*>* meshes;

      constexpr static float kMinSphericalSampleArea = 3e-4f;
      constexpr static float kMaxSphericalSampleArea = 6.22f;

      const TriangleMesh* GetMesh() const;
  };  

} // namespace pbr

#endif // !PBR_TRIANGLE_HPP
