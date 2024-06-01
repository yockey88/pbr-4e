/**
 * \file goemetry/disk.cpp
 **/
#include "geometry/disk.hpp"
#include "geometry/shape.hpp"
#include "math/sampling_functions.hpp"
#include "math/vecmath.hpp"

namespace pbr {

  Disk::Disk(const Transform* render_from_object , const Transform* object_from_render , bool reverse_orientation ,
             float height , float radius , float inner_radius)
      : render_from_object(render_from_object) , object_from_render(object_from_render) , reverse_orientation(reverse_orientation) ,
        transform_swaps_handedness(render_from_object->SwapsHandedness()) , height(height) , radius(radius) , inner_radius(inner_radius) {
  }

  float Disk::Area() const {
    return phi_max * 0.5f * (glm::pow(radius , 2) - glm::pow(inner_radius , 2));
  }

  float Disk::PDF(const Interaction& interaction) const {
    return 1 / Area();
  }

  float Disk::PDF(const ShapeSampleContext& ctx , const glm::vec3& wi) const {
    Ray ray = ctx.SpawnRay(wi);
    Opt<ShapeIntersection> isect = Intersect(ray);
    if (!isect.has_value()) {
      return 0.f;
    }

    float pdf = (1 / Area()) / (AbsDot(isect->inter->normal , -wi) / 
                                DistanceSquared(ctx.pi , isect->inter->point));
    if (IsInf(pdf)) {
      return 0.f;
    }

    return pdf;
  }

  Bounds3 Disk::Bounds() const {
    return render_from_object->TransformBounds(
      Bounds3{
        { -radius , -radius , height } ,
        {  radius ,  radius , height }
      }
    );
  }

  DirectionCone Disk::NormalBounds() const {
    glm::vec3 n = render_from_object->TransformNormal({ 0 , 0 , 1 });
    if (reverse_orientation) {
      n = -n;
    }

    return DirectionCone(n);
  }

  Opt<ShapeSample> Disk::Sample(const Point2& u) const  {
    Point2 pd = SampleUniformDiskConcentric(u);
    Point3 pobj{ pd.x * radius , pd.y * radius , height };
    Point3 point = render_from_object->TransformPoint(pobj);
    glm::vec3 n = Normalize(render_from_object->TransformNormal({ 0 , 0 , 1 }));

    if (reverse_orientation) {
      n *= -1;
    }

    float phi = std::atan2(pd.y , pd.x);
    if (phi < 0) {
      phi += 2 * pi;
    }

    float radius_sample = glm::sqrt(glm::pow(pobj.x , 2) + glm::pow(pobj.y , 2));
    Point2 uv{ phi / phi_max , (radius - radius_sample) / (radius - inner_radius) };
    return ShapeSample {
      NewRef<Interaction>(point , n , uv) ,
      1 / Area()
    };
  }

  Opt<ShapeSample> Disk::Sample(const ShapeSampleContext& ctx , const glm::vec2& u) const  {
    Opt<ShapeSample> ss = Sample(u);
    if (!ss.has_value()) {
      return std::nullopt;
    }

    ss->inter->time = ctx.time;
    glm::vec3 wi = ss->inter->point - ctx.pi;
    if (LengthSquared(wi) == 0) {
      return std::nullopt;
    }

    wi = Normalize(wi);

    ss->pdf /= AbsDot(ss->inter->normal , -wi) / DistanceSquared(ctx.pi , ss->inter->point);
    if (IsInf(ss->pdf)) {
      return std::nullopt;
    }

    return ss;
  }
      
  bool Disk::IntersectP(const Ray& ray , float tmax) const {
    return BasicIntersect(ray , tmax).has_value();
  }
      
  Opt<QuadraticIntersection> Disk::BasicIntersect(const Ray& r , float tmax) const {
    /// transform ray origin and direction to object space
    Point3 oi = object_from_render->TransformPoint(r.Origin());
    glm::vec3 di = object_from_render->TransformVector(r.Direction());

    /// compute plane intersection for disk
    if (di.z == 0) {
      return std::nullopt;
    } 

    float shape_hit = (height - oi.z) / di.z;
    if (shape_hit <= 0 || shape_hit >= tmax) {
      return std::nullopt;
    }

    /// see if hit point is inside disk radii and phi_max
    Point3 phit = oi + shape_hit * di;
    float dist2 = glm::pow(phit.x , 2) + glm::pow(phit.y , 2);
    if (dist2 > glm::pow(radius , 2) || dist2 < glm::pow(inner_radius , 2)) {
      return std::nullopt;
    }

    /// test disk phi value against phi_max
    float phi = std::atan2(phit.y , phit.x);
    if (phi < 0) {
      phi += 2 * pi;
    }

    if (phi > phi_max) {
      return std::nullopt;
    }

    return QuadraticIntersection{
      shape_hit , phit , phi
    };
  }
      
  Ref<SurfaceInteraction> Disk::InteractionFromIntersection(const QuadraticIntersection& isect , const glm::vec3& wo , float time) const {
    Point3 phit = isect.obj_point;
    float phi = isect.phi;

    /// find parametric representation of disk hit
    float u = phi / phi_max;
    float rhit = glm::sqrt(glm::pow(phit.x , 2) + glm::pow(phit.y , 2));
    float v = (radius - rhit) / (radius - inner_radius);
    
    glm::vec3 dpdu {
      -phi_max * phit.y , phi_max * phit.x , 0
    };
    glm::vec3 dpdv = glm::vec3{ phit.x , phit.y , 0 } * (inner_radius - radius) / rhit; 

    glm::vec3 dndu{ 0 , 0 , 0 } , dndv{ 0 , 0 , 0 };

    /// refine disk intersection point
    phit.z = height;

    bool flip_normal = reverse_orientation ^ transform_swaps_handedness;
    glm::vec3 wo_obj = object_from_render->TransformVector(wo);
    return render_from_object->TransformSurfaceInteraction(
      NewRef<SurfaceInteraction>(phit , Point2(u , v) , wo_obj , dpdu , dpdv , dndu , dndv , time , flip_normal)
    );
  }

} // namespace pbr
