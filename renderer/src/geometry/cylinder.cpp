/**
 * \file geometry/cylinder.cpp
 **/
#include "geometry/cylinder.hpp"
#include "geometry/shape.hpp"
#include "math/direction_cone.hpp"
#include "math/vecmath.hpp"

namespace pbr {

  Cylinder::Cylinder(const Transform* render_from_object , const Transform* object_from_render , bool reverse_orientation , 
                     float radius , float zmin , float zmax , float phi_max)
      : render_from_object(render_from_object) , object_from_render(object_from_render) , reverse_orientation(reverse_orientation) ,
        transform_swaps_handedness(render_from_object->SwapsHandedness()) , radius(radius) , zmin(glm::min(zmin , zmax)) ,
        zmax(glm::max(zmin , zmax)) , phi_max(glm::radians(Clamp(phi_max , 0 , 360))) {}

  float Cylinder::Area() const {
    return (zmax - zmin) * radius * phi_max;
  }

  float Cylinder::PDF(const Interaction& interaction) const  {
    return 1 / Area();
  }

  float Cylinder::PDF(const ShapeSampleContext& ctx , const glm::vec3& wi) const  {
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

  Bounds3 Cylinder::Bounds() const {
    return render_from_object->TransformBounds(
      Bounds3{
        { -radius , -radius , zmin } ,
        { radius , radius , zmax }
      }
    );
  }

  DirectionCone Cylinder::NormalBounds() const {
    return DirectionCone::EntireSphere();
  }

  Opt<ShapeSample> Cylinder::Sample(const Point2& u) const {
    float z = Lerp(u[0] , zmin , zmax);
    float phi = u[1] * phi_max;
  
    /// compute cylinder sample position and normal from z and phi
    Point3 pobj{
      radius * glm::cos(phi) , 
      radius * glm::sin(phi) , 
      z
    };

    Point3 pi = render_from_object->TransformPoint(pobj);
    glm::vec3 n = Normalize(render_from_object->TransformNormal({ pobj.x , pobj.y , 0 }));
    if (reverse_orientation) {
      n *= -1;
    }

    Point2 uv{
      phi / phi_max ,
      (pobj.z - zmin) / (zmax - zmin)
    };
    return ShapeSample{
      NewRef<Interaction>(pi , n , uv),
      1 / Area()
    };
  }

  Opt<ShapeSample> Cylinder::Sample(const ShapeSampleContext& ctx , const glm::vec2& u) const {
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
      
  bool Cylinder::IntersectP(const Ray& ray , float tmax) const {
    return BasicIntersect(ray , tmax).has_value();
  }

  Opt<QuadraticIntersection> Cylinder::BasicIntersect(const Ray& ray , float tmax) const {
    float phi;
    Point3 phit;

    /// transform ray origin and direction to object space
    Point3 oi = object_from_render->TransformPoint(ray.Origin());
    glm::vec3 di = object_from_render->TransformVector(ray.Direction());

    /// solve quadratic eqn for t0 and t1 values
    float t0 , t1;

    float a = glm::pow(di.x , 2) + glm::pow(di.y , 2);
    float b = 2 * (di.x * oi.x + di.y * oi.y);
    float c = glm::pow(oi.x , 2) + glm::pow(oi.y , 2) - glm::pow(radius , 2);

    /// check quadric shape to and t1 for nearest intersection
    float f = b / (2 * a);
    float vx = oi.x - f * di.x;
    float vy = oi.y - f * di.y;
    float len = glm::sqrt(glm::pow(vx , 2) + glm::pow(vy , 2));
    float discriminant = 4 * a * (radius + len) * (radius - len);
    if (discriminant < 0) {
      return std::nullopt;
    }
    
    float sqrt_discrim = glm::sqrt(discriminant);
    float q;
    if (b < 0) {
      q = -0.5f * (b - sqrt_discrim);
    } else {
      q = 0.5f * (b + sqrt_discrim); 
    }

    t0 = q / a;
    t1 = c / q;

    if (t0 > t1) {
      std::swap(t0 , t1);
    }

    if (t0 > tmax || t1 <= 0) {
      return std::nullopt;
    }

    /// check quadratic shape t0 and t1 for nearest intersection
    float shape_hit = t0;

    if (shape_hit <= 0) {
      shape_hit = t1;
      if (shape_hit > tmax) {
        return std::nullopt;
      }
    }
    
    /// compute cylinder hit point and phi
    phit = oi + shape_hit * di;
    float hit_rad = glm::sqrt(glm::pow(phit.x , 2) + glm::pow(phit.y , 2));
    phit.x *= radius / hit_rad;
    phit.y *= radius / hit_rad;

    phi = std::atan2(phit.y , phit.x);
    if (phi < 0) {
      phi += 2 * pi;
    }

    /// test sphere intersection against clipping parameters
    if (phit.z < zmin || phit.z > zmax || phi > phi_max) {
      if (shape_hit == t1) {
        return std::nullopt;
      }

      shape_hit = t1;
      if (t1 > tmax) {
        return std::nullopt;
      }

      phit = oi + shape_hit * di;
      float hit_rad = glm::sqrt(glm::pow(phit.x , 2) + glm::pow(phit.y , 2));
      phit.x *= radius / hit_rad;
      phit.y *= radius / hit_rad;

      phi = std::atan2(phit.y , phit.x);
      if (phi < 0) {
        phi += 2 * pi;
      }

      if (phit.z < zmin || phit.z > zmax || phi > phi_max) {
        return std::nullopt;
      }
    }

    return QuadraticIntersection{
      shape_hit , phit , phi
    };
  }

  Ref<SurfaceInteraction> Cylinder::InteractionFromIntersection(const QuadraticIntersection& isect , const glm::vec3& wo , float time) const {
    Point3 phit = isect.obj_point;
    float phi = isect.phi;

    /// find parametric representation of cylinder hit
    float u = phi / phi_max;
    float v = (phit.z - zmin) / (zmax - zmin);

    /// compute dpdu and dpdv
    glm::vec3 dpdu{ 
      -phi_max * phit.y , phi_max * phit.x , 0
    };
    glm::vec3 dpdv{
      0 , 0 , zmax - zmin
    };


    /// compute dndu and dndv
    glm::vec3 d2Pduu = -phi_max * phi_max * glm::vec3(phit.x , phit.y , 0);
    glm::vec3 d2Pduv{ 0 , 0 , 0 };
    glm::vec3 d2Pdvv{ 0 , 0 , 0 };

    /// compute coefficients of fundumental forms
    float E = glm::dot(dpdu , dpdu) , F = glm::dot(dpdu , dpdv) , G = glm::dot(dpdv , dpdv);
    glm::vec3 n = Normalize(glm::cross(dpdu , dpdv));
    float e = glm::dot(n , d2Pduu) , f = glm::dot(n , d2Pduv) , g = glm::dot(n , d2Pdvv);

    /// compute dndu and dndv from fundamental form coefficients
    float EGF2 = DifferenceOfProducts(E , G , F , F);
    float inv_EGF2 = (EGF2 == 0) ?
      0 : 1 / EGF2;

    glm::vec3 dndu = (f * F - e * G) * inv_EGF2 * dpdu + (e * F - f * E) * inv_EGF2 * dpdv;
    glm::vec3 dndv = (g * F - f * G) * inv_EGF2 * dpdu + (f * F - g * E) * inv_EGF2 * dpdv;

    bool flip_normal = reverse_orientation ^ transform_swaps_handedness;
    glm::vec3 wo_obj = object_from_render->TransformVector(wo);
    return render_from_object->TransformSurfaceInteraction(
      NewRef<SurfaceInteraction>(
        phit , Point2(u , v) , wo_obj , dpdu , dpdv , dndu , dndv , time , flip_normal
      )
    );

    return nullptr;
  }

} // namespace pbr
