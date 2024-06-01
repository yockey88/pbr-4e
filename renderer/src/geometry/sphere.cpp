/**
 * \file geometry/sphere.cpp
 **/
#include "geometry/sphere.hpp"

#include "geometry/shape.hpp"
#include "math/math.hpp"
#include "math/vecmath.hpp"
#include "math/sampling_functions.hpp"

namespace pbr {

  Sphere::Sphere(const Transform* render_from_object , const Transform* object_from_render ,
                 bool reverse_orientation , float radius , float zmin , float zmax , float phi_max)
      : render_from_object(render_from_object) , object_from_render(object_from_render) , reverse_orientation(reverse_orientation) ,
        transform_swaps_handedness(render_from_object->SwapsHandedness()) , radius(radius) , 
        zmin(Clamp(glm::min(zmin , zmax) , -radius , radius)) , zmax(Clamp(glm::max(zmin , zmax) , -radius , radius)) ,
        theta_zmin(glm::acos(Clamp(glm::min(zmin , zmax) / radius , -1 , 1))) , theta_zmax(glm::acos(Clamp(glm::max(zmin , zmax) / radius , -1 , 1))) ,
        phi_max(glm::radians(Clamp(phi_max , 0 , 360))) {}

  float Sphere::Area() const {
    return phi_max * radius * (zmax - zmin);
  }
      
  float Sphere::PDF(const Interaction& interaction) const {
    return 1 / Area();
  }
      
  float Sphere::PDF(const ShapeSampleContext& ctx , const glm::vec3& wi) const {
    Point3 center = render_from_object->TransformPoint({ 0 , 0 , 0 });
    Point3 origin = ctx.OffsetRayOriginPoint(center);

    if (DistanceSquared(origin , center) <= glm::pow(radius , 2)) {
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

    float sin_2_theta_max = radius * radius / DistanceSquared(ctx.pi , center);
    float cos_theta_max = glm::sqrt(1 - sin_2_theta_max);
    float one_minus_cos_theta_max = 1 - cos_theta_max;
    if (sin_2_theta_max < 0.00068523f /* sin^2(1.5 deg) */) {
      one_minus_cos_theta_max = sin_2_theta_max / 2;
    }

    return 1 / (2 * pi * one_minus_cos_theta_max);
  }
  
  Bounds3 Sphere::Bounds() const {
    return render_from_object->TransformBounds(
      Bounds3(
        Point3(-radius , -radius , zmin) ,
        Point3(radius , radius , zmax)
      )
    );
  }
      
  DirectionCone Sphere::NormalBounds() const {
    return DirectionCone::EntireSphere();
  }
      
  Opt<ShapeSample> Sphere::Sample(const Point2& u) const {
    Point3 pobj = Point3{ 0 , 0 , 0 } + radius * SampleUniformSphere(u);
    pobj *= radius / Distance(pobj , { 0 , 0 , 0 });

    glm::vec3 nobj{ pobj.x , pobj.y , pobj.z };
    glm::vec3 n = Normalize(render_from_object->TransformNormal(nobj));
    if (reverse_orientation) {
      n *= -1;
    }

    float theta = SafeAcos(pobj.z / radius);
    float phi = std::atan2(pobj.y , pobj.x);
    if (phi < 0) {
      phi += 2 * pi;
    }

    Point2 uv{
      phi / phi_max ,
      (theta - theta_zmin) / (theta_zmax - theta_zmin)
    };

    Point3 point = render_from_object->TransformPoint(pobj);
    return ShapeSample{
      NewRef<Interaction>(point , n , uv) , 
      1 / Area()
    };
  }
      
  Opt<ShapeSample> Sphere::Sample(const ShapeSampleContext& ctx , const glm::vec2& u) const {
    Point3 center = render_from_object->TransformPoint({ 0 , 0 , 0 });
    Point3 origin = ctx.OffsetRayOriginPoint(center);
    if (DistanceSquared(origin , center) <= glm::pow(radius , 2)) {
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

    float sin_theta_max = radius / Distance(ctx.pi , center);
    float sin_2_theta_max = glm::pow(sin_theta_max , 2);
    float cos_theta_max = glm::sqrt(1 - sin_2_theta_max);
    float one_minus_cos_theta_max = 1 - cos_theta_max;

    float cos_theta = (cos_theta_max - 1) * u[0] + 1;
    float sin_2_theta = 1 - glm::pow(cos_theta , 2);

    if (sin_2_theta_max < 0.00068523f /* sin^2(1.f deg) */) {
      sin_2_theta = sin_2_theta_max * u[0];
      cos_theta = glm::sqrt(1 - sin_2_theta);
      one_minus_cos_theta_max = sin_2_theta_max / 2;
    }

    float cos_alpha = sin_2_theta / sin_theta_max + 
                      cos_theta * glm::pow(1 - sin_2_theta / glm::pow(sin_theta_max , 2) , 2);
    float sin_alpha = glm::sqrt(1 - glm::pow(cos_alpha , 2));

    float phi = u[1] * 2 * pi;
    glm::vec3 w = SphericalDirection(sin_alpha , cos_alpha , phi);
    Frame sampling_frame = Frame::FromZ(Normalize(center - ctx.pi));
    glm::vec3 n = sampling_frame.Local(-w);
    Point3 p = center + radius * Point3(n.x , n.y , n.z);
    if (reverse_orientation) {
      n *= -1;
    }

    Point3 pobj = object_from_render->TransformPoint(p);
    float theta = SafeAcos(pobj.z / radius);
    float sphere_phi = std::atan2(pobj.y , pobj.x);
    if (sphere_phi < 0) {
      sphere_phi += 2 * pi;
    }

    Point2 uv{
      sphere_phi / phi_max ,
      (theta - theta_zmin) / (theta_zmax - theta_zmin)
    };

    return ShapeSample{
      NewRef<Interaction>(p , n , ctx.time , uv) ,
      1 / (2 * pi * one_minus_cos_theta_max)
    };
  }
      
  Opt<QuadraticIntersection> Sphere::BasicIntersect(const Ray& r , float tmax) const {
    float phi;
    Point3 phit;

    /// transform ray origin and direction to object space
    Point3 oi = object_from_render->TransformPoint(r.Origin());
    glm::vec3 di = object_from_render->TransformVector(r.Direction());
    
    float t0 , t1;

    /// solve quadratic eqn to compute t0 and t1
    float a = LengthSquared(di);
    float b = 2 * glm::dot(di , oi);
    float c = LengthSquared(oi) - glm::pow(radius , 2);

    glm::vec3 v(oi - b / (2 * a) * di);
    float l = v.length();
    float discriminant = 4 * a * (radius + l) * (radius - 1);

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

    /// compute sphere hit position and phi
    phit = oi + shape_hit + di;
    phit *= radius / Distance(phit , { 0 , 0 , 0 });

    if (phit.x == 0 && phit.y == 0) {
      phit.x = 1e-5f * radius;
    }

    phi = std::atan2(phit.y , phit.x);
    if (phi < 0) {
      phi += 2.f * pi;
    }

    /// test sphere intersection against clipping parameters
    if ((zmin > -radius && phit.z < zmin) || (zmax < radius && phit.z > zmax) || phi > phi_max) {
      if (shape_hit == t1) {
        return std::nullopt;
      }

      if (t1 > tmax) {
        return std::nullopt;
      }
      shape_hit = t1;

      phit = oi + shape_hit * di;
      phit *= radius / Distance(phit , { 0 , 0 , 0 });

      if (phit.x == 0 && phit.y == 0) {
        phit.x = 1e-5f * radius;
      }

      phi = std::atan2(phit.y , phit.x);
      if (phi < 0) {
        phi += 2.f * pi;
      }

      if ((zmin > - radius && phit.z < zmin) || (zmax < radius && phit.z > zmax) || phi > phi_max) {
        return std::nullopt;
      }
    }

    return QuadraticIntersection(shape_hit , phit , phi);
  }
      
  Ref<SurfaceInteraction> Sphere::InteractionFromIntersection(const QuadraticIntersection& isect , const glm::vec3& wo , float time) const {
    Point3 phit = isect.obj_point;
    float phi = isect.phi;

    /// find parametric representation of sphere hit
    float u = phi / phi_max;
    float cos_theta = phit.z / radius;
    float theta = SafeAcos(cos_theta);
    float v = (theta - theta_zmin) / (theta_zmax - theta_zmin);

    /// compute sphere dp/du and dp/dv
    float zradius = glm::sqrt(glm::pow(phit.x , 2) + glm::pow(phit.y , 2));
    float cos_phi = phit.x / zradius;
    float sin_phi = phit.y / zradius;

    glm::vec3 dpdu{ -phi_max * phit.y , phi_max * phit.x , 0 };
    float sin_theta = glm::sqrt(1 - glm::pow(cos_theta , 2));

    glm::vec3 dpdv = (theta_zmax - theta_zmin) * glm::vec3{
      phit.x * cos_phi , phit.z * sin_phi , -radius * sin_theta
    };

    /// compute sphere dn/du and dn/dv
    glm::vec3 d2Pduu = -phi_max * phi_max * glm::vec3{ phit.x , phit.y , 0 };
    glm::vec3 d2Pduv = (theta_zmax - theta_zmin) * phit.z * phi_max * glm::vec3{
      -sin_phi , cos_phi , 0
    };

    glm::vec3 d2Pdvv = -glm::pow(theta_zmax - theta_zmin , 2) * glm::vec3{ phit.x , phit.y , phit.z };
    float E = glm::dot(dpdu , dpdu) , F = glm::dot(dpdu , dpdv) , G = glm::dot(dpdv , dpdv);
    glm::vec3 n = Normalize(glm::cross(dpdu , dpdv));
    float e = glm::dot(n , d2Pduu) , f = glm::dot(n , d2Pduv) , g = glm::dot(n , d2Pdvv);

    float EGF2 = DifferenceOfProducts(E , G , F , F);
    float inv_EGF2 = (EGF2 == 0) ? 
      0 : 1 / EGF2;

    glm::vec3 dndu = glm::vec3((f * F - e * G) * inv_EGF2 * dpdu + 
                               (e * F - f * E) * inv_EGF2 * dpdv);
    glm::vec3 dndv = glm::vec3((g * F - f * G) * inv_EGF2 * dpdu + 
                               (f * F - g * E) * inv_EGF2 * dpdv);

    bool flip_normal = reverse_orientation ^ transform_swaps_handedness;
    glm::vec3 wo_obj = object_from_render->TransformVector(wo);
    return render_from_object->TransformSurfaceInteraction(
      NewRef<SurfaceInteraction>(phit , Point2(u , v) , wo_obj , dpdu , dpdv , dndu , dndv , time , flip_normal)
    );
  }

} // namespace pbr
