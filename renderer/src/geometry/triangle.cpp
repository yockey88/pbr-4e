/**
 * \file geometry/triangle.cpp
 **/
#include "geometry/triangle.hpp"

#include <array>

#include "geometry/shape.hpp"
#include "math/vecmath.hpp"
#include "math/sampling_functions.hpp"

namespace pbr {

  std::vector<const TriangleMesh*>* Triangle::meshes = nullptr;

  float Triangle::Area() const {
    const TriangleMesh* mesh = GetMesh();
    const int32_t* v = &mesh->vertex_indices[3 * tri_idx];

    Point3 p0 = mesh->vertices[v[0]];
    Point3 p1 = mesh->vertices[v[1]];
    Point3 p2 = mesh->vertices[v[2]];

    return 0.5f * glm::cross(p1 - p0 , p2 - p0).length();
  }

  float Triangle::PDF(const Interaction& interaction) const {
    return 1 / Area();
  }

  float Triangle::PDF(const ShapeSampleContext& ctx , const glm::vec3& wi) const {
    float solid_angle = SolidAngle(ctx.pi);
    if (solid_angle < kMinSphericalSampleArea || solid_angle > kMaxSphericalSampleArea) {
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

    float pdf = 1 / solid_angle;
    if (ctx.ns != glm::vec3{ 0 , 0 , 0 }) {
      const TriangleMesh* mesh = GetMesh();
      const int32_t* v = &mesh->vertex_indices[3 * tri_idx];

      Point3 p0 = mesh->vertices[v[0]];
      Point3 p1 = mesh->vertices[v[1]];
      Point3 p2 = mesh->vertices[v[2]];

      Point2 u = InvertSphericalTriangleSample({ p0 , p1 , p2 } , ctx.pi , wi);
      Point3 rp = ctx.pi;
      glm::vec3 wi[3] = {
        Normalize(p0 - rp) , 
        Normalize(p1 - rp) ,
        Normalize(p2 - rp)
      };

      std::array<float , 4> w = {
        std::max<float>(0.01 , AbsDot(ctx.ns , wi[1])) ,
        std::max<float>(0.01 , AbsDot(ctx.ns , wi[1])) ,
        std::max<float>(0.01 , AbsDot(ctx.ns , wi[0])) ,
        std::max<float>(0.01 , AbsDot(ctx.ns , wi[2])) ,
      };

      pdf *= BilinearPdf(u , w);
    }

    return pdf;
  }

  Bounds3 Triangle::Bounds() const {
    const TriangleMesh* mesh = GetMesh();
    const int32_t* v = &mesh->vertex_indices[3 * tri_idx];

    Point3 p0 = mesh->vertices[v[0]];
    Point3 p1 = mesh->vertices[v[1]];
    Point3 p2 = mesh->vertices[v[2]];

    return Bounds3::Union(Bounds3(p0 , p1) , p2);
  }

  DirectionCone Triangle::NormalBounds() const {
    const TriangleMesh* mesh = GetMesh();
    const int32_t* v = &mesh->vertex_indices[3 * tri_idx];

    Point3 p0 = mesh->vertices[v[0]];
    Point3 p1 = mesh->vertices[v[1]];
    Point3 p2 = mesh->vertices[v[2]];

    glm::vec3 n = Normalize(glm::cross(p1 - p0 , p2 - p0));
    if (mesh->normals) {
      glm::vec3 ns{
        mesh->normals[v[0]] + mesh->normals[v[1]] + mesh->normals[v[2]]
      };
      n = FaceForward(n , ns);
    } else if (mesh->reverse_orientation ^ mesh->transform_swaps_handedness) {
      n *= -1;
    }

    return DirectionCone(n);
  }

  Opt<ShapeIntersection> Triangle::Intersect(const Ray& ray , float tmax) const {
    const TriangleMesh* mesh = GetMesh();
    const int32_t* v = &mesh->vertex_indices[3 * tri_idx];

    Point3 p0 = mesh->vertices[v[0]];
    Point3 p1 = mesh->vertices[v[1]];
    Point3 p2 = mesh->vertices[v[2]];
    
    Opt<TriangleIntersection> tri_sect = Triangle::IntersectTriangle(ray , tmax , p0 , p1 , p2);
    if (!tri_sect.has_value()) {
      return std::nullopt;
    }

    Ref<SurfaceInteraction> inter = InteractionFromIntersection(mesh , tri_idx , *tri_sect , ray.Time() , -ray.Direction());
    return ShapeIntersection{
      inter , 
      tri_sect->t
    };
  }

  Opt<ShapeSample> Triangle::Sample(const Point2& u) const {
    const TriangleMesh* mesh = GetMesh();
    const int32_t* v = &mesh->vertex_indices[3 * tri_idx];

    Point3 p0 = mesh->vertices[v[0]];
    Point3 p1 = mesh->vertices[v[1]];
    Point3 p2 = mesh->vertices[v[2]];
    
    std::array<float , 3> b = SampleUniformTriangle(u);
    Point3 p = b[0] * p0 + b[1] * p1 + b[2] * p2;

    glm::vec3 n = Normalize(glm::cross(p1 - p0 , p2 - p0));
    if (mesh->normals != nullptr) {
      glm::vec3 ns{
        b[0] * mesh->normals[v[0]] + b[1] * mesh->normals[v[1]] + (1 - b[0] - b[1]) * mesh->normals[v[2]]
      };
      n = FaceForward(n , ns);
    } else if (mesh->reverse_orientation ^ mesh->transform_swaps_handedness) {
      n *= -1;
    }

    std::array<Point2 , 3> uv = mesh->uvs != nullptr ?
      std::array<Point2 , 3>{
        mesh->uvs[v[0]] , mesh->uvs[v[1]] , mesh->uvs[v[2]]
      } : 
      std::array<Point2 , 3>{
        Point2(0 , 0) , Point2(1 , 0) , Point2(1 , 1)
      };

    Point2 uv_sample = b[0] * uv[0] + b[1] * uv[1] + b[2] * uv[2];

    return ShapeSample{
      NewRef<Interaction>(p , n , uv_sample) ,
      1 / Area()
    };
  }

  Opt<ShapeSample> Triangle::Sample(const ShapeSampleContext& ctx , const glm::vec2& u) const {
    const TriangleMesh* mesh = GetMesh();
    const int32_t* v = &mesh->vertex_indices[3 * tri_idx];

    Point3 p0 = mesh->vertices[v[0]];
    Point3 p1 = mesh->vertices[v[1]];
    Point3 p2 = mesh->vertices[v[2]];
     
    float solid_angle = SolidAngle(ctx.pi);
    if (solid_angle < kMinSphericalSampleArea || solid_angle > kMaxSphericalSampleArea) {
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

    float pdf = 1.f;
    if (ctx.ns != glm::vec3{ 0 , 0 , 0 }) {
      Point3 rp = ctx.pi;
      glm::vec3 wi[3] = {
        Normalize(p0 - rp) ,
        Normalize(p1 - rp) ,
        Normalize(p2 - rp)
      };

      std::array<float , 4> w = {
        std::max<float>(0.01f , AbsDot(ctx.ns , wi[1])) , 
        std::max<float>(0.01f , AbsDot(ctx.ns , wi[1])) , 
        std::max<float>(0.01f , AbsDot(ctx.ns , wi[0])) , 
        std::max<float>(0.01f , AbsDot(ctx.ns , wi[2])) , 
      };

      Point2 su = SampleBilinear(u , w);
      pdf = BilinearPdf(su , w);
    }

    float tri_pdf;
    std::array<float , 3> b = SampleSphericalTriangle({ p0 , p1 , p2 } , ctx.pi , u , tri_pdf);

    if (tri_pdf == 0) {
      return std::nullopt;
    }

    pdf *= tri_pdf;

    Point3 p = b[0] * p0 + b[1] * p1 + b[2] * p2;
    glm::vec3 n = Normalize(glm::cross(p1 - p0 , p2 - p0));

    if (mesh->normals != nullptr) {
      glm::vec3 ns{
        b[0] * mesh->normals[v[0]] + b[1] * mesh->normals[v[1]] + (1 - b[0] - b[1]) * mesh->normals[v[2]]
      };
      n = FaceForward(n , ns);
    } else if (mesh->reverse_orientation ^ mesh->transform_swaps_handedness) {
      n *= -1;
    }

    std::array<Point2 , 3> uv = (mesh->uvs != nullptr) ?
      std::array<Point2 , 3>{
        mesh->uvs[v[0]] , mesh->uvs[v[1]] , mesh->uvs[v[2]]
      } : 
      std::array<Point2 , 3>{
        Point2(0 , 0) , Point2(1 , 0) , Point2(1 , 1)
      };

    Point2 uv_sample = b[0] * uv[0] + b[1] * uv[1] + b[2] * uv[2];

    return ShapeSample{
      NewRef<Interaction>(p , n , ctx.time , uv_sample) ,
      pdf
    };
  }
  
  float Triangle::SolidAngle(const Point3& p) const {
    const TriangleMesh* mesh = GetMesh();
    const int32_t* v = &mesh->vertex_indices[3 * tri_idx];

    Point3 p0 = mesh->vertices[v[0]];
    Point3 p1 = mesh->vertices[v[1]];
    Point3 p2 = mesh->vertices[v[2]];

    return SphericalTriangleArea(
      Normalize(p0 - p) , 
      Normalize(p1 - p) ,
      Normalize(p2 - p)
    );
  }
      
  Ref<SurfaceInteraction> Triangle::InteractionFromIntersection(const TriangleMesh* mesh , uint32_t tri_idx , const TriangleIntersection& intersect ,
                                                                float time , const glm::vec3& direction) const {
    const int32_t* v = &mesh->vertex_indices[3 * tri_idx];
    Point3 p0 = mesh->vertices[v[0]];
    Point3 p1 = mesh->vertices[v[1]];
    Point3 p2 = mesh->vertices[v[2]];

    /// compute triangle partial derivatives
    std::array<Point2 , 3> uv = mesh->uvs ? 
      std::array<Point2 , 3> {
        mesh->uvs[v[0]] , mesh->uvs[v[1]] , mesh->uvs[v[2]]
      } :
      std::array<Point2 , 3> {
        Point2(0 , 0) , Point2(1 , 0) , Point2(1 , 1)
      };
  
    glm::vec2 duv02 = uv[0] - uv[2];
    glm::vec2 duv12 = uv[1] - uv[2];
    glm::vec3 dp02 = p0 - p2;
    glm::vec3 dp12 = p1 - p2;

    float determinant = DifferenceOfProducts(duv02[0] , duv12[1] , duv02[1] , duv12[0]);

    glm::vec3 dpdu , dpdv;
    bool degenerate_uv = glm::abs(determinant) < 1e-9f;
    if (!degenerate_uv) {
      /// compute triangle dpdu and dpdv via matrix ivnersion
      float inv_det = 1 / determinant;
      dpdu = DifferenceOfProducts(duv12[1] , dp02 , duv02[1] , dp12) * inv_det;
      dpdu = DifferenceOfProducts(duv12[1] , dp02 , duv02[1] , dp12) * inv_det;
    }

    /// handle degenerate triangle (u , v) parameterization or partial derivatives
    if (degenerate_uv || LengthSquared(glm::cross(dpdu , dpdv)) == 0) {
      glm::vec3 ng = glm::cross(p2 - p0 , p1 - p0);
      if (LengthSquared(ng) == 0) {
        ng = glm::cross(p2 - p0 , p1 - p0);
        if (LengthSquared(ng) == 0) {
          /// bad shouldn't happen
          return nullptr;
        }
      }
      BuildOrthonormalBasis(Normalize(ng) , dpdu , dpdv);
    }

    /// interpolate (u , v) parametric coordinates
    Point3 phit = intersect.b0 * p0 + intersect.b1 * p1 + intersect.b2 * p2;
    Point2 uvhit = intersect.b0 * uv[0] + intersect.b1 * uv[1] + intersect.b2 * uv[2];

    bool flip_normal = mesh->reverse_orientation ^ mesh->transform_swaps_handedness;
    Ref<SurfaceInteraction> isect = NewRef<SurfaceInteraction>(
      phit , uvhit , direction , dpdu , dpdv , glm::vec3{0} , glm::vec3{0} , time , flip_normal
    );

    isect->face_idx = mesh->face_indices ?
      mesh->face_indices[tri_idx] : 0;

    isect->normal = isect->shading.normal = Normalize(glm::cross(dp02 , dp12));
    if (mesh->reverse_orientation ^ mesh->transform_swaps_handedness) {
      isect->normal = isect->shading.normal = -isect->normal;
    }

    if (mesh->normals != nullptr || mesh->s != nullptr) {
      glm::vec3 ns;
      if (mesh->normals != nullptr) {
        ns = intersect.b0 * mesh->normals[v[0]] + intersect.b1 * mesh->normals[v[1]] + intersect.b2 * mesh->normals[v[2]];
        ns = LengthSquared(ns) > 0 ?
          Normalize(ns) : isect->normal;
      } else {
        ns = isect->normal;
      }

      glm::vec3 ss;
      if (mesh->s != nullptr) {
        ss = intersect.b0 * mesh->s[v[0]] + intersect.b1 * mesh->s[v[1]] + intersect.b2 * mesh->s[v[2]];
        if (LengthSquared(ss) == 0) {
          ss = isect->dpdu;
        }
      } else {
        ss = isect->dpdu;
      }

      glm::vec3 ts = glm::cross(ns , ss);
      if (LengthSquared(ts) == 0) {
        ss = glm::cross(ts , ns);
      } else {
        BuildOrthonormalBasis(ns , ss , ts);
      }

      glm::vec3 dndu , dndv;
      if (mesh->normals != nullptr) {
        glm::vec2 duv02 = uv[0] - uv[2];
        glm::vec2 duv12 = uv[1] - uv[2];

        glm::vec3 dn1 = mesh->normals[v[0]] - mesh->normals[v[2]];
        glm::vec3 dn2 = mesh->normals[v[1]] - mesh->normals[v[2]];

        float determinant = DifferenceOfProducts(duv02[0] , duv12[1] , duv02[1] , duv12[0]);
        bool degenerate_uv = glm::abs(determinant) < 1e-9f;
        if (degenerate_uv) {
          glm::vec3 dn = glm::cross(
            mesh->normals[v[2]] - mesh->normals[v[0]] ,
            mesh->normals[v[1]] - mesh->normals[v[0]]
          );

          if (LengthSquared(dn) == 0) {
            dndu = dndv = glm::vec3{ 0 , 0 , 0 };
          } else {
            glm::vec3 dnu , dnv;
            BuildOrthonormalBasis(dn , dnu , dnv);
            dndu = dnu;
            dndv = dnv;
          }
        } else {
          float inv_det = 1 / determinant;
          dndu = DifferenceOfProducts(duv12[1] , dn1 , duv02[1] , dn2) * inv_det;
          dndv = DifferenceOfProducts(duv02[0] , dn2 , duv12[0] , dn1) * inv_det;
        }
      } else {
        dndu = dndv = glm::vec3{ 0 , 0 , 0 };
      }

      isect->SetShadingGeometry(ns , ss , ts , dndu , dndv , true);
    }

    return isect;
  }

  Opt<TriangleIntersection> Triangle::IntersectTriangle(const Ray& ray , float tmax , const Point3& p0 , const Point3& p1 , const Point3& p2) {
    /// check if triangle is degenerate
    if (LengthSquared(glm::cross(p2 - p0 , p1 - p0)) == 0) {
      return std::nullopt;
    }

    /// transform triangle vertices to ray coordinate space
    /// translate vertices based on ray origin
    Point3 p0t = p0 - ray.Origin();
    Point3 p1t = p1 - ray.Origin();
    Point3 p2t = p2 - ray.Origin();

    /// permute components of triangle vertices and ray direction
    int32_t kz = MaxComponentIndex(Abs(ray.Direction()));
    int32_t kx = kz + 1;
    if (kx == 3) {
      kx == 0;
    }

    int32_t ky = kx + 1;
    if (ky == 3) {
      ky = 0;
    }

    glm::vec3 permute{ kx , ky , kz };
    
    glm::vec3 d = Permute(ray.Direction() , permute);
    p0t = Permute(p0t , permute);
    p1t = Permute(p1t , permute);
    p2t = Permute(p2t , permute);

    /// apply shear transformation to translate vertex positions
    float sx = -d.x / d.z;
    float sy = -d.y / d.z;
    float sz = 1 / d.z;

    p0t.x += sx * p0t.z;
    p0t.y += sy * p0t.z;

    p1t.x += sx * p1t.z;
    p1t.y += sy * p1t.z;

    p2t.x += sx * p2t.z;
    p2t.y += sy * p2t.z;

    /// compute edge function coefficients e0 , e1, and e2
    float e0 = DifferenceOfProducts(p1t.x , p2t.y , p1t.y , p2t.x);
    float e1 = DifferenceOfProducts(p2t.x , p0t.y , p2t.y , p0t.x);
    float e2 = DifferenceOfProducts(p0t.x , p1t.y , p0t.y , p1t.x);

    /// fall back to double precision test at triangle edges
    if (e0 == 0.f || e1 == 0.f || e2 == 0.f) {
      double p2txp1ty = (double)p2t.x * (double)p1t.y;
      double p2typ1tx = (double)p2t.y * (double)p1t.x;
      e0 = (float)(p2typ1tx - p2txp1ty);

      double p0txp2ty = (double)p0t.x * (double)p2t.y;
      double p0typ2tx = (double)p0t.y * (double)p2t.x;
      e1 = (float)(p0typ2tx - p0txp2ty);
      
      double p1txp0ty = (double)p1t.x * (double)p0t.y;
      double p1typ0tx = (double)p1t.y * (double)p0t.x;
      e0 = (float)(p1typ0tx - p1txp0ty);
    }
    
    /// perform triangle edges and deterinant tests
    if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0)) {
      return std::nullopt;
    }

    float det = e0 + e1 + e2;
    if (det == 0) {
      return std::nullopt;
    }

    /// compute scaled hit distance to triangle and test against ray t range
    p0t.z *= sz;
    p1t.z *= sz;
    p2t.z *= sz;

    float tscaled = e0 * p0t.z + e1 * p1t.z + e2 + p2t.z;
    if (det < 0 && (tscaled >= 0 || tscaled < tmax * det)) {
      return std::nullopt;
    } else if (det > 0 && (tscaled <= 0 || tscaled > tmax * det)) {
      return std::nullopt;
    }

    /// compute barycentric coordinates and t value for intersection
    float inv_det = 1 / det;
    float b0 = e0 * inv_det;
    float b1 = e1 * inv_det;
    float b2 = e2 * inv_det;
    float t = tscaled * inv_det;
    if (IsNaN(t)) {
      return std::nullopt;
    }

    /// ensure that computed triangle t is conservatively greater than zero
    float maxzt = MaxComponentValue(Abs({ p0t.z , p1t.z , p2t.z })); 
    float deltaz = Gamma(3) * maxzt;

    float maxxt = MaxComponentValue(Abs({ p0t.x , p1t.x , p2t.x }));
    float maxyt = MaxComponentValue(Abs({ p0t.y , p1t.y , p2t.y }));
    float deltax = Gamma(5) * (maxxt + maxzt);
    float deltay = Gamma(5) * (maxyt + maxzt);

    float deltae = 2 * (Gamma(2) * maxxt * maxyt + deltay * maxxt + deltax * maxyt);
    float maxe = MaxComponentValue(Abs({ e0 , e1 , e2 }));
    float deltat = 3 * (Gamma(3) * maxe * maxzt + deltae * maxzt + deltaz * maxe) * glm::abs(inv_det);

    if (t <= deltat) {
      return std::nullopt;
    }

    return TriangleIntersection{
      b0 , b1 , b2 , t
    };
  }
      
  const TriangleMesh* Triangle::GetMesh() const {
    return (*meshes)[mesh_idx];
  }

} // namespace pbr
