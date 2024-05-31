/**
 * \file geometry/interaction.cpp
 **/
#include "geometry/interaction.hpp"

#include "math/vecmath.hpp"

namespace pbr {

  bool Interaction::IsSurfaceInteraction() const {
    return normal != glm::vec3(0 , 0 , 0);
  }

  bool Interaction::IsMediumInteraction() const {
    return !IsSurfaceInteraction();
  }

  const SurfaceInteraction& Interaction::AsSurface() const {
    return (const SurfaceInteraction&)*this;
  }

  SurfaceInteraction& Interaction::AsSurface() {
    return (SurfaceInteraction&)*this;
  }
  
  const MediumInteraction& Interaction::AsMedium() const {
    return (const MediumInteraction&)*this;
  }

  MediumInteraction& Interaction::AsMedium() {
    return (MediumInteraction&)*this;
  }

  SurfaceInteraction::SurfaceInteraction(const Point3& pi , const Point2& uv , const glm::vec3& wo , const glm::vec3& dpdu ,
                                         const glm::vec3& dpdv , const glm::vec3& dndu , const glm::vec3& dndv , float time ,
                                         bool flip_normal)
      : Interaction(pi , glm::vec3(Normalize(glm::cross(dpdu , dpdv))) , uv , wo , time) ,
        dpdu(dpdu) , dpdv(dpdv) , dndu(dndu) , dndv(dndv) {
    shading.normal = normal; 
    shading.dpdu = dpdu;
    shading.dpdv = dpdv;
    shading.dndu = dndu;
    shading.dndv = dndv;

    if (flip_normal) {
      normal *= -1;
      shading.normal *= -1;
    }
  }
      
  void SurfaceInteraction::SetShadingGeometry(const glm::vec3& ns , const glm::vec3& dpdus , const glm::vec3& dpdvs ,
                                              const glm::vec3& dndus , const glm::vec3& dndvs , bool orientation_authoritative) {
    shading.normal = ns;

    if (orientation_authoritative) {
      normal = FaceForward(normal , shading.normal);
    } else {
      shading.normal = FaceForward(shading.normal , normal);
    }

    shading.dpdu = dpdus;
    shading.dpdv = dpdvs;
    shading.dndu = dndus;
    shading.dndv = dndvs;
  }

} // namespace pbr
