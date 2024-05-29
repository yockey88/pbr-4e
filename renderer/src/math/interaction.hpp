/**
 * \file math/interaction.hpp
 **/
#ifndef PBR_INTERACTION_HPP
#define PBR_INTERACTION_HPP

#include "base/medium.hpp"
#include "math/vecmath.hpp"

namespace pbr {

  class SurfaceInteraction;
  class MediumInteraction;
  
  class MediumInterface {};
  class PhaseFunction {};

  class Interaction {
    public:
      Interaction(const Point3& p , const glm::vec3& n , const Point2& uv , const glm::vec3& wo , float time) 
        : point(p) , normal(n) , uv(uv) , wo(Normalize(wo)) , time(time) {}
      Interaction(const Point3& p , const glm::vec3& wo , float time , Ref<Medium> medium)
        : point(p) , wo(wo) , time(time) , medium(medium) {}

      bool IsSurfaceInteraction() const;
      bool IsMediumInteraction() const;

      const SurfaceInteraction& AsSurface() const;
      SurfaceInteraction& AsSurface();
      
      const MediumInteraction& AsMedium() const;
      MediumInteraction& AsMedium();

      Point3 point;
      glm::vec3 normal;
      Point2 uv;
      glm::vec3 wo;
      float time;

      const MediumInterface* medium_interface = nullptr;
      Ref<Medium> medium = nullptr;
  };

  class SurfaceInteraction : public Interaction {
    public:
      SurfaceInteraction(const Point3& pi , const Point2& uv , const glm::vec3& wo , const glm::vec3& dpdu ,
                         const glm::vec3& dpdv , const glm::vec3& dndu , const glm::vec3& dndv , float time , 
                         bool flip_normal);

      void SetShadingGeometry(const glm::vec3& ns , const glm::vec3& dpdus , const glm::vec3& dpdvs ,
                              const glm::vec3& dndus , const glm::vec3& dndvs , bool orientation_authoritative);

      struct {
        glm::vec3 normal;
        glm::vec3 dpdu , dpdv;
        glm::vec3 dndu , dndv;
      } shading;

      glm::vec3 dpdu , dpdv;
      glm::vec3 dndu , dndv;

      int32_t face_idx = 0;
  };

  class MediumInteraction : public Interaction {
    public:
      MediumInteraction(const Point3& p , const glm::vec3& wo , float time , Ref<Medium> medium , PhaseFunction phase)
        : Interaction(p , wo , time , medium) , phase(phase) {}

      PhaseFunction phase;
  };

} // namespace pbr

#endif // !PBR_INTERACTION_HPP
