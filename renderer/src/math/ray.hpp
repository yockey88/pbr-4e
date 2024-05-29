/**
 * \file math/ray.hpp
 **/
#ifndef PBR_RAY_HPP
#define PBR_RAY_HPP

#include "base/medium.hpp"
#include "math/vecmath.hpp"

namespace pbr {

  class Ray {
    public:
      Ray()
        : origin({ 0 , 0 , 0 }) , direction({ 0 , 0 , 0 }) , time(0) , medium(nullptr) {}
      Ray(const Point3& origin , const glm::vec3& direction , float time , Ref<Medium> medium = nullptr)
        : origin(origin) , direction(direction) , time(time) , medium(medium) {}

      virtual ~Ray() {}

      const Point3& Origin() const;
      const glm::vec3& Direction() const;
      float Time() const;
      Ref<Medium> GetMedium() const;

      Point3 At(float t) const;
      
      Point3 operator()(float t) const;

    protected:
      Point3 origin;
      glm::vec3 direction;
      float time;
      Ref<Medium> medium = nullptr;
  };

  class RayDifferential : public Ray {
    public:
      RayDifferential() : Ray() {} 
      RayDifferential(const Point3& origin , const glm::vec3& direction , float time , Ref<Medium> medium = nullptr)
        : Ray(origin , direction , time , medium) {}

      explicit RayDifferential(const Ray& r) : Ray(r) {}

      virtual ~RayDifferential() override {}

      const bool HasDifferentials() const;

      const Point3& RxOrigin() const;
      const Point3& RyOrigin() const;

      const glm::vec3& RxDirection() const;
      const glm::vec3& RyDirection() const;

      void ScaleDifferentials(float s);

    private:
      bool has_differentials = false;
      Point3 rx_origin , ry_origin;
      glm::vec3 rx_dir , ry_dir;
  };

} // namespace pbr

#endif // !PBR_RAY_HPP
