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

      void SetOrigin(const Point3& o);
      void SetDirection(const glm::vec3& dir);
      void SetTime(float time);
      void SetMedium(const Ref<Medium>& medium);

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
      RayDifferential(const Ray& r) : Ray(r) {}
      RayDifferential(const Point3& origin , const glm::vec3& direction , float time , Ref<Medium> medium = nullptr)
        : Ray(origin , direction , time , medium) {}


      virtual ~RayDifferential() override {}

      void SetRxOrigin(const Point3& o);
      void SetRyOrigin(const Point3& o);

      void SetRxDirection(const glm::vec3& dir);
      void SetRyDirection(const glm::vec3& dir);

      void SetHasDifferentials(bool has);

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
