/**
 * \file math/transform.hpp
 **/
#ifndef PBR_TRANSFORM_HPP
#define PBR_TRANSFORM_HPP

#include "math/ray.hpp"
#include "math/bounds.hpp"
#include "math/frame.hpp"
#include "geometry/interaction.hpp"

namespace pbr {

  class Transform {
    public:
      Transform();

      Transform(const glm::mat4& m);
      Transform(const glm::quat& q);
      
      Transform(const float mat[16]);
      Transform(const float mat[4][4]);

      explicit Transform(const Frame& f);

      bool operator==(const Transform& t) const;
      bool operator!=(const Transform& t) const;

      bool IsIdentity();

      bool HasScale(float tolerance = 1e-3f) const;

      bool SwapsHandedness() const;

      glm::vec3 TransformVector(const glm::vec3& v) const;
      glm::vec3 TransformNormal(const glm::vec3& n) const;
      Point3 TransformPoint(const Point3& p) const;
      Ray TransformRay(const Ray& r , float* tmax = nullptr) const;
      Bounds3 TransformBounds(const Bounds3& b) const;
      Ref<SurfaceInteraction> TransformSurfaceInteraction(const Ref<SurfaceInteraction>& s) const;

      Point3 ApplyInverse(const Point3& p) const;

      Transform operator*(const Transform& t) const;

      const glm::mat4& Matrix();

      const glm::mat4& InverseMatrix();

      void Decompose(glm::vec3& T , glm::mat4& R , glm::mat4& S) const;

      static Transform Translate(const glm::vec3& v);
      static Transform Translate(float x , float y , float z);
      static Transform Scale(const glm::vec3& v);
      static Transform Scale(float x , float y , float z);
      static Transform RotateX(float angle);
      static Transform RotateY(float angle);
      static Transform RotateZ(float angle);

      static Transform Rotate(float sin , float cos , const glm::vec3& axis);
      static Transform Rotate(float angle , const glm::vec3& axis);
       
      static Transform RotateFromTo(const glm::vec3& from , const glm::vec3& to);

      static Transform LookAt(const glm::vec3& pos , const Point3& look , const glm::vec3& up);

      static Transform Transpose(const Transform& t);

      static Transform Inverse(const Transform& t);

    private:
      glm::mat4 model;
      glm::mat4 inverse_model;

      static glm::mat4 identity;
  }; 
  
  class AnimatedTransform {
    public:
      AnimatedTransform() {}
      AnimatedTransform(const Transform& t) 
        : AnimatedTransform(t , 0 , t , 1) {}
      AnimatedTransform(const Transform& start , float start_t , 
                        const Transform& end   , float end_t);

      bool HasScale() const;

      Point3 TransformVector(const glm::vec3& v , float time) const;
      Point3 TransformPoint(const Point3& p , float time) const;
      glm::vec3 TransformNormal(const glm::vec3& n , float time) const;
      Ray TransformRay(const Ray& r , float* tmax = nullptr) const;

      Point3 ApplyInverse(const Point3& p , float time) const;

      Transform Interpolate(float time) const;

      Transform start_transform , end_transform;

      float start_time = std::numeric_limits<float>::max();
      float end_time = std::numeric_limits<float>::min();

    private:
      glm::vec3 T[2];
      glm::quat R[2];
      glm::mat4 S[2];

      bool actually_animated = false;
      bool has_rotation = false;
  };

} // namespace pbr

#endif // !PBR_TRANSFORM_HPP
