/**
 * \file math/transform.hpp
 **/
#ifndef PBR_TRANSFORM_HPP
#define PBR_TRANSFORM_HPP

#include <glm/glm.hpp>

#include "math/vecmath.hpp"
#include "math/ray.hpp"
#include "math/bounds.hpp"
#include "math/frame.hpp"

namespace pbr {

  class Transform {
    public:
      Transform();

      Transform(const glm::mat4& m);
      
      Transform(const float mat[16]);
      Transform(const float mat[4][4]);

      explicit Transform(const Frame& f);

      bool operator==(const Transform& t);
      bool operator!=(const Transform& t);

      bool IsIdentity();

      bool HasScale(float tolerance = 1e-3f) const;

      bool SwapsHandedness() const;

      glm::vec3 TransformVector(const glm::vec3& v) const;
      glm::vec3 TransformNormal(const glm::vec3& n) const;
      Point3 TransformPoint(const Point3& p) const;
      Ray TransformRay(const Ray& r , float& tmax) const;
      Bounds3 TransformBounds(const Bounds3& b) const;

      Transform operator*(const Transform& t) const;

      const glm::mat4& Matrix();

      const glm::mat4& InverseMatrix();

      static Transform Translate(const glm::vec3& v);
      static Transform Scale(const glm::vec3& v);
      static Transform RotateX(float angle);
      static Transform RotateY(float angle);
      static Transform RotateZ(float angle);

      static Transform Rotate(float sin , float cos , const glm::vec3& axis);
      static Transform Rotate(float angle , const glm::vec3& axis);
      
      static Transform RotateFromTo(const glm::vec3& from , const glm::vec3& to);

      static Transform LookAt(const glm::vec3& pos , const Point3& look , const glm::vec3& up);

      static Transform Transpose(const Transform& t);

    private:
      glm::mat4 model;
      glm::mat4 inverse_model;

      static glm::mat4 identity;
  }; 

} // namespace pbr

#endif // !PBR_TRANSFORM_HPP
