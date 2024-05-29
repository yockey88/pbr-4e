/**
 * \file math/transform.cpp
 **/
#include "math/transform.hpp"
#include "math/vecmath.hpp"

#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/matrix.hpp>

namespace pbr {

namespace {

  inline glm::mat4 Mat4From2DArr(const float mat[4][4]) {
    return glm::mat4(
      glm::make_vec4(mat[0]) ,
      glm::make_vec4(mat[1]) ,
      glm::make_vec4(mat[2]) ,
      glm::make_vec4(mat[3])
    );
  }

} // anonymous namespace
  
  glm::mat4 Transform::identity = glm::identity<glm::mat4>();

  Transform::Transform(const glm::mat4& m)
      : model(m) {
    inverse_model = glm::inverse(model);
  }
      
  Transform::Transform(const float mat[16]) {
    model = glm::make_mat4(mat);
    inverse_model = glm::inverse(model);
  }
      
  Transform::Transform(const float mat[4][4]) {
    model = Mat4From2DArr(mat);
    inverse_model = glm::inverse(model);
  }
      
  Transform::Transform(const Frame& f) {
    model = glm::mat4{
      f.x.x , f.x.y , f.x.z , 0 ,
      f.y.x , f.y.y , f.y.z , 0 ,
      f.z.x , f.z.y , f.z.z , 0 , 
      0     , 0     , 0     , 1
    };

    inverse_model = glm::inverse(model);
  }

  bool Transform::operator==(const Transform& t) {
    return model == model;
  }

  bool Transform::operator!=(const Transform& t) {
    return model != model;
  }

  bool Transform::IsIdentity() {
    return model == identity;
  }
      
  bool Transform::HasScale(float tolerance) const {
    float la2 = LengthSquared(TransformVector(glm::vec3(1 , 0 , 0)));
    float lb2 = LengthSquared(TransformVector(glm::vec3(0 , 1 , 0)));
    float lc2 = LengthSquared(TransformVector(glm::vec3(0 , 0 , 1)));

    return (glm::abs(la2 - 1) > tolerance ||
            glm::abs(lb2 - 1) > tolerance ||
            glm::abs(lc2 - 1) > tolerance);
  }
      
  bool Transform::SwapsHandedness() const {
    glm::mat3 m(
      model[0][0] , model[0][1] , model[0][2] ,
      model[1][0] , model[1][1] , model[1][2] ,
      model[2][0] , model[2][1] , model[2][2]
    );

    return glm::determinant(m) < 0;
  }
  
  glm::vec3 Transform::TransformVector(const glm::vec3& v) const {
    return glm::vec3(model[0][0] * v.x + model[0][1] * v.y + model[0][2] * v.z + model[0][3] ,
                     model[1][0] * v.x + model[1][1] * v.y + model[1][2] * v.z + model[1][3] ,
                     model[2][0] * v.x + model[2][1] * v.y + model[2][2] * v.z + model[2][3]);
  }
      
  glm::vec3 Transform::TransformNormal(const glm::vec3& n) const {
    float x = n.x;
    float y = n.y;
    float z = n.z;
    return glm::vec3(inverse_model[0][0] * x + inverse_model[1][0] * y + inverse_model[2][0] * z ,
                     inverse_model[0][1] * x + inverse_model[1][1] * y + inverse_model[2][1] * z ,
                     inverse_model[0][2] * x + inverse_model[1][2] * y + inverse_model[2][2] * z);
  }
      
  Point3 Transform::TransformPoint(const Point3& p) const {
    float wp = model[3][0] * p.x + model[3][1] * p.y + model[3][2] * p.z + model[3][3];
    if (wp == 1) {
      return glm::vec3(model[0][0] * p.x + model[0][1] * p.y + model[0][2] * p.z + model[0][3] ,
                       model[1][0] * p.x + model[1][1] * p.y + model[1][2] * p.z + model[1][3] ,
                       model[2][0] * p.x + model[2][1] * p.y + model[2][2] * p.z + model[2][3]);
    } else {
      return (glm::vec3(model[0][0] * p.x + model[0][1] * p.y + model[0][2] * p.z + model[0][3] ,
                        model[1][0] * p.x + model[1][1] * p.y + model[1][2] * p.z + model[1][3] ,
                        model[2][0] * p.x + model[2][1] * p.y + model[2][2] * p.z + model[2][3]) / wp);
    }
  }
      
  Ray Transform::TransformRay(const Ray& r , float& tmax) const {
    Point3 o = TransformPoint(r.Origin());
    glm::vec3 d = TransformVector(r.Direction());
    return Ray(o , d , r.Time() , r.GetMedium());
  }
      
  Bounds3 Transform::TransformBounds(const Bounds3& b) const {
    Bounds3 bt;
    for (int32_t i = 0; i < 8; ++i) {
      bt = Bounds3::Union(bt , TransformPoint(b.Corner(i)));
    }
    return bt;
  }
      
  Transform Transform::operator*(const Transform& t) const {
    return Transform(model * t.model);
  }

  const glm::mat4& Transform::Matrix() {
    return model;
  }

  const glm::mat4& Transform::InverseMatrix() {
    return inverse_model;
  }
      
  Transform Transform::Translate(const glm::vec3& v) {
    return Transform(
      glm::mat4{
        1 , 0 , 0 , v.x ,
        0 , 1 , 0 , v.y ,
        0 , 0 , 1 , v.z ,
        0 , 0 , 0 , 1
      }
    );
  }

  Transform Transform::Scale(const glm::vec3& v) {
    return Transform(
      glm::mat4{
        v.x , 0   , 0   , 0 ,  
        0   , v.y , 0   , 0 , 
        0   , 0   , v.z , 0 , 
        0   , 0   , 0   , 1
      }
    );
  }

  Transform Transform::RotateX(float angle) {
    float sin = glm::sin(glm::radians(angle));
    float cos = glm::cos(glm::radians(angle));
    return Transform(
      glm::mat4{
        1 , 0   , 0    , 0 ,
        0 , cos , -sin , 0 ,
        0 , sin ,  cos , 0 ,
        0 , 0   , 0    , 1
      }
    );
  }

  Transform Transform::RotateY(float angle) {
    float sin = glm::sin(glm::radians(angle));
    float cos = glm::cos(glm::radians(angle));
    return Transform(
      glm::mat4{
         cos , 0 , sin , 0 ,
         0   , 1 , 0   , 0 ,
        -sin , 0 , cos , 0 ,
         0   , 0 , 0   , 1
      }
    );
  }

  Transform Transform::RotateZ(float angle) {
    float sin = glm::sin(glm::radians(angle));
    float cos = glm::cos(glm::radians(angle));
    return Transform(
      glm::mat4{
        cos , -sin , 0 , 0 ,
        sin ,  cos , 0 , 0 ,
        0   ,  0   , 1 , 0 ,
        0   ,  0   , 0 , 1
      }
    );
  }

  Transform Transform::Rotate(float sin , float cos , const glm::vec3& axis) {
    glm::vec3 a = Normalize(axis);
    glm::mat4 m;
    m[0][0] = a.x * a.x + (1 - a.x * a.x) * cos;
    m[0][1] = a.y * a.y + (1 - cos) - a.z * sin;
    m[0][2] = a.x * a.z + (1 - cos) + a.y * sin;
    m[0][3] = 0;
    
    m[1][0] = a.x * a.y + (1 - cos) + a.z * sin;
    m[1][1] = a.y * a.y + (1 - a.y * a.y) * cos;
    m[1][2] = a.y * a.z + (1 - cos) - a.x * sin;
    m[1][3] = 0;

    m[2][0] = a.x * a.z + (1 - cos) - a.y * sin;
    m[2][1] = a.y * a.y + (1 - cos) - a.x * sin;
    m[2][2] = a.z * a.z + (1 - a.z * a.z) * cos;
    m[2][3] = 0;

    return Transform(m);
  }

  Transform Transform::Rotate(float angle , const glm::vec3& axis) {
    float sin = glm::sin(glm::radians(angle));
    float cos = glm::cos(glm::radians(angle));
    return Rotate(sin , cos , axis); 
  }
      
  Transform Transform::RotateFromTo(const glm::vec3& from , const glm::vec3& to) {
    glm::vec3 refl;
    if (glm::abs(from.x) < 0.72f && glm::abs(to.x) < 0.72f) {
      refl = glm::vec3(1 , 0 , 0); 
    } else if (glm::abs(from.y) < 0.72f && glm::abs(to.y) < 0.72f) {
      refl = glm::vec3(0 , 1 , 0);
    } else {
      refl = glm::vec3(0 , 0 , 1);
    }

    glm::vec3 u = refl - from;
    glm::vec3 v = refl - to;
    glm::mat4 r;

    for (int32_t i = 0; i < 3; ++i) {
      for (int32_t j = 0; j < 3; ++j) {
        r[i][j] = ((i == j) ? 1 : 0) -
          2 / glm::dot(u , v) * u[i] * u[j] -
          2 / glm::dot(v , v) * v[i] * v[j] +
          4 * glm::dot(u , v) / (glm::dot(u , v) * glm::dot(v , v)) * v[i] * u[j];
      }
    }

    return Transform(r);
  }
      
  Transform Transform::LookAt(const glm::vec3& pos , const Point3& look , const glm::vec3& up) {
    glm::mat4 world_from_cam;
    world_from_cam[0][3] = pos.x;
    world_from_cam[1][3] = pos.y;
    world_from_cam[2][3] = pos.z;
    world_from_cam[3][3] = 1;

    glm::vec3 dir = Normalize(look - pos);
    glm::vec3 right = Normalize(glm::cross(Normalize(up) , dir));
    glm::vec3 new_up = glm::cross(dir , right);

    world_from_cam[0][0] = right.x;
    world_from_cam[1][0] = right.x;
    world_from_cam[2][0] = right.x;
    world_from_cam[3][0] = 0.f;

    world_from_cam[0][1] = new_up.x;
    world_from_cam[1][1] = new_up.x;
    world_from_cam[2][1] = new_up.x;
    world_from_cam[3][1] = 0.f;

    world_from_cam[0][2] = dir.x;
    world_from_cam[1][2] = dir.x;
    world_from_cam[2][2] = dir.x;
    world_from_cam[3][2] = 0.f;

    glm::mat4 camera_from_world = glm::inverse(world_from_cam);
    return Transform(camera_from_world);
  }
      
  Transform Transform::Transpose(const Transform& t) {
    return Transform(glm::transpose(t.model));
  }

} // namespace pbr
