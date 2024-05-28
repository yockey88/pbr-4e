/**
 * \file vecmath.cpp
 **/
#include "math/vecmath.hpp"

namespace pbr {
  
  void BuildOrthonormalBasis(const glm::vec3& v1 , glm::vec3& v2 , glm::vec3& v3) {
    float sign = std::copysign(1.f , v1.z);
    float a = -1.f / (sign + v1.z);
    float b = v1.x * v1.y * a;
    v2 = glm::vec3(1 + sign * glm::pow(v1.x , 2) * a , sign * b , -sign * v1.x);
    v3 = glm::vec3(b , sign + glm::pow(v1.y , 2) * a , -v1.y);
  }
  
  float SphericalTriangleArea(const glm::vec3& a , const glm::vec3& b , const glm::vec3& c) {
    auto numerator = glm::dot(a , glm::cross(b , c));
    auto denominator = 1 + glm::dot(a , b) + glm::dot(a , c) + glm::dot(b , c);
    auto a_tan = std::atan2(numerator , denominator);
    return std::abs(2 * a_tan);
  }
  
  float SphericalQuadArea(const glm::vec3& a , const glm::vec3& b , const glm::vec3& c , const glm::vec3& d) {
    glm::vec3 a_b = glm::cross(a , b);
    glm::vec3 b_c = glm::cross(b , c);
    glm::vec3 c_d = glm::cross(c , d);
    glm::vec3 d_a = glm::cross(d , a);

    if (LengthSquared(a_b) == 0 || LengthSquared(b_c) == 0 || 
        LengthSquared(c_d) == 0 || LengthSquared(d_a) == 0) {
      return 0;
    }

    a_b = Normalize(a_b);
    b_c = Normalize(b_c);
    c_d = Normalize(c_d);
    d_a = Normalize(d_a);

    float alpha = AngleBetween(d_a , -a_b);
    float beta = AngleBetween(a_b , -b_c);
    float gamma = AngleBetween(b_c , -c_d);
    float delta = AngleBetween(c_d , -d_a);

    return std::abs(alpha + beta + gamma + delta - 2 * pi);
  }
  
  glm::vec3 SphericalDirection(float sin_theta , float cos_theta , float phi) {
    return {
      Clamp(sin_theta , -1 , 1) * glm::cos(phi) ,
      Clamp(sin_theta , -1 , 1) * glm::sin(phi) ,
      Clamp(cos_theta , -1 , 1)
    };
  }
  
  glm::vec3 EqualAreaSquareToSphere(const Point2& p) {
    float u = 2 * p.x - 1;
    float v = 2 * p.y - 1;

    float up = glm::abs(u);
    float vp = glm::abs(v);

    float signed_dist = 1 - (up + vp);
    float d = glm::abs(signed_dist);
    float r = 1 - d;

    float phi = ((r == 0) ?
      1 : (vp - up) / r + 1) * pi / 4;

    float z = std::copysign(1 - glm::pow(r , 2) , signed_dist);

    float cos_phi = std::copysign(glm::cos(phi) , u);
    float sin_phi = std::copysign(glm::sin(phi) , v);

    return {
      cos_phi * r * SafeSqrt(2 - glm::pow(r , 2)) ,
      sin_phi * r * SafeSqrt(2 - glm::pow(r , 2)) ,
      z
    };
  }
  
} // namespace pbr
