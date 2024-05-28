/**
 * \file math/vecmath.hpp
 **/
#ifndef PBR_VECMATH_HPP
#define PBR_VECMATH_HPP

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/detail/qualifier.hpp>

#include "math/math.hpp"

namespace pbr {

  template <size_t N , typename T>
  using Point = glm::vec<N , T , glm::defaultp>;

  using Point2 = Point<2 , float>;
  using Point3 = Point<3 , float>;
  using Point4 = Point<4 , float>;

  template <typename T>
  inline typename std::enable_if_t<std::is_integral_v<T> , T> FMA(T a , T b , T c) {
    return a * b + c;
  }

  template <typename Ta , typename Tb , typename Tc , typename Td>
  inline auto DifferenceOfProducts(Ta a , Tb b , Tc c , Td d) {
    auto cd = c * d;
    auto diff_of_products = FMA(a , b , -cd);
    auto error = FMA(-c , d , cd);
    return diff_of_products + error;
  }

  template <size_t N , typename T>
  inline float LengthSquared(const glm::vec<N , T , glm::defaultp>& v) {
    return glm::pow(v.length() , 2); 
  }
  
  inline float LengthSquared(const glm::vec2& v) {
    return LengthSquared<2 , float>(v);
  }

  inline float LengthSquared(const glm::vec3& v) {
    return LengthSquared<3 , float>(v);
  }

  template <size_t N , typename T>
  inline glm::vec<N , T , glm::defaultp> Normalize(const glm::vec<N , T , glm::defaultp>& v) {
    return v / v.length();
  }
  
  inline glm::vec2 Normalize(const glm::vec2& v) {
    return Normalize<2 , float>(v);
  }

  inline glm::vec3 Normalize(const glm::vec3& v) {
    return Normalize<3 , float>(v);
  }
  
  template <size_t N , typename T>
  inline float AbsDot(const glm::vec<N , T , glm::defaultp>& v1 , const glm::vec<N , T , glm::defaultp>& v2) { 
    return glm::abs(glm::dot(v1 , v2)); 
  }

  inline float AbsDot(const glm::vec2& v1 , const glm::vec2& v2) {
    return AbsDot<2 , float>(v1 , v2);
  }
  
  inline float AbsDot(const glm::vec3& v1 , const glm::vec3& v2) {
    return AbsDot<3 , float>(v1 , v2);
  }

  template <size_t N , typename T>
  inline glm::vec<N , T , glm::defaultp> GramSchmidt(const glm::vec<N , T , glm::defaultp>& v , const glm::vec<N , T , glm::defaultp>& w) {
    return v - glm::dot(v , w) * w;
  }
  
  inline glm::vec2 GramSchmidt(const glm::vec2& v , const glm::vec2& w) {
    return GramSchmidt<2 , float>(v , w);
  }
  
  inline glm::vec3 GramSchmidt(const glm::vec3& v , const glm::vec3& w) {
    return GramSchmidt<3 , float>(v , w);
  }

  template <size_t N , typename T>
  inline float AngleBetween(const glm::vec<N , T , glm::defaultp>& v1 , const glm::vec<N , T , glm::defaultp>& v2) {
    if (glm::dot(v1 , v2) < 0) {
      return pi - 2 * SafeAsin(float((v1 + v2).length()) / 2);
    } else {
      return 2 * SafeAsin(float((v2 - v1).length()) / 2);
    }
  }

  inline float AngleBetween(const glm::vec2& v1, const glm::vec2& v2) {
    return AngleBetween<2 , float>(v1 , v2);
  }
  
  inline float AngleBetween(const glm::vec3& v1, const glm::vec3& v2) {
    return AngleBetween<3 , float>(v1 , v2);
  }

  template <size_t N , typename T>
  inline auto Distance(const Point<N , T>& p1 , const Point<N , T>& p2) {
    return (p1 - p2).length();
  }

  template <size_t N , typename T>
  inline auto DistanceSquared(const Point<N , T>& p1 , const Point<N , T>& p2) {
    return LengthSquared(p1 - p2);
  }
  
  inline float DistanceSquared(const Point2& p1 , const Point2& p2) {
    return LengthSquared(p1 - p2);
  }
  
  inline float DistanceSquared(const Point3& p1 , const Point3& p2) {
    return LengthSquared(p1 - p2);
  }

  template <size_t N , typename T>
  inline glm::vec<N , T , glm::defaultp> FaceForward(const glm::vec<N , T , glm::defaultp>& n , const glm::vec<N , T , glm::defaultp>& v) {
    return (glm::dot(n , v) < 0.f) ? 
      -n : n;
  }

  template <typename T , glm::qualifier Q>
  inline glm::vec<2 , T , Q> Min(const glm::vec<2 , T , Q>& v1 , const glm::vec<2 , T , Q>& v2) {
    return {
      glm::min(v1.x , v2.x) ,
      glm::min(v1.y , v2.y)
    }; 
  }
  
  template <typename T , glm::qualifier Q>
  inline glm::vec<3 , T , Q> Min(const glm::vec<3 , T , Q>& v1 , const glm::vec<3 , T , Q>& v2) {
    return {
      glm::min(v1.x , v2.x) ,
      glm::min(v1.y , v2.y) ,
      glm::min(v1.z , v2.z)
    }; 
  }
  
  template <typename T , glm::qualifier Q>
  inline glm::vec<2 , T , Q> Max(const glm::vec<2 , T , Q>& v1 , const glm::vec<2 , T , Q>& v2) {
    return {
      glm::max(v1.x , v2.x) ,
      glm::max(v1.y , v2.y)
    }; 
  }
  
  template <typename T , glm::qualifier Q>
  inline glm::vec<3 , T , Q> Max(const glm::vec<3 , T , Q>& v1 , const glm::vec<3 , T , Q>& v2) {
    return {
      glm::max(v1.x , v2.x) ,
      glm::max(v1.y , v2.y) ,
      glm::max(v1.z , v2.z)
    }; 
  }

  /// to do generalize this to n-dimensions to use template pattern
  void BuildOrthonormalBasis(const glm::vec3& v1 , glm::vec3& v2 , glm::vec3& v3);

  /// Spherical geometry math

  inline float CosTheta(const glm::vec3& w) {
    return w.z;
  }
  
  inline float Cos2Theta(const glm::vec3& w) {
    return glm::pow(w.z , 2);
  }
  
  inline float AbsCosTheta(const glm::vec3& w) {
    return glm::abs(w.z);
  }
  
  inline float Sin2Theta(const glm::vec3& w) {
    return std::max<float>(0 , 1 - Cos2Theta(w));
  }
  
  inline float SinTheta(const glm::vec3& w) {
    return glm::sqrt(Sin2Theta(w));
  }

  inline float TanTheta(const glm::vec3& w) {
    return SinTheta(w) / CosTheta(w);
  }
  
  inline float Tan2Theta(const glm::vec3& w) {
    return Sin2Theta(w) / Cos2Theta(w);
  }

  inline float SphericalTheta(const glm::vec3& v) {
    return SafeAcos(v.z);
  }

  inline float CosPhi(const glm::vec3& w) {
    float sin_theta = SinTheta(w);
    return sin_theta == 0 ?
      1 : Clamp(w.x / sin_theta , -1 , 1);
  }

  inline float SinPhi(const glm::vec3& w) {
    float sin_theta = SinTheta(w);
    return sin_theta == 0 ?
      1 : Clamp(w.y / sin_theta , -1 , 1);
  }

  inline float CosDPhi(const glm::vec3& wa , const glm::vec3& wb) {
    float waxy = glm::pow(wa.x , 2) + glm::pow(wa.y , 2);
    float wbxy = glm::pow(wb.x , 2) + glm::pow(wb.y , 2);

    if (waxy == 0 || wbxy == 0) {
      return 1;
    }

    return Clamp((wa.x * wb.x + wa.y * wb.y) / glm::sqrt(waxy * wbxy) , -1 , 1);
  }

  inline float SphericalPhi(const glm::vec3& v) {
    float p = std::atan2(v.y , v.x);
    return (p < 0) ? 
      (p + 2 * pi) : p;
  }

  float SphericalTriangleArea(const glm::vec3& a , const glm::vec3& b , const glm::vec3& c);

  float SphericalQuadArea(const glm::vec3& a , const glm::vec3& b , const glm::vec3& c , const glm::vec3& d);

  glm::vec3 SphericalDirection(float sin_theta , float cos_theta , float phi);

  glm::vec3 EqualAreaSquareToSphere(const Point2& p);

} // namespace pbr

#endif // !PBR_VECMATH_HPP
