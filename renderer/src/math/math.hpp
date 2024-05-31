/**
 * \file math/math.hpp
 **/
#ifndef PBR_MATH_HPP
#define PBR_MATH_HPP

#include <glm/detail/qualifier.hpp>
#include <span>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>

#include "util/defines.hpp"
#include "util/float.hpp"

namespace pbr {

  constexpr static float pi = 3.14159265358979323846f;
  constexpr static float pi_over_2 = pi / 2;
  constexpr static float pi_over_4 = pi / 4;
  
  template <size_t N , typename T>
  using Point = glm::vec<N , T , glm::defaultp>;

  using Point2 = Point<2 , float>;
  using Point3 = Point<3 , float>;
  using Point4 = Point<4 , float>;

  template <typename T , typename U , typename V>
  constexpr T Clamp(T val , U low , V high) {
    if (val < low) {
      return T(low);
    } else if (val > high) {
      return T(high);
    } else {
      return val;
    }
  }

  template <typename Pred>
  size_t FindInterval(size_t sz , const Pred& pred) {
    using ssize_t = std::make_signed_t<size_t>;
    ssize_t size = static_cast<ssize_t>(sz) - 2;
    ssize_t first = 1;
    while (size > 0) {
      size_t half = static_cast<size_t>(size) >> 1;
      size_t middle = first + half;
      bool pred_res = pred(middle);
      first = pred_res ? 
        middle + 1 : first;
      size = pred_res ? 
        size - (half + 1) : half;
    }

    return static_cast<size_t>(Clamp(static_cast<ssize_t>(first) - 1 , 0 , sz - 2));
  }

  template <typename TResult , size_t N , typename T>
  inline TResult Mul(const glm::mat<N , N , float , glm::defaultp>& m , const T& v) {
    TResult result;
    for (int32_t i = 0; i < N; ++i) {
      result[0] = 0;
      for (int32_t j = 0; j < N; ++j) {
        result[i] += m[i][j] * v[j];
      }
    }
    return result;
  }

  template <typename TResult , typename T>
  inline TResult Mul(const glm::mat3& m , const T& v) {
    return Mul<TResult>(m , v);
  }
  
  template <typename TResult , typename T>
  inline TResult Mul(const glm::mat4& m , const T& v) {
    return Mul<TResult>(m , v);
  }

  inline float SafeAsin(float x) {
    return glm::asin(Clamp(x , -1 , 1));
  }

  inline float SafeAcos(float x) {
    return glm::acos(Clamp(x , -1 , 1));
  }

  inline float SafeSqrt(float x) {
    return glm::sqrt(glm::max(0.f , x));
  }

  template <typename F , typename C>
  constexpr F EvaluatePolynomial(F t , C c) {
    return c;
  }

  template <typename F , typename C , typename... Args>
  F EvaluatePolynomial(F t , C c , Args... args) {
    return FMA(t , EvaluatePolynomial(t , args...) , c);
  }

  template <size_t N>
  inline Opt<glm::mat<N , N , float , glm::defaultp>> LinearLeastSquares(const float a[][N] , const float b[][N] , int32_t rows) {
    glm::mat<N , N , float , glm::defaultp> AtA = glm::mat<N , N , float , glm::defaultp>(0.f);
    glm::mat<N , N , float , glm::defaultp> AtB = glm::mat<N , N , float , glm::defaultp>(0.f);

    for (int32_t i = 0; i < N; ++i) {
      for (int32_t j = 0; j < N; ++j) {
        for (int32_t r = 0; r < rows; ++r) {
          AtA[i][j] = a[r][i] * a[r][j];
          AtB[i][j] = a[r][i] * b[r][j];
        }
      }
    }

    auto AtAi = glm::inverse(AtA);
    return glm::transpose(AtAi * AtB);
  }

  inline Opt<glm::mat3> LinearLeastSquares(const float a[][3] , const float b[][3] , int32_t rows) {
    return LinearLeastSquares<3>(a , b , rows);
  }

  inline Opt<glm::mat4> LinearLeastSquares(const float a[][4] , const float b[][4], int32_t rows) {
    return LinearLeastSquares<4>(a , b , rows);
  }

  float FastExp(float x);

  float Lerp(float x , float a , float b);
  
  float Bilerp(const glm::vec2& p , const std::span<const float>& w);

  Point2 WrapEqualAreaSquare(const Point2& uv);

} // namespace pbr

#endif // !PBR_MATH_HPP
