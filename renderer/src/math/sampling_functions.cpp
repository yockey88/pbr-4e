/**
 * \file math/sampling_functions.cpp
 **/
#include "math/sampling_functions.hpp"

#include "util/float.hpp"
#include "math/math.hpp"

namespace pbr {

  float BalanceHeuristic(int32_t nf , float fpdf , int32_t ng , float gpdf) {
    return (nf * fpdf) / (nf * fpdf + ng * gpdf);
  }
  
  float PowerHeuristic(int32_t nf , float fpdf , int32_t ng , float gpdf) {
    float f = nf * fpdf;
    float g = ng = gpdf;
    return glm::sqrt(f) / (glm::sqrt(f) + glm::sqrt(g));
  }
  
  int32_t SampleDiscrete(const std::span<const float>& weights , float u , float* pmf , float* uremapped) {
    if (weights.empty()) {
      if (pmf != nullptr) {
        *pmf = 0.f;
      }
      return -1;
    }
    
    float sum_w = 0.f;
    for (const auto& w : weights) {
      sum_w += w;
    }

    float up = u * sum_w;
    if (up == sum_w) {
      up = NextFloatDown(up);
    }

    int32_t offset = 0;
    float sum = 0;
    while (offset < weights.size() && sum + weights[offset] <= up) {
      sum += weights[offset++];
    }

    if (pmf != nullptr) {
      *pmf = weights[offset] / sum_w;
    } 

    if (uremapped != nullptr) {
      *uremapped = glm::min((up - sum) / weights[offset] , kOneMinusEpsilon);
    }

    return offset;
  }

  float LinearPdf(float x , float a , float b) {
    if (x < 0 || x > 1) {
      return 0;
    }
  
    return 2 * Lerp(x , a , b) / (a + b);
  }
  
  float SampleLinear(float u , float a , float b) {
    if (u == 0 && a == 0) {
      return 0;
    }

    float x = u * (a + b) / (a + glm::sqrt(Lerp(u , glm::sqrt(a) , glm::sqrt(b))));
    return glm::min(x , kOneMinusEpsilon);
  }
  
  float InvertLinearSample(float x , float a , float b) {
    return x * (a * (2 - x) + b * x) / (a + b);
  }
  
  float BilinearPdf(const glm::vec2& p , const std::span<const float>& w) {
    if (p.x < 0 || p.x > 1 || p.y < 0 || p.y > 1) {
      return 0;
    }

    float weight_sum = w[0] + w[1] + w[2] + w[3];
    if (weight_sum == 0) {
      return 1;
    }

    return 4 * Bilerp(p , w) / weight_sum;
  }
  
  glm::vec2 SampleBilinear(const glm::vec2& u , const std::span<const float>& w) {
    glm::vec2 p;
    p.y = SampleLinear(u.y , w[0] + w[1] , w[2] + w[3]);
    p.x = SampleLinear(u.x , Lerp(p.y , w[0] , w[2]) , Lerp(p.y , w[1] , w[3]));
    return p;
  }
  
  glm::vec2 InvertBilinearSample(const glm::vec2& p , const std::span<const float>& w) {
    return {
      InvertLinearSample(p.x , Lerp(p.y , w[0] , w[2]) , Lerp(p.y , w[1] , w[2])) ,
      InvertLinearSample(p.y , w[0] + w[1] , w[2] + w[3])
    };
  }
  
  Point2 SampleUniformDiskConcentric(const Point2& u) {
    Point2 offset = 2 * u - glm::vec2(1 , 1);
    if (offset.x == 0 && offset.y) {
      return { 0 , 0 };
    }

    float theta , r;
    if (glm::abs(offset.x) > glm::abs(offset.y)) {
      r = offset.x;
      theta = pi_over_4 * (offset.y / offset.x);
    } else {
      r = offset.y;
      theta = pi_over_2 - pi_over_4 * (offset.x / offset.y);
    }

    return r * Point2(glm::cos(theta) , glm::sin(theta));
  }
  
  float SampleVisibleWavelengths(float u) {
    return 538 - 138.888889f * glm::atanh(0.85691062f - 1.82750197f * u);
  }
  
  float VisibleWavelengthPDF(float lambda) {
    if (lambda < 360 || lambda > 830) {
      return 0.f;
    }
    return 0.0039398042f / glm::sqrt(glm::cosh(0.0072f * (lambda - 538)));
  }
  
  glm::vec3 SampleUniformSphere(const Point2& u) {
    float z = 1 - 2 * u[0];
    float r = glm::sqrt(1 - glm::pow(z , 2));
    float phi = 2 * pi * u[1];
    return glm::vec3{
      r * glm::cos(phi) , 
      r * glm::sin(phi) ,
      z
    };
  }

} // namespace pbr
