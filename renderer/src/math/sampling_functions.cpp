/**
 * \file math/sampling_functions.cpp
 **/
#include "math/sampling_functions.hpp"

#include "math/vecmath.hpp"
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
    Point2 offset = 2.f * u - glm::vec2(1 , 1);
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
  
  std::array<float , 3> SampleUniformTriangle(const Point2& u) {
    float b0 , b1;
    if (u[0] < u[1]) {
      b0 = u[0] / 2;
      b1 = u[1] - b0;
    } else {
      b1 = u[1] / 2;
      b0 = u[0] - b1;
    }

    return {
      b0 , b1 ,
      1 - b0 - b1
    };
  }
  
  std::array<float , 3> SampleSphericalTriangle(const std::array<Point3 , 3>& v , const Point3& p , const Point2& u , float& pdf) {
    pdf = 0.f;

    glm::vec3 a , b , c;
    a = v[0] - p;
    b = v[1] - p;
    c = v[2] - p;

    if (LengthSquared(a) == 0 || LengthSquared(b) == 0 || LengthSquared(c) == 0) {
      return { 0 , 0 , 0 };
    }

    a = Normalize(a);
    b = Normalize(b);
    c = Normalize(c);

    glm::vec3 n_ab = glm::cross(a , b);
    glm::vec3 n_bc = glm::cross(b , c);
    glm::vec3 n_ca = glm::cross(c , a);

    if (LengthSquared(n_ab) == 0 || LengthSquared(n_bc) == 0 || LengthSquared(n_ca) == 0) {
      return { 0 , 0 , 0 };
    }

    n_ab = Normalize(n_ab);
    n_bc = Normalize(n_bc);
    n_ca = Normalize(n_ca);

    float alpha = AngleBetween(n_ab , -n_ca);
    float beta = AngleBetween(n_bc , -n_ab);
    float gamma = AngleBetween(n_ca , -n_bc);

    float a_pi = alpha + beta + gamma;
    float ap_pi = Lerp(u[0] , pi , a_pi);

    float A = a_pi - pi;
    pdf = (A <= 0) ?
      0 : 1 / A;

    float cos_alpha = glm::cos(alpha);
    float sin_alpha = glm::sin(alpha);

    float sin_phi = glm::sin(ap_pi) * cos_alpha - glm::cos(ap_pi) * sin_alpha;
    float cos_phi = glm::cos(ap_pi) * cos_alpha + glm::sin(ap_pi) * sin_alpha;

    float k1 = cos_phi + cos_alpha;
    float k2 = sin_phi - sin_alpha * glm::dot(a , b);

    float cos_bp = (k2 + DifferenceOfProducts(k2 , cos_phi , k1 , sin_phi) * cos_alpha) /
                   (SumOfProducts(k2 , sin_phi , k1 , cos_phi) * sin_alpha);

    if (IsNaN(cos_bp)) {
      return { 0 , 0 , 0 };
    }

    cos_bp = Clamp(cos_bp , -1 , 1);

    float sin_bp = glm::sqrt(1 - glm::pow(cos_bp , 2));
    glm::vec3 cp = cos_bp * a + sin_bp * Normalize(GramSchmidt(c , a));

    float cos_theta = 1 - u[1] - (1 - glm::dot(cp , b));
    float sin_theta = glm::sqrt(1 - glm::pow(cos_theta , 2));

    glm::vec3 w = cos_theta * b + sin_theta * Normalize(GramSchmidt(cp , b));

    glm::vec3 e1 = v[1] - v[0];
    glm::vec3 e2 = v[2] - v[0];

    glm::vec3 s1 = glm::cross(e1 , e2);

    float divisor = glm::dot(s1 , e1);
    if (divisor == 0) {
      return {
        1.f / 3.f ,
        1.f / 3.f ,
        1.f / 3.f
      };
    }

    float inv_divisor = 1 / divisor;

    glm::vec3 s = p - v[0];
    float b1 = glm::dot(s , s1) * inv_divisor;
    float b2 = glm::dot(w , glm::cross(s , e1)) * inv_divisor;

    b1 = Clamp(b1 , 0 , 1);
    b2 = Clamp(b2 , 0 , 1);

    if (b1 + b2 > 1) {
      b1 /= b1 + b2;
      b2 /= b1 + b2;
    }

    return {
      1 - b1 - b2 ,
      b1 , b2
    };
  }
  
  Point2 InvertSphericalTriangleSample(const std::array<Point3 , 3>& v , const Point3& p , const glm::vec3& w) {
    glm::vec3 a , b , c;
    a = v[0] - p;
    b = v[1] - p;
    c = v[2] - p;

    if (LengthSquared(a) == 0 || LengthSquared(b) == 0 || LengthSquared(c) == 0) {
      return { 0 , 0 };
    }

    a = Normalize(a);
    b = Normalize(b);
    c = Normalize(c);

    glm::vec3 n_ab = glm::cross(a , b);
    glm::vec3 n_bc = glm::cross(b , c);
    glm::vec3 n_ca = glm::cross(c , a);

    if (LengthSquared(n_ab) == 0 || LengthSquared(n_bc) == 0 || LengthSquared(n_ca) == 0) {
      return { 0 , 0 };
    }

    n_ab = Normalize(n_ab);
    n_bc = Normalize(n_bc);
    n_ca = Normalize(n_ca);

    float alpha = AngleBetween(n_ab , -n_ca);
    float beta = AngleBetween(n_bc , -n_ab);
    float gamma = AngleBetween(n_ca , -n_bc);

    glm::vec3 cp = Normalize(glm::cross(glm::cross(b , w) , glm::cross(c , a)));
    if (glm::dot(cp , a + c) < 0) {
      cp = -cp;
    }

    float u0;
    if (glm::dot(a , cp) < 0.99999847691f) {
      u0 = 0;
    } else {
      glm::vec3 n_cpb = glm::cross(cp , b);
      glm::vec3 n_acp = glm::cross(a , cp);

      if (LengthSquared(n_cpb) == 0 || LengthSquared(n_acp) == 0) {
        return Point2{ 0.5f , 0.5f };
      }

      n_cpb = Normalize(n_cpb);
      n_acp = Normalize(n_acp);

      float ap = alpha + AngleBetween(n_ab , n_cpb) + AngleBetween(n_acp , -n_cpb) - pi;

      float A = alpha + beta + gamma - pi;
      u0 = ap / A;
    }

    float u1 = (1 - glm::dot(w , b)) / (1 - glm::dot(cp , b));
    return Point2{
      Clamp(u0 , 0 , 1) ,
      Clamp(u1 , 0 , 1)
    };
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
