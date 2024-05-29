/**
 * \file radiometry/spectrum.cpp
 **/
#include "radiometry/spectrum.hpp"

#include "math/math.hpp"
#include <algorithm>

namespace pbr {

  float Blackbody(float wavelength , float t) {
    if (t <= 0.f) {
      return 0.f;
    }

    float l = wavelength * 1e-9f;
    float le = (2 * kPlancks * glm::pow(kLightSpeed , 2)) /
      (glm::pow(l , 5) * (FastExp((kPlancks * kLightSpeed) / (l * kBoltzmann * t)) - 1));
    return le;
  }

  float ConstantSpectrum::MaxValue() const {
    return c;
  }

  float ConstantSpectrum::Sample(float wavelength) const {
    return c;
  }
      
  DenselySampledSpectrum::DenselySampledSpectrum(Ref<Spectrum> spec , int32_t lmin , int32_t lmax)
      : lambda_min(lmin) , lambda_max(lmax) , values(lmax - lmin + 1) {
    if (spec != nullptr) {
      for (int32_t l = lambda_min; l <= lambda_max; ++l) {
        values[l] = spec->Sample(l);
      }
    }
  }
  
  float DenselySampledSpectrum::MaxValue() const {
    if (values.empty()) {
      return 0.f;
    }

    return *std::max_element(values.begin() , values.end());
  }
      
  float DenselySampledSpectrum::Sample(float wavelength) const {
    int32_t offset = std::lround(wavelength) - lambda_min;
    if (offset < 0 || offset >= values.size()) {
      return 0.f;
    }

    return values[offset];
  }
      
  PiecewiseLinearSpectrum::PiecewiseLinearSpectrum(const std::span<const float> ls , const std::span<const float> vs) {
    for (const auto& l : ls) {
      lambdas.push_back(l);
    } 
    for (const auto& v : vs) {
      values.push_back(v);
    }
  }
      
  float PiecewiseLinearSpectrum::MaxValue() const {
    if (values.empty()) {
      return 0.f;
    }

    return *std::max_element(values.begin() , values.end());
  }
      
  float PiecewiseLinearSpectrum::Sample(float wavelength) const {
    if (lambdas.empty() || wavelength < lambdas.front() || wavelength > lambdas.back()) {
      return 0.f;
    } 

    int32_t o = FindInterval(lambdas.size() , [&](int32_t i) { return lambdas[i] <= wavelength; });
    float t = (wavelength - lambdas[o]) / (lambdas[o + 1] - lambdas[o]);
    return Lerp(t , values[o] , values[o + 1]);
  }

  BlackbodySpectrum::BlackbodySpectrum(float temp) 
      : temp(temp) {
    float lambda_max = kWiensDisplacement;    
    normalization_factor = 1.f / Blackbody(lambda_max * 1e9f , temp);
  }

  float BlackbodySpectrum::MaxValue() const {
    return 1.f;
  }

  float BlackbodySpectrum::Sample(float wavelength) const {
    return Blackbody(wavelength , temp) * normalization_factor;
  }

} // namespace pbr
