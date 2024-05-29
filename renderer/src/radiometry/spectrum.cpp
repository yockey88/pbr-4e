/**
 * \file radiometry/spectrum.cpp
 **/
#include "radiometry/spectrum.hpp"

#include <algorithm>

#include "math/math.hpp"
#include "radiometry/color.hpp"

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

  SampledSpectrum ConstantSpectrum::SampleWavelengths(const SampledWavelengths& wavelengths) const {
    return SampledSpectrum(c); 
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

  SampledSpectrum DenselySampledSpectrum::SampleWavelengths(const SampledWavelengths& wavelengths) const {
    SampledSpectrum s;
    for (int32_t i = 0; i < kSpectrumSamples; ++i) {
      int32_t offset = std::lround(wavelengths[i]) - lambda_min;
      if (offset < 0 || offset >= values.size()) {
        s[i] = 0;
      } else {
        s[i] = values[offset];
      }
    }
    return s;
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
  
  SampledSpectrum PiecewiseLinearSpectrum::SampleWavelengths(const SampledWavelengths& wavelengths) const {
    SampledSpectrum s;
    for (int32_t i = 0; i < kSpectrumSamples; ++i) {
      s[i] = Sample(wavelengths[i]);
    }
    return s;
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
  
  SampledSpectrum BlackbodySpectrum::SampleWavelengths(const SampledWavelengths& wavelengths) const {
    SampledSpectrum s;
    for (int32_t i = 0; i < kSpectrumSamples; ++i) {
      s[i] = Blackbody(wavelengths[i] , temp);
    }
    return s;
  }
      
  SampledSpectrum::SampledSpectrum() {
    for (auto i = 0; i < kSpectrumSamples; ++i) {
      values[i] = 0.f;
    }
  }

  SampledSpectrum::SampledSpectrum(const std::span<const float>& v) {
    for (auto i = 0; i < kSpectrumSamples; ++i) {
      values[i] = v[i];
    }
  }

  SampledSpectrum::SampledSpectrum(float c) {
    values.fill(c);
  }

  float SampledSpectrum::operator[](size_t i) const {
    return values[i];
  }

  float& SampledSpectrum::operator[](size_t i) {
    return values[i];
  }
      
  float SampledSpectrum::Average() const {
    float sum = values[0];
    for (int32_t i = 1; i < kSpectrumSamples; ++i) {
      sum += values[i];
    }
    return sum / kSpectrumSamples;
  }

  SampledSpectrum SampledSpectrum::operator*(float a) const {
    SampledSpectrum ret = *this;
    for (int i = 0; i < kSpectrumSamples; ++i) {
      ret.values[i] *= a;
    }
    return ret;
  }

  SampledSpectrum& SampledSpectrum::operator*=(float a) {
    for (int i = 0; i < kSpectrumSamples; ++i) {
      values[i] *= a;
    }
    return *this;
  }

  SampledSpectrum SampledSpectrum::operator/(float a) const {
    SampledSpectrum ret = *this;
    return ret /= a;
  }

  SampledSpectrum& SampledSpectrum::operator/=(float a) {
    for (int32_t i = 0; i < kSpectrumSamples; ++i)
        values[i] /= a;
    return *this;
  }

  SampledSpectrum SampledSpectrum::operator*(const SampledSpectrum &s) const {
    SampledSpectrum ret = *this;
    return ret *= s;
  }

  SampledSpectrum& SampledSpectrum::operator*=(const SampledSpectrum &s) {
    for (int i = 0; i < kSpectrumSamples; ++i) {
      values[i] *= s.values[i];
    }
    return *this;
  }

  SampledSpectrum& SampledSpectrum::operator+=(const SampledSpectrum& s) {
    for (int32_t i = 0; i < kSpectrumSamples; ++i) {
      values[i] += s[i];
    }
    return *this;
  }
      
  XYZ SampledSpectrum::ToXYZ(const SampledWavelengths& lambda) {
    SampledSpectrum X = spectra::X()->SampleWavelengths(lambda); 
    SampledSpectrum Y = spectra::Y()->SampleWavelengths(lambda); 
    SampledSpectrum Z = spectra::Z()->SampleWavelengths(lambda); 

    SampledSpectrum pdf = lambda.PDF();

    return XYZ(
      SaveDiv(X * *this , pdf).Average() ,
      SaveDiv(Y * *this , pdf).Average() ,
      SaveDiv(Z * *this , pdf).Average()
    ) / kCIEYIntegral;
  }
      
  SampledSpectrum SampledSpectrum::SaveDiv(const SampledSpectrum& a , const SampledSpectrum& b) {
    return SampledSpectrum(
      std::array<const float , kSpectrumSamples>{
        ((b[0] != 0) ? a[0] / b[0] : 0.f) ,
        ((b[1] != 0) ? a[1] / b[1] : 0.f) ,
        ((b[2] != 0) ? a[2] / b[2] : 0.f) ,
        ((b[3] != 0) ? a[3] / b[3] : 0.f) ,
      }
    );
  }

  float SampledWavelengths::operator[](size_t i) const {
    return lambda[i];
  }

  float& SampledWavelengths::operator[](size_t i) {
    return lambda[i];
  }
      
  SampledSpectrum SampledWavelengths::PDF() const {
    return SampledSpectrum(pdf);
  }
      
  void SampledWavelengths::TerminateSecondary() {
    if (SecondaryTerminated()) {
      return;
    }

    for (int32_t i = 1; i < kSpectrumSamples; ++i) {
      pdf[i] = 0.f;
    }
    pdf[0] /= kSpectrumSamples;
  }
      
  bool SampledWavelengths::SecondaryTerminated() const {
    for (int32_t i = 1; i < kSpectrumSamples; ++i) {
      if (pdf[i] != 0.f) {
        return false;
      }
    }
    return true;
  }

  SampledWavelengths SampledWavelengths::SampledUniform(float u , float lmin , float lmax) {
    SampledWavelengths swl;
    swl.lambda[0] = Lerp(u , lmin , lmax);
    
    float delta = (lmax - lmin) / kSpectrumSamples;
    for (int32_t i = 1; i < kSpectrumSamples; ++i) {
      swl.lambda[i] = swl.lambda[i - 1] + delta;

      if (swl.lambda[i] > lmax) {
        swl.lambda[i] = lmin + (swl.lambda[i] - lmax);
      }
    }

    for (int32_t i = 0; i < kSpectrumSamples; ++i) {
      swl.pdf[i] = 1 / (lmax - lmin);
    }

    return swl;
  }

} // namespace pbr
