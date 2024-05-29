/**
 * \file radiometry/spectrum.hpp
 **/
#ifndef PBR_SPECTRUM_HPP
#define PBR_SPECTRUM_HPP

#include <span>
#include <vector>
#include <array>

#include "base/ref.hpp"

namespace pbr {

  class XYZ;
  
  class SampledSpectrum;
  class SampledWavelengths;

  constexpr static float kLightSpeed = 299792458.f;
  constexpr static float kPlancks = 6.62606957e-34f;
  constexpr static float kBoltzmann = 1.3806488e-23f;
  constexpr static float kStefanBoltzmann = 5.67032e-8f;
  constexpr static float kWiensDisplacement = 2.8977721e-3f;

  constexpr static float kLambdaMin = 360;
  constexpr static float kLambdaMax = 830;

  constexpr static uint32_t kSpectrumSamples = 4;

  float Blackbody(float wavelength , float t);

  class Spectrum : public RefCounted {
    public:
      virtual ~Spectrum() {}

      virtual float MaxValue() const = 0;
      virtual float Sample(float wavelength) const = 0;
      virtual SampledSpectrum SampleWavelengths(const SampledWavelengths& wavelengths) const  = 0;
  };

  class ConstantSpectrum : public Spectrum {
    public:
      ConstantSpectrum(float c)
        : c(c) {}

      virtual ~ConstantSpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;
      virtual SampledSpectrum SampleWavelengths(const SampledWavelengths& wavelengths) const override;

    private:
      float c;
  };

  class DenselySampledSpectrum : public Spectrum {
    public:
      DenselySampledSpectrum(Ref<Spectrum> spec , int32_t lmin = kLambdaMin , int32_t lmax = kLambdaMax);

      virtual ~DenselySampledSpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;
      virtual SampledSpectrum SampleWavelengths(const SampledWavelengths& wavelengths) const override;

    private:
      int32_t lambda_min , lambda_max;
      std::vector<float> values;
  };

  class PiecewiseLinearSpectrum : public Spectrum {
    public:
      PiecewiseLinearSpectrum(const std::span<const float> lambdas , const std::span<const float> values);
      
      virtual ~PiecewiseLinearSpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;
      virtual SampledSpectrum SampleWavelengths(const SampledWavelengths& wavelengths) const override;

    private:
      std::vector<float> lambdas;
      std::vector<float> values;
  };

  class BlackbodySpectrum : public Spectrum {
    public:
      BlackbodySpectrum(float temp);

      virtual ~BlackbodySpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;
      virtual SampledSpectrum SampleWavelengths(const SampledWavelengths& wavelengths) const override;

    private:
      float temp;
      float normalization_factor;
  };

  class SampledSpectrum {
    public:
      SampledSpectrum();
      SampledSpectrum(const std::span<const float>& values);
      explicit SampledSpectrum(float c);

      float operator[](size_t i) const;
      float& operator[](size_t i);

      float Average() const;

      SampledSpectrum operator*(float a) const;
      SampledSpectrum &operator*=(float a);

      SampledSpectrum operator/(float a) const;
      SampledSpectrum &operator/=(float a);

      SampledSpectrum operator*(const SampledSpectrum &s) const;
      SampledSpectrum& operator*=(const SampledSpectrum &s);

      SampledSpectrum& operator+=(const SampledSpectrum& s);

      XYZ ToXYZ(const SampledWavelengths& lambda);

      static SampledSpectrum SaveDiv(const SampledSpectrum& a , const SampledSpectrum& b);

    private:
      std::array<float , kSpectrumSamples> values;
  };

  class SampledWavelengths {
    public:
      float operator[](size_t i) const;
      float& operator[](size_t i);

      SampledSpectrum PDF() const;

      void TerminateSecondary();

      bool SecondaryTerminated() const;
      
      static SampledWavelengths SampledUniform(float u , float lmin = kLambdaMin , float lmax = kLambdaMax);

    private:
      std::array<float , kSpectrumSamples> lambda;
      std::array<float , kSpectrumSamples> pdf;
  };

} // namespace pbr

#endif // !PBR_SPECTRUM_HPP
