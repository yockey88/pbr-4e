/**
 * \file radiometry/spectrum.hpp
 **/
#ifndef PBR_SPECTRUM_HPP
#define PBR_SPECTRUM_HPP

#include <span>
#include <vector>

#include "base/ref.hpp"

namespace pbr {

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
  };

  class ConstantSpectrum : public Spectrum {
    public:
      ConstantSpectrum(float c)
        : c(c) {}

      virtual ~ConstantSpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;

    private:
      float c;
  };

  class DenselySampledSpectrum : public Spectrum {
    public:
      DenselySampledSpectrum(Ref<Spectrum> spec , int32_t lmin = kLambdaMin , int32_t lmax = kLambdaMax);

      virtual ~DenselySampledSpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;

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

    private:
      float temp;
      float normalization_factor;
  };

  class SampledSpectrum : public Spectrum {
    public:
    private:
  };

} // namespace pbr

#endif // !PBR_SPECTRUM_HPP
