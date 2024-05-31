/**
 * \file radiometry/color.hpp
 **/
#ifndef PBR_COLOR_HPP
#define PBR_COLOR_HPP

#include <string>

#include "math/vecmath.hpp"
#include "radiometry/spectrum.hpp"

namespace pbr {

  constexpr static float kCIEYIntegral = 106.856895f;

namespace spectra {

  const Ref<DenselySampledSpectrum>& X();
  const Ref<DenselySampledSpectrum>& Y();
  const Ref<DenselySampledSpectrum>& Z();

  void Init();
  void Cleanup();

} // namespace spectra

  float InnerProduct(const Ref<Spectrum>& f , const Ref<Spectrum>& g);

  glm::mat3 WhiteBalance(const Point2& src_w , const Point2& target_w);

  class XYZ {
    public:
      XYZ()
        : x(0.f) , y(0.f) , z(0.f) {}
      XYZ(float x , float y , float z)
        : x(x) , y(y) , z(z) {}

      XYZ operator/(float a) const;
      XYZ& operator/=(float a);

      float operator[](int32_t c) const;
      float& operator[](int32_t c);

      XYZ operator*(float a) const;
      XYZ &operator*=(float a);


      Point2 xy() const;

      static XYZ SpectrumToXYZ(const Ref<Spectrum>& s);
      static XYZ FromXY_Y(const Point2& xy , float Y = 1.f);

      float x , y , z;
  };

  class RGB {
    public:
      RGB()
        : r(0.f) , g(0.f) , b(0.f) {}
      RGB(float r , float g , float b)
        : r(r) , g(g) , b(b) {}

      bool operator==(const RGB& s) const;
      bool operator!=(const RGB& s) const;

      float operator[](int32_t c) const;
      float& operator[](int32_t c);

      RGB operator/(float s) const;
      RGB& operator/=(float s);
      
      RGB operator*(float s) const;
      RGB& operator*=(float s);

      friend RGB operator*(float a , RGB s) { return s * a; }

      float r , g , b;
  };
  
  template <typename U , typename V>
  inline RGB Clamp(RGB rgb , U min , V max) {
    return RGB(
      Clamp(rgb.r , min , max) ,
      Clamp(rgb.g , min , max) ,
      Clamp(rgb.b , min , max)
    );
  }

  inline RGB ClampZero(RGB rgb) {
    return RGB(
      std::max<float>(0 , rgb.r) ,
      std::max<float>(0 , rgb.g) ,
      std::max<float>(0 , rgb.b)
    );
  }

  class RGBSigmoidPolynomial;
  class RGBToSpectrumTable;

  class RGBColorSpace : public RefCounted {
    public:
      RGBColorSpace(const Point2& r , const Point2& g , const Point2& b , const Ref<Spectrum>& illuminant , 
                    const RGBToSpectrumTable* rgb_table);

      bool operator==(const RGBColorSpace& other) const;
      bool operator!=(const RGBColorSpace& other) const;

      RGB ToRGB(XYZ xyz) const;
      XYZ ToXYZ(RGB rgb) const;
      
      glm::mat3 ConvertRGBColorSpace(const RGBColorSpace& from , const RGBColorSpace& to);

      RGBSigmoidPolynomial ToRGBCoeffs(RGB rgb) const;

      static const RGBColorSpace* GetName(const std::string& name);
      static const RGBColorSpace* Lookup(const Point2& r , const Point2& g , const Point2& b , const Point2& w);
      
      static void Init();
      static void Cleanup();

      Point2 r , g , b , w;
      Ref<DenselySampledSpectrum> illuminant = nullptr;

      static const RGBColorSpace* srgb;
      static const RGBColorSpace* dci_p3;
      static const RGBColorSpace* rec_2020;
      static const RGBColorSpace* aces2056_1;

      glm::mat3 xyz_from_rgb , rgb_from_xyz;

    private:
      RGBToSpectrumTable* rgb_to_spectrum = nullptr;
  };

  class RGBSigmoidPolynomial {
    public:
      RGBSigmoidPolynomial() {}
      RGBSigmoidPolynomial(float c0 , float c1 , float c2)
        : c0(c0) , c1(c1) , c2(c2) {}

      float Eval(float wavelength) const;

      float MaxValue() const;

      static float Sigmoid(float lamda);

    private:
      float c0 , c1 , c2;
  };

  class RGBToSpectrumTable {
    public:
      constexpr static int32_t res = 64;
      using CoeffientArray = float[3][res][res][res][3];

      RGBToSpectrumTable(const float* z_nodes , const CoeffientArray* coeffs)
        : z_nodes(z_nodes) , coeffs(coeffs) {}

      RGBSigmoidPolynomial Eval(RGB rgb) const;

    private:
      const float* z_nodes = nullptr;
      const CoeffientArray* coeffs;
  };

  class RGBAlbedoSpectrum : public Spectrum {
    public:
      RGBAlbedoSpectrum(const RGBColorSpace& cs , RGB rgb);

      virtual ~RGBAlbedoSpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;
      virtual SampledSpectrum SampleWavelengths(const SampledWavelengths& wavelengths) const override;

    private:
      RGBSigmoidPolynomial rsp;
  };

  class RGBUnboundedSpectrum : public Spectrum {
    public:
      RGBUnboundedSpectrum(const RGBColorSpace& cs , RGB rgb);
      
      virtual ~RGBUnboundedSpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;
      virtual SampledSpectrum SampleWavelengths(const SampledWavelengths& wavelengths) const override;

    private:
      float scale = 1.f;
      RGBSigmoidPolynomial rsp;
  };

  class RGBIlluminantSpectrum : public Spectrum {
    public:
      RGBIlluminantSpectrum(const RGBColorSpace& cs , RGB rgb);

      virtual ~RGBIlluminantSpectrum() override {}

      virtual float MaxValue() const override;
      virtual float Sample(float wavelength) const override;
      virtual SampledSpectrum SampleWavelengths(const SampledWavelengths& wavelengths) const override;

    private:
      float scale;
      RGBSigmoidPolynomial rsp;
      const Ref<DenselySampledSpectrum> illuminant;
  };

} // namespace pbr

#endif // !PBR_COLOR_HPP
