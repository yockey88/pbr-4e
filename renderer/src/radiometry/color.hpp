/**
 * \file radiometry/color.hpp
 **/
#ifndef PBR_COLOR_HPP
#define PBR_COLOR_HPP

#include "math/vecmath.hpp"
#include "radiometry/spectrum.hpp"

namespace pbr {

  constexpr static float kCIEYIntegral = 106.856895f;

namespace spectra {

  const Ref<DenselySampledSpectrum>& X();
  const Ref<DenselySampledSpectrum>& Y();
  const Ref<DenselySampledSpectrum>& Z();

} // namespace spectra

  float InnerProduct(const Ref<Spectrum>& f , const Ref<Spectrum>& g);

  class XYZ {
    public:
      XYZ(float x , float y , float z)
        : x(x) , y(y) , z(z) {}

      XYZ operator/(float a) const;
      XYZ& operator/=(float a);

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

      float r , g , b;
  };

  class RGBColorSpace {
    public:
    private:
  };

} // namespace pbr

#endif // !PBR_COLOR_HPP
