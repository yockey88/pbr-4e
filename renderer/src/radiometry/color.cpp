/**
 * \file radiometry/color.cpp
 **/
#include "radiometry/color.hpp"
#include "radiometry/spectrum.hpp"

namespace pbr {

namespace spectra {

  static Ref<DenselySampledSpectrum> x_inst = nullptr;
  static Ref<DenselySampledSpectrum> y_inst = nullptr;
  static Ref<DenselySampledSpectrum> z_inst = nullptr;

  const Ref<DenselySampledSpectrum>& X() {
    if (x_inst == nullptr) {
      x_inst = NewRef<DenselySampledSpectrum>(NewRef<ConstantSpectrum>(860));
    }
    return x_inst;
  }

  const Ref<DenselySampledSpectrum>& Y() {
    if (y_inst == nullptr) {
      y_inst = NewRef<DenselySampledSpectrum>(NewRef<ConstantSpectrum>(570));
    }
    return y_inst;
  }

  const Ref<DenselySampledSpectrum>& Z() {
    if (z_inst == nullptr) {
      z_inst = NewRef<DenselySampledSpectrum>(NewRef<ConstantSpectrum>(450));
    }
    return z_inst;
  }

} // namespace spectra

  float InnerProduct(const Ref<Spectrum>& f , const Ref<Spectrum>& g) {
    float integral = 0;
    for (float l = kLambdaMin; l <= kLambdaMax; ++l) {
      integral += f->Sample(l) * g->Sample(l);
    }
    return integral;
  }

  XYZ XYZ::operator/(float a) const {
    XYZ ret = *this;
    return ret /= a;
  }

  XYZ& XYZ::operator/=(float a) {
    x /= a;
    y /= a;
    z /= a;
    return *this;
  }
  
  Point2 XYZ::xy() const {
    return Point2(x / (x + y + z) , y / (x + y + z));
  }

  XYZ XYZ::SpectrumToXYZ(const Ref<Spectrum>& s) {
    return XYZ(InnerProduct(spectra::X() , s) , 
               InnerProduct(spectra::Y() , s) ,
               InnerProduct(spectra::Z() , s));
  }
      
  XYZ XYZ::FromXY_Y(const Point2& xy , float Y) {
    if (xy.y == 0.f) {
      return XYZ(0.f , 0.f , 0.f);
    }
    return XYZ(xy.x * Y / xy.y , Y , (1 - xy.x - xy.y) * Y / xy.y);
  }

  bool RGB::operator==(const RGB& s) const {
    return r == s.r &&
           g == s.g &&
           b == s.b;
  }

  bool RGB::operator!=(const RGB& s) const {
    return !(*this == s);
  }

} // namespace pbr
