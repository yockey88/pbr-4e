/**
 * \file radiometry/color.cpp
 **/
#include "radiometry/color.hpp"
#include "radiometry/spectrum.hpp"

#include <iterator>
#include <algorithm>

namespace pbr {

  const RGBColorSpace* RGBColorSpace::srgb = nullptr;
  const RGBColorSpace* RGBColorSpace::dci_p3 = nullptr;
  const RGBColorSpace* RGBColorSpace::rec_2020 = nullptr;
  const RGBColorSpace* RGBColorSpace::aces2056_1 = nullptr;

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

  void Init() {
    x_inst = NewRef<DenselySampledSpectrum>(NewRef<ConstantSpectrum>(860));
    y_inst = NewRef<DenselySampledSpectrum>(NewRef<ConstantSpectrum>(570));
    z_inst = NewRef<DenselySampledSpectrum>(NewRef<ConstantSpectrum>(450));
  }
  
  void Cleanup() {
    x_inst = nullptr;
    y_inst = nullptr;
    z_inst = nullptr;
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

  float XYZ::operator[](int32_t c) const {
    if (c == 0) {
      return x;
    } else if (c == 1) {
      return y;
    } else {
      return z;
    }
  }

  float& XYZ::operator[](int32_t c) {
    if (c == 0) {
      return x;
    } else if (c == 1) {
      return y;
    } else {
      return z;
    }
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

  float RGB::operator[](int32_t c) const {
    if (c == 0) {
      return r;
    } else if (c == 1) {
      return g;
    } else {
      return b;
    }
  }

  float& RGB::operator[](int32_t c) {
    if (c == 0) {
      return r;
    } else if (c == 1) {
      return g;
    } else {
      return b;
    }
  }

  RGB RGB::operator/(float s) const {
    RGB ret = *this;
    return ret /= s;
  }

  RGB& RGB::operator/=(float s) {
    r /= s;
    g /= s;
    b /= s;
    return *this;
  }
      
  RGBColorSpace::RGBColorSpace(const Point2& r , const Point2& g , const Point2& b , const Ref<Spectrum>& illuminant , 
                               const RGBToSpectrumTable* rgb_table) 
      : r(r) , g(g) , b(b) , illuminant(illuminant) {
    XYZ W = XYZ::SpectrumToXYZ(illuminant);
    w = W.xy();
    XYZ R = XYZ::FromXY_Y(r);
    XYZ G = XYZ::FromXY_Y(g);
    XYZ B = XYZ::FromXY_Y(b);

    glm::mat3 rgb {
      R.x , G.x , B.x ,
      R.y , G.y , B.y ,
      R.z , G.z , B.z 
    };

    XYZ C = Mul<XYZ>(glm::inverse(rgb) , W);

    glm::mat3 diag {
      C[0] , 0    , 0    ,
      0    , C[1] , 0    ,
      0    , 0    , C[2]
    };
    xyz_from_rgb = rgb * diag;
    rgb_from_xyz = glm::inverse(xyz_from_rgb);

    Init();
  }

  bool RGBColorSpace::operator==(const RGBColorSpace& other) const {
    return (r == other.r && g == other.g && g == other.b && w == other.w && 
            rgb_to_spectrum == other.rgb_to_spectrum); 
  }

  bool RGBColorSpace::operator!=(const RGBColorSpace& other) const {
    return !(*this == other);
  }

  RGB RGBColorSpace::ToRGB(XYZ xyz) const {
    return Mul<RGB>(rgb_from_xyz , xyz);
  }

  XYZ RGBColorSpace::ToXYZ(RGB rgb) const {
    return Mul<XYZ>(xyz_from_rgb , rgb);
  }
      
  glm::mat3 RGBColorSpace::ConvertRGBColorSpace(const RGBColorSpace& from , const RGBColorSpace& to) {
    if (from == to) {
      return {};
    }
    return to.rgb_from_xyz * from.xyz_from_rgb;
  }
      
  RGBSigmoidPolynomial RGBColorSpace::ToRGBCoeffs(RGB rgb) const {
    return rgb_to_spectrum->Eval(ClampZero(rgb));
  }

  const RGBColorSpace* RGBColorSpace::GetName(const std::string& name) {
    std::string new_name;
    std::transform(name.begin() , name.end() , std::back_inserter(new_name) , ::tolower);
    
    if (name == "aces2065-1") {
      return aces2056_1;
    } else if (name == "rec2020") {
      return rec_2020;
    } else if (name == "dci-p3") {
      return dci_p3;
    } else if (name == "srgb") {
      return srgb;
    } else {
      return nullptr;
    }
  }

  const RGBColorSpace* RGBColorSpace::Lookup(const Point2& r , const Point2& g , const Point2& b , const Point2& w) {
    auto close_enough = [](const Point2& a , const Point2& b) {
      return ((a.x == b.x || glm::abs((a.x - b.x) / b.x) < 1e-3) &&
              (a.y == b.y || glm::abs((a.y - b.y) / b.y) < 1e-3));
    };

    for (const RGBColorSpace* s : { srgb , dci_p3 , rec_2020 , aces2056_1 }) {
      if (close_enough(r , s->r) && close_enough(g , s->g) && close_enough(b , s->b) && close_enough(w , s->w)) {
        return s;
      }
    }
    return nullptr;
  }
      
  void RGBColorSpace::Init() {
    // srgb = new RGBColorSpace(Point2(0.64f , 0.33f) , Point2() , Point2() , Point2() , 
    //                           GetNamedSpectrum("stdillum-D65") , RGBToSpectrumTable::srgb);
    // dci_p3 = new RGBColorSpace(Point2(0.64f , 0.33f) , Point2() , Point2() , Point2() , 
    //                            GetNamedSpectrum("stdillum-D65") , RGBToSpectrumTable::dci_p3);
    // rec_2020 = new RGBColorSpace(Point2(0.64f , 0.33f) , Point2() , Point2() , Point2() , 
    //                              GetNamedSpectrum("stdillum-D65") , RGBToSpectrumTable::rec_2020);
    // aces2056_1 = new RGBColorSpace(Point2(0.64f , 0.33f) , Point2() , Point2() , Point2() , 
    //                                GetNamedSpectrum("illum-acesD60") , RGBToSpectrumTable::aces2056_1);
  }
      
  void RGBColorSpace::Cleanup() {
    delete srgb;
    srgb = nullptr;
    delete dci_p3;
    dci_p3 = nullptr;
    delete rec_2020;
    rec_2020 = nullptr;
    delete aces2056_1;
    aces2056_1 = nullptr;
  }

  float RGBSigmoidPolynomial::Eval(float wavelength) const {
    return Sigmoid(EvaluatePolynomial(wavelength , c2 , c1 , c0));
  }
      
  float RGBSigmoidPolynomial::MaxValue() const {
    float res = glm::max(Eval(360) , Eval(830));
    float lambda = -c1 / (2 * c0);
    if (lambda >= 360 && lambda <= 830) {
      res = glm::max(res , Eval(lambda));
    }
    return res;
  }

  float RGBSigmoidPolynomial::Sigmoid(float x) {
    if (std::isinf(x)) {
      return x > 0 ?
        1.f : 0.f;
    }
    return 0.5f + x / (2 * glm::sqrt(1 + glm::pow(x , 2)));
  }
      
  RGBSigmoidPolynomial RGBToSpectrumTable::Eval(RGB rgb) const {
    if (rgb[0] == rgb[1] && rgb[1] == rgb[2]) {
      return RGBSigmoidPolynomial(0 , 0 , (rgb[0] - 0.5f) / glm::sqrt(rgb[0] * (1 - rgb[0])));
    }

    int32_t maxc = (rgb[0] > rgb[1]) ?
      ((rgb[0] > rgb[2]) ? 0 : 2) :
      ((rgb[1] > rgb[2]) ? 1 : 2);

    float z = rgb[maxc];
    float x = rgb[(maxc + 1) % 3] * (res - 1) / z;
    float y = rgb[(maxc + 2) % 3] * (res - 1) / z;

    int32_t xi = glm::min(static_cast<int32_t>(x) , res - 2);
    int32_t yi = glm::min(static_cast<int32_t>(y) , res - 2);
    int32_t zi = FindInterval(res , [&](int32_t i) { return z_nodes[i] < z; });

    float dx = x - xi;
    float dy = y - yi;
    float dz = z - zi;

    std::array<float , 3> c;
    for (int32_t i = 0; i < 3; ++i) {
      auto co = [&](int32_t dx , int32_t dy , int32_t dz) {
        return (*coeffs)[maxc][zi + dz][yi + dy][xi + dx][i];
      };

      c[i] = Lerp(dz , Lerp(dy , Lerp(dx , co(0 , 0 , 0) , co(1 , 0 , 0)) ,
                                 Lerp(dx , co(0 , 1 , 0) , co(1 , 1 , 0))) ,
                       Lerp(dy , Lerp(dx , co(0 , 0 , 1) , co(1 , 0 , 1)) ,
                                 Lerp(dx , co(0 , 1 , 1) , co(1 , 1 , 1))));
    }

    return RGBSigmoidPolynomial(c[0] , c[1] , c[2]);
  }
      
  RGBAlbedoSpectrum::RGBAlbedoSpectrum(const RGBColorSpace& cs , RGB rgb) {
    rsp = cs.ToRGBCoeffs(rgb);
  }
  
  float RGBAlbedoSpectrum::MaxValue() const {
    return rsp.MaxValue();
  }

  float RGBAlbedoSpectrum::Sample(float wavelength) const {
    return rsp.Eval(wavelength);
  }

  SampledSpectrum RGBAlbedoSpectrum::SampleWavelengths(const SampledWavelengths& wavelengths) const {
    SampledSpectrum s;
    for (int32_t i = 0; i < kSpectrumSamples; ++i) {
      s[i] = Sample(wavelengths[i]);
    }
    return s;
  }
      
  RGBUnboundedSpectrum::RGBUnboundedSpectrum(const RGBColorSpace& cs , RGB rgb) {
    float m = std::max({ rgb.r , rgb.g , rgb.b });
    scale = 2.f * m;
    rsp = cs.ToRGBCoeffs(scale ? rgb / scale : RGB(0 , 0 , 0));
  }

  float RGBUnboundedSpectrum::MaxValue() const {
    return scale * rsp.MaxValue();
  }

  float RGBUnboundedSpectrum::Sample(float wavelength) const {
    return scale * rsp.Eval(wavelength);
  }

  SampledSpectrum RGBUnboundedSpectrum::SampleWavelengths(const SampledWavelengths& wavelengths) const {
    SampledSpectrum s;
    for (int32_t i = 0; i < kSpectrumSamples; ++i) {
      s[i] = Sample(wavelengths[i]);
    }
    return s;
  }
  
  RGBIlluminantSpectrum::RGBIlluminantSpectrum(const RGBColorSpace& cs , RGB rgb) 
      : illuminant(cs.illuminant) {
    float m = std::max({ rgb.r , rgb.g , rgb.b });
    scale = 2.f * m;
    rsp = cs.ToRGBCoeffs(scale ? rgb / scale : RGB(0 , 0 , 0));
  }

  float RGBIlluminantSpectrum::MaxValue() const {
    return scale * rsp.MaxValue();
  }

  float RGBIlluminantSpectrum::Sample(float wavelength) const {
    return scale * rsp.Eval(wavelength);
  }

  SampledSpectrum RGBIlluminantSpectrum::SampleWavelengths(const SampledWavelengths& wavelengths) const {
    SampledSpectrum s;
    for (int32_t i = 0; i < kSpectrumSamples; ++i) {
      s[i] = Sample(wavelengths[i]);
    }
    return s;
  }

} // namespace pbr
