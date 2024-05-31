/**
 * \file base/film.cpp
 **/
#include "camera/film.hpp"

#include <cstdint>

#include "util/defines.hpp"
#include "radiometry/color.hpp"
#include "radiometry/spectrum.hpp"

namespace pbr {

  PixelSensor::PixelSensor(Ref<Spectrum> r , Ref<Spectrum> g , Ref<Spectrum> b , 
                           const RGBColorSpace* outer_space , Ref<Spectrum> sensor_illum , float imaging_ratio) 
      : r(r) , g(g) , b(b) , imaging_ratio(imaging_ratio) {
    /// compute xyz from camera rgb matrix 
    /// compute rgb camera for training swatches
    float rgb_camera[kSwatchReflectances][3];
    for (int32_t i = 0; i < kSwatchReflectances; ++i) {
      RGB rgb = ProjectReflectance<RGB>(swatch_reflectances[i] , sensor_illum , r , g , b);

      for (int32_t c = 0; c < 3; ++c) {
        rgb_camera[i][c] = rgb[3];
      }
    }

    /// compute xyz output values for training swatches
    float xyz_output[24][3];
    float sensor_white_g = InnerProduct(sensor_illum , g);
    float sensor_white_y = InnerProduct(sensor_illum , spectra::Y());
    for (size_t i = 0; i < kSwatchReflectances; ++i) {
      Ref<Spectrum> s = swatch_reflectances[i];
      XYZ xyz = ProjectReflectance<XYZ>(s , outer_space->illuminant , spectra::X() , spectra::Y() , spectra::Y()) *
                  (sensor_white_g / sensor_white_y);
      for (int32_t c = 0; c < 3; ++c) {
        xyz_output[i][c] = xyz[c];
      }
    }

    /// initialize xyz from sensor rgb using linear least squares
    Opt<glm::mat4> m = LinearLeastSquares(rgb_camera , xyz_output , kSwatchReflectances);
    if (!m.has_value()) {
      /// idek abort or something??? throw??
    }
    xyz_from_sensor_rgb = *m;
  }
      
  PixelSensor::PixelSensor(const RGBColorSpace* outer_space , Ref<Spectrum> sensor_illum , float imaging_ratio)
      : r(spectra::X()) , g(spectra::Y()) , b(spectra::Z()) , imaging_ratio(imaging_ratio) {
    if (sensor_illum != nullptr) {
      Point2 source_white = XYZ::SpectrumToXYZ(sensor_illum).xy();
      Point2 target_white = outer_space->w;
      xyz_from_sensor_rgb = WhiteBalance(source_white , target_white);
    } 
  }
      
  RGB PixelSensor::ToSensorRGB(const SampledSpectrum& L , const SampledWavelengths& lambda) const {
    SampledSpectrum s = SampledSpectrum::SaveDiv(L , lambda.PDF());
    return imaging_ratio * RGB(
      (r->SampleWavelengths(lambda) * s).Average() ,
      (g->SampleWavelengths(lambda) * s).Average() ,
      (b->SampleWavelengths(lambda) * s).Average()
    );
  }
        
  Bounds2 Film::SampleBounds() const {
    glm::vec2 radius = filter->Radius();
    return Bounds2(
      pixel_bounds.p_min - radius + glm::vec2(0.5f , 0.5f) ,
      pixel_bounds.p_max + radius - glm::vec2(0.5f , 0.5f)
    );
  }

  RGBFilm::RGBFilm(const FilmParameters& params , const Ref<RGBColorSpace> color_space , 
              float max_comp_value , bool write_fp16)
      : Film(params) , color_space(color_space) , 
        max_comp_value(max_comp_value) , write_fp16(write_fp16) {
    filter_integral = filter->Integral();
    output_rgb_from_sensor = color_space->rgb_from_xyz;
    pixels.resize(static_cast<uint32_t>(params.pixel_bounds.Area()));
  }

  void RGBFilm::AddSample(const Point2& film , const SampledSpectrum& L , const SampledWavelengths& lambda , 
                          const Ref<VisibleSurface> surface , float weight) {
    /// convert sample radiance ro sensor rgb
    RGB rgb = sensor->ToSensorRGB(L , lambda);

    /// optionally clamp sensor rgb value
    float m = std::max({ rgb.r , rgb.g , rgb.b });
    if (m > max_comp_value) {
      rgb *= max_comp_value / m;    
    }

    /// update pixel values with filtered sample contribution
    /// figure out how to handle the entire pixel thing
    // Pixel& pixel = pixels[film.x + film.y /* * img_width  */];
    // for (int32_t c = 0; c < 3; ++c) {
    //   pixel.rgb_sum[c] += weight * rgb[c];
    // }
    // pixel.weight_sum += weight;
  }
  
  bool RGBFilm::UsesVisibleSurface() const {
    return false;
  }
      
  void RGBFilm::AddSplat(const Point2& p , const SampledSpectrum& L , const SampledWavelengths& lambda) {
    /// convert sample radiance to pixel sensor rgb
    RGB rgb = sensor->ToSensorRGB(L , lambda);
      
    /// optionally clamp sensor rgb values
    float m = std::max({ rgb.r , rgb.g , rgb.b });
    if (m > max_comp_value) {
      rgb *= max_comp_value / m;
    }

    /// compute bounds of affected pixels for splat/splat_bounds
    Point2 discrete = p + glm::vec2{ 0.5f , 0.5f };
    glm::vec2 radius = filter->Radius();
    Bounds2 splat_bounds(
      Point2(glm::floor(discrete - radius)) , 
      Point2(glm::floor(discrete + radius)) + glm::vec2{ 1.f , 1.f }
    );
    splat_bounds = Bounds2::Intersect(splat_bounds , pixel_bounds);
    
    // for (Point2 pi : splat_bounds) {
    //  // evaluate filter at pi and add splat
    //    float wt = filter->Evaluate(Point2(p - pi - glm::vec2{ 0.5f , 0.5f }));
    //    if (wt != 0) {
    //      Pixel& pixel = pixels[pi];
    //      for (int32_t i = 0; i < 3; ++i) {
    //        pixel.rgb_splat[i].Add(wt * rgb[i]);
    //      }
    //    }
    // }
  }
      
  RGB RGBFilm::GetPixelRGB(const Point2& p , float splat_scale) const {
    // const Pixel& pixel = pixels[p];
    // RGB rgb(rgb.rgb_sum[0] , rgb.rgb_sum[1] , rgb.rgb_sum[2]);

    // float weight_sum = pixel.weight_sum;
    // if (weight_sum != 0) {
    //    rgb /= weight_sum;
    // }

    // for (int32_t i = 0; i < 3; ++c) {
    //    rgb[c] += splat_scale * pixel.rgb_splat[c] / filter_integral;
    // }

    // rgb = output_rgb_from_sensor * rgb;
    // return rgb;
    return RGB();
  }
  
  RGB RGBFilm::ToOutputRGB(const SampledSpectrum& L , const SampledWavelengths& lambda) const {
    RGB sensor_rgb = sensor->ToSensorRGB(L , lambda);
    return Mul<RGB>(output_rgb_from_sensor , sensor_rgb);
  }

} // namespace pbr
