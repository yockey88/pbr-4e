/**
 * \file camera/film.hpp
 **/
#ifndef PBR_FILM_HPP
#define PBR_FILM_HPP

#include <vector>

#include "util/ref.hpp"
#include "math/math.hpp"
#include "math/bounds.hpp"
#include "math/interaction.hpp"
#include "radiometry/spectrum.hpp"
#include "camera/image.hpp"
#include "camera/filter.hpp"

namespace pbr {
  
  class PixelSensor : public RefCounted {
    public:
      PixelSensor(Ref<Spectrum> r , Ref<Spectrum> g , Ref<Spectrum> b , 
                  const RGBColorSpace* outer_space , Ref<Spectrum> sensor_illum , float imaging_ratio);
      PixelSensor(const RGBColorSpace* outer_space , Ref<Spectrum> sensor_illum , float imaging_ratio);

      RGB ToSensorRGB(const SampledSpectrum& L , const SampledWavelengths& lambda) const;

    private:
      constexpr static int32_t kSwatchReflectances = 24;
      static Ref<Spectrum> swatch_reflectances[kSwatchReflectances];

      Ref<DenselySampledSpectrum> r , g , b;
      float imaging_ratio;

      glm::mat4 xyz_from_sensor_rgb;
      
      template <typename T>
      inline T ProjectReflectance(Ref<Spectrum> refl , Ref<Spectrum> illum , 
                                  Ref<Spectrum> b1 , Ref<Spectrum> b2 , Ref<Spectrum> b3) {
        T result;
        float integral = 0;
        for (float l = kLambdaMin; l < kLambdaMax; ++l) {
          integral += b2->Sample(l) * illum->Sample(l);
          result[0] += b1->Sample(l) * refl->Sample(l) * illum->Sample(l);
          result[0] += b2->Sample(l) * refl->Sample(l) * illum->Sample(l);
          result[0] += b3->Sample(l) * refl->Sample(l) * illum->Sample(l);
        }
        return result / integral;
      }
  };

  class VisibleSurface : public RefCounted {
    public:
      VisibleSurface(const SurfaceInteraction& si , const SampledSpectrum& albedo , const SampledWavelengths& lambda);

      Point2 uv;
      Point3 p;
      glm::vec3 n , ns;
      float time = 0;
      glm::vec3 dpdx , dpdy;
      SampledSpectrum albedo;
      bool set = false;
  };

  struct FilmParameters {
    Point2 full_resolution;
    Bounds2 pixel_bounds;
    Ref<Filter> filter;
    float diagonal;
    Ref<PixelSensor> sensor;
    std::string filename;
  };

  class Film : public RefCounted {
    public:
      Film(const FilmParameters& p) 
        : full_resolution(p.full_resolution) , pixel_bounds(p.pixel_bounds) , filter(p.filter) , 
          diagonal(p.diagonal * 0.001f) , sensor(p.sensor) , filename(p.filename) {}
      
      virtual bool UsesVisibleSurface() const = 0;
      
      virtual Bounds2 SampleBounds() const;

      virtual void AddSample(const Point2& film , const SampledSpectrum& L , 
                             const SampledWavelengths& lambda , const Ref<VisibleSurface> surface , float weight) = 0;

      virtual RGB GetPixelRGB(const Point2& p , float splat_scale) const = 0;
      virtual RGB ToOutputRGB(const SampledSpectrum& L , const SampledWavelengths& lamnbda) const = 0;

      virtual void WriteImage(Ref<ImageMetadata> metadata , float splat_scale = 1) = 0;
      virtual Image GetImage(Ref<ImageMetadata> metadata , float splat_scale = 1) = 0;

      
      virtual float Diagonal() const = 0;
      
      virtual Point2 FullResolution() const = 0; 

      virtual Bounds2 PixelBounds() const = 0;

      virtual Filter GetFilter() const = 0;

      virtual const Ref<PixelSensor> GetPixelSensor() const = 0;

      virtual SampledWavelengths SampleWavelengths(float u) const = 0;

    protected:
      Point2 full_resolution;
      Bounds2 pixel_bounds;
      Ref<Filter> filter;
      float diagonal;
      const Ref<PixelSensor> sensor;
      std::string filename;
  };

  class RGBFilm : public Film {
    public:
      RGBFilm(const FilmParameters& params , const Ref<RGBColorSpace> color_space , 
              float max_comp_value , bool write_fp16);
      
      void AddSplat(const Point2& p , const SampledSpectrum& L , const SampledWavelengths& lambda);

      virtual void AddSample(const Point2& film , const SampledSpectrum& L , const SampledWavelengths& lambda , 
                             const Ref<VisibleSurface> surface , float weight) override;

      virtual bool UsesVisibleSurface() const override;

      virtual RGB GetPixelRGB(const Point2& p , float splat_scale = 1) const override;;
      virtual RGB ToOutputRGB(const SampledSpectrum& L , const SampledWavelengths& lambda) const override;

    private:
      struct Pixel {
        double rgb_sum[3] = { 0.f , 0.f , 0.f };
        double weight_sum = 0.f;

        /// write a wrapper to store atomic pixels here
        ///     need to be atomic because rgb_splat accessed by multiple threads when 
        ///     sampling scene light rays
        // std::atomic<double> rgb_splat[3];
      };

      const Ref<RGBColorSpace> color_space = nullptr;
      float max_comp_value;
      bool write_fp16;
      float filter_integral;
      glm::mat3 output_rgb_from_sensor;

      std::vector<Pixel> pixels;
  };

  class GBufferFilm : public Film {
    public:
    private:
      struct Pixel {
        double rgb_sum[3] = { 0.f , 0.f , 0.f };
        double weight_sum = 0.f , gbuffer_weight_sum = 0.f;
        // std::atomic<double> rgb_splat[3];
        Point3 sum;
        float dzdx_sum = 0.f , dzdy_sum = 0.f;
        glm::vec3 n_sum , ns_sum;
        Point2 uv_sum;
        double rgb_albedo_sum[3] = { 0.f , 0.f , 0.f };
        // VarianceEstimator<float> rgb_variance[3];
      };
  };

  class SpectralFilm : public Film {};

} // namespace pbr

#endif // !PBR_FILM_HPP
