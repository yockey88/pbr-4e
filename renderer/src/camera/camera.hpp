/**
 * \file camera/camera.hpp
 **/
#ifndef PBR_CAMERA_HPP
#define PBR_CAMERA_HPP

#include <optional>

#include "base/ref.hpp"
#include "math/vecmath.hpp"
#include "math/ray.hpp"
#include "math/transform.hpp"
#include "radiometry/spectrum.hpp"
#include "camera/film.hpp"
#include "camera_transform.hpp"

namespace pbr {

  struct CameraSample {
    Point2 film;
    Point2 lens;
    float time = 0;
    float filter_weight = 1;
  };

  struct CameraRay {
    Ray ray;
    SampledSpectrum weight = SampledSpectrum(1);
  };

  struct CameraRayDifferential {
    RayDifferential ray;
    SampledSpectrum weight = SampledSpectrum(1);
  };

  struct CameraParameters {
    CameraTransform cam_transform;
    float shutter_open = 0 , shutter_close = 1;
    Ref<Film> film;
    Ref<Medium> medium;
  };

  struct ImageMetadata : public RefCounted {};

  class Camera : RefCounted {
    public:
      Camera(const CameraParameters& p);

      virtual ~Camera() {}
      
      Ref<Film> GetFilm() const;

      const CameraTransform& GetCameraTransform() const;
      
      float SampleTime(float u) const;

      virtual std::optional<CameraRayDifferential> GenerateRayDifferential(const CameraSample& sample , SampledWavelengths& lambda) const;

      virtual std::optional<CameraRay> GenerateRay(const CameraSample& sample , SampledWavelengths& lambda) const = 0;
      virtual void InitMetadata(Ref<ImageMetadata> metadata) const = 0;

    protected:
      CameraTransform cam_transform;
      float shutter_open , shutter_close;
      Ref<Film> film = nullptr;
      Ref<Medium> medium = nullptr;

      Ray RenderFromCamera(const Ray& r) const;

    private:
      AnimatedTransform render_from_camera;
      Transform world_from_render;
  }; 

  class ProjectiveCamera : public Camera {
    public:
      ProjectiveCamera(const CameraParameters& parameters , const Transform& screen_from_camera , 
                       const Bounds2& screen_window , float lens_radius , float focal_distance);

    protected:
      Transform screen_from_camera , camera_from_raster;
      Transform raster_from_screen , screen_from_raster;
      float lens_radius , focal_distance;
  };

  class PerspectiveCamera : public ProjectiveCamera {
    public:
  };

  class OrthographicCamera : public ProjectiveCamera {
    public:
  };

  class SphericalCamera : public Camera {
    public:
  };

  class RealisticCamera : public Camera {
    public:
  };

} // namespace pbr

#endif // !PBR_CAMERA_HPP
