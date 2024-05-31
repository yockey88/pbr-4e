/**
 * \file camera/camera.hpp
 **/
#ifndef PBR_CAMERA_HPP
#define PBR_CAMERA_HPP

#include "util/defines.hpp"
#include "util/ref.hpp"
#include "math/ray.hpp"
#include "math/transform.hpp"
#include "radiometry/spectrum.hpp"
#include "camera/camera_transform.hpp"
#include "camera/image.hpp"
#include "camera/film.hpp"

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


  Transform Orthographic(float near , float far);
  Transform Perspective(float fov , float n , float f);

  class Camera : RefCounted {
    public:
      Camera(const CameraParameters& p);

      virtual ~Camera() {}
      
      Ref<Film> GetFilm() const;

      const CameraTransform& GetCameraTransform() const;
      
      float SampleTime(float u) const;
      
      void InitMetadata(Ref<ImageMetadata> metadata) const;

      virtual Opt<CameraRayDifferential> GenerateRayDifferential(const CameraSample& sample , SampledWavelengths& lambda) const;

      virtual Opt<CameraRay> GenerateRay(const CameraSample& sample , SampledWavelengths& lambda) const = 0;

    protected:
      CameraTransform cam_transform;
      float shutter_open , shutter_close;
      Ref<Film> film = nullptr;
      Ref<Medium> medium = nullptr;

      glm::vec3 min_pos_differential_x , min_pos_differential_y;
      glm::vec3 min_dir_differential_x , min_dir_differential_y;

      glm::vec3 CameraFromRender(const glm::vec3& v , float time) const;
      Ray RenderFromCamera(const Ray& r) const;
      void FindMinimumDifferentials();

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
      glm::vec3 dx_camera , dy_camera;
      float lens_radius , focal_distance;
  };
  
  class OrthographicCamera : public ProjectiveCamera {
    public:
      OrthographicCamera(const CameraParameters& parameters , const Transform& screen_from_camera , 
                         const Bounds2& screen_window , float lens_radius , float focal_distance);

      virtual Opt<CameraRayDifferential> GenerateRayDifferential(const CameraSample& sample , SampledWavelengths& lambda) const override;

      virtual Opt<CameraRay> GenerateRay(const CameraSample& sample , SampledWavelengths& lambda) const override;

    private:
  };

  class PerspectiveCamera : public ProjectiveCamera {
    public:
      PerspectiveCamera(const CameraParameters& parameters , const Transform& screen_from_camera , 
                        const Bounds2& screen_window , float lens_radius , float focal_distance , float fov);
      
      virtual Opt<CameraRayDifferential> GenerateRayDifferential(const CameraSample& sample , SampledWavelengths& lambda) const override;

      virtual Opt<CameraRay> GenerateRay(const CameraSample& sample , SampledWavelengths& lambda) const override;

    private:
      float cos_total_width;
      float A;
  };

  class SphericalCamera : public Camera {
    public:
      enum class Mapping {
        EQUI_RECTANGULAR , 
        EQUAL_AREA
      };

      SphericalCamera(const CameraParameters& parameters , Mapping mapping);

      virtual Opt<CameraRay> GenerateRay(const CameraSample& sample , SampledWavelengths& lambda) const override;

    private:
      Mapping mapping;
  };

  class RealisticCamera : public Camera {
    public:
  };

} // namespace pbr

#endif // !PBR_CAMERA_HPP
