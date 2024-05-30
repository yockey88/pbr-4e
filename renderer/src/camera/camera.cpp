/**
 * \file camera/camera.cpp
 **/
#include "camera/camera.hpp"

namespace pbr {

  Camera::Camera(const CameraParameters& p) {
    cam_transform = p.cam_transform;
    shutter_open = p.shutter_open;
    shutter_close = p.shutter_close;
    film = p.film;
    medium = p.medium;
  }

  Ref<Film> Camera::GetFilm() const {
    return film;
  }
      
  const CameraTransform& Camera::GetCameraTransform() const {
    return cam_transform;
  }
      
  float Camera::SampleTime(float u) const {
    return Lerp(u , shutter_open , shutter_close);
  }
      
  std::optional<CameraRayDifferential> Camera::GenerateRayDifferential(const CameraSample& sample , SampledWavelengths& lambda) const {
    std::optional<CameraRay> cr = GenerateRay(sample , lambda);
    if (!cr.has_value()) {
      return std::nullopt;
    }

    RayDifferential rd(cr->ray);

    std::optional<CameraRay> rx;
    for (float eps : { 0.05f , -0.05f }) {
      CameraSample sshift = sample;
      sshift.film.x += eps;
      
      if (rx = GenerateRay(sshift , lambda); rx.has_value()) {
        rd.SetRxOrigin(rd.Origin() + (rx->ray.Origin() - rd.Origin()) / eps);
        rd.SetRxDirection(rd.Direction() + (rx->ray.Direction() - rd.Direction()) / eps);
        break;
      }
    }

    std::optional<CameraRay> ry;
    for (float eps : { 0.05f , -0.05f }) {
      CameraSample sshift = sample;
      sshift.film.y += eps;
      
      if (ry = GenerateRay(sshift , lambda); ry.has_value()) {
        rd.SetRyOrigin(rd.Origin() + (ry->ray.Origin() - rd.Origin()) / eps);
        rd.SetRyDirection(rd.Direction() + (ry->ray.Direction() - rd.Direction()) / eps);
        break;
      }
    }

    rd.SetHasDifferentials(rx.has_value() && ry.has_value());
    return CameraRayDifferential{
      rd ,
      cr->weight
    };
  }

  Ray Camera::RenderFromCamera(const Ray& r) const {
    return cam_transform.RenderFromCameraRay(r);
  }
      
  ProjectiveCamera::ProjectiveCamera(const CameraParameters& parameters , const Transform& screen_from_camera , 
                                     const Bounds2& screen_window , float lens_radius , float focal_distance) 
      : Camera(parameters) , screen_from_camera(screen_from_camera) , lens_radius(lens_radius) , focal_distance(focal_distance) {
    Transform NDC_from_screen = Transform::Scale({ 
      1 / (screen_window.p_max.x - screen_window.p_min.x) ,
      1 / (screen_window.p_max.y - screen_window.p_min.y) , 
      1 
    }) * Transform::Translate({ -screen_window.p_min.x , -screen_window.p_max.y , 0 });
    Transform raster_from_NDC = Transform::Scale({ film->FullResolution().x , -film->FullResolution().y , 1 });

    raster_from_screen = raster_from_NDC * NDC_from_screen;
    screen_from_raster = Transform::Inverse(raster_from_screen);
    camera_from_raster = Transform::Inverse(screen_from_camera) * screen_from_raster;
  }

} // namespace pbr
