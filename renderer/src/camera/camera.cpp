/**
 * \file camera/camera.cpp
 **/
#include "camera/camera.hpp"

#include "math/sampling_functions.hpp"
#include "math/vecmath.hpp"
#include "radiometry/spectrum.hpp"

namespace pbr {
  
  Transform Orthographic(float near , float far) {
    return Transform::Scale(1 , 1 , 1 / (far - near)) * Transform::Translate(glm::vec3{ 0 , 0 , -near });
  }
  
  Transform Perspective(float fov , float n , float f) {
    glm::mat4 persp{
      1 , 0 ,           0 ,                0 ,
      0 , 1 ,           0 ,                0 ,
      0 , 0 , f / (f - n) , -f * n / (f - n) ,
      0 , 0 ,           1 ,                0
    };

    float inv_tan_ang = 1.f / glm::tan(glm::radians(fov) / 2);
    return Transform::Scale(inv_tan_ang , inv_tan_ang , 1) * Transform(persp);
  }

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
      
  void Camera::InitMetadata(Ref<ImageMetadata> metadata) const {
    // metadata->camera_from_world = cam_transform.CameraFromWorldCS(shutter_open).GetMatrix();
  }
      
  Opt<CameraRayDifferential> Camera::GenerateRayDifferential(const CameraSample& sample , SampledWavelengths& lambda) const {
    std::optional<CameraRay> cr = GenerateRay(sample , lambda);
    if (!cr.has_value()) {
      return std::nullopt;
    }

    RayDifferential rd(cr->ray);

    Opt<CameraRay> rx;
    for (float eps : { 0.05f , -0.05f }) {
      CameraSample sshift = sample;
      sshift.film.x += eps;
      
      if (rx = GenerateRay(sshift , lambda); rx.has_value()) {
        rd.SetRxOrigin(rd.Origin() + (rx->ray.Origin() - rd.Origin()) / eps);
        rd.SetRxDirection(rd.Direction() + (rx->ray.Direction() - rd.Direction()) / eps);
        break;
      }
    }

    Opt<CameraRay> ry;
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
      
  glm::vec3 Camera::CameraFromRender(const glm::vec3& v , float time) const {
    return cam_transform.CameraFromRender(v , time);
  }

  Ray Camera::RenderFromCamera(const Ray& r) const {
    return cam_transform.RenderFromCameraRay(r);
  }

  void Camera::FindMinimumDifferentials() {
    min_pos_differential_x = min_pos_differential_y =
      min_dir_differential_x = min_dir_differential_y = glm::vec3{ infinity , infinity , infinity };

    CameraSample sample;
    sample.lens = { 0.5f , 0.5f };
    sample.time = 0.5f;
    SampledWavelengths lambda = SampledWavelengths::SampleVisible(0.5);

    int32_t n = 512;
    for (int32_t i = 0; i < n; ++i) {
      sample.film.x = float(i) / (n - 1) * film->FullResolution().x;
      sample.film.y = float(i) / (n - 1) * film->FullResolution().y;

      Opt<CameraRayDifferential> crd = GenerateRayDifferential(sample , lambda);
      if (!crd.has_value()) {
        continue;
      }

      RayDifferential& ray = crd->ray;
      glm::vec3 dox = CameraFromRender(ray.RxOrigin() - ray.Origin() , ray.Time());
      if (dox.length() < min_pos_differential_x.length()) {
        min_pos_differential_x = dox;
      }

      glm::vec3 doy = CameraFromRender(ray.RyOrigin() - ray.Origin() , ray.Time());
      if (doy.length() < min_pos_differential_y.length()) {
        min_pos_differential_y = doy;
      }

      ray.SetDirection(Normalize(ray.Direction()));
      ray.SetRxDirection(Normalize(ray.RxDirection()));
      ray.SetRyDirection(Normalize(ray.RyDirection()));

      Frame f = Frame::FromZ(ray.Direction());
      glm::vec3 df = f.Local(ray.Direction());
      glm::vec3 dxf = Normalize(f.Local(ray.RxDirection()));
      glm::vec3 dyf = Normalize(f.Local(ray.RyDirection()));

      if ((dxf - df).length() < min_dir_differential_x.length()) {
        min_dir_differential_x = dxf - df;
      }

      if ((dyf - df).length() < min_dir_differential_y.length()) {
        min_dir_differential_y = dyf - df;
      }
    }
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

  OrthographicCamera::OrthographicCamera(const CameraParameters& parameters , const Transform& screen_from_camera , 
                                         const Bounds2& screen_window , float lens_radius , float focal_distance)
      : ProjectiveCamera(parameters , Orthographic(0 , 1) , screen_window , lens_radius , focal_distance) {
    dx_camera = camera_from_raster.TransformVector({ 1 , 0 , 0 });
    dy_camera = camera_from_raster.TransformVector({ 0 , 1 , 0 });
  } 
      
  Opt<CameraRayDifferential> OrthographicCamera::GenerateRayDifferential(const CameraSample& sample , SampledWavelengths& lambda) const {
    /// compute main orthographic viewing ray
    Point3 film = Point3(sample.film.x , sample.film.y , 0);
    Point3 camera = camera_from_raster.TransformPoint(film);

    RayDifferential ray(camera , glm::vec3(0 , 0 , 1) , SampleTime(sample.time) , medium);

    /// compute ray differentials for orthographic camera
    if (lens_radius > 0) {
      /// sample point on lens
      Point2 lens = lens_radius * SampleUniformDiskConcentric(sample.lens);

      /// compute point on plane of focus
      float ft = focal_distance / ray.Direction().z;
      Point3 focus = ray.At(ft);

      /// update ray for effect of lens
      ray.SetOrigin({ lens.x , lens.y , 0 });
      ray.SetDirection(Normalize(focus - ray.Origin()));

      focus = camera + dx_camera + (ft * glm::vec3{ 0 , 0 , 1 });
      ray.SetRxOrigin({ lens.x , lens.y , 0 });
      ray.SetRxDirection(Normalize(focus - ray.RxOrigin()));

      focus = camera + dy_camera + (ft * glm::vec3{ 0 , 0 , 1});
      ray.SetRyOrigin({ lens.x , lens.y , 0 });
      ray.SetRyDirection(Normalize(focus - ray.RyOrigin()));
    } else {
      ray.SetRxOrigin(ray.Origin() + dy_camera);
      ray.SetRyOrigin(ray.Origin() + dy_camera);
      ray.SetRyDirection(ray.Direction());
      ray.SetRxDirection(ray.Direction());
    }

    ray.SetHasDifferentials(true);
    return CameraRayDifferential{ RenderFromCamera(ray) };
  }
      
  Opt<CameraRay> OrthographicCamera::GenerateRay(const CameraSample& sample , SampledWavelengths& lambda) const {
    Point3 p_film = Point3(sample.film.x , sample.film.y , 0);
    Point3 p_cam = camera_from_raster.TransformVector(p_film);

    Ray ray(p_cam , glm::vec3{ 0 , 0 , 1 } , SampleTime(sample.time) , medium);

    if (lens_radius > 0) {
      /// compute raster for depth of field
      Point2 plens = lens_radius * SampleUniformDiskConcentric(sample.lens);
      
      /// compute point on plane of focus
      float ft = focal_distance / ray.Direction().z;
      Point3 p_focus = ray.At(ft);

      /// update ray for effect of lens
      ray.SetOrigin(Point3(plens.x , plens.y , 0));
      ray.SetDirection(Normalize(p_focus - ray.Origin()));
    }

    return CameraRay(RenderFromCamera(ray));
  }
      
  PerspectiveCamera::PerspectiveCamera(const CameraParameters& parameters , const Transform& screen_from_camera , 
                                       const Bounds2& screen_window , float lens_radius , float focal_distance , float fov) 
      : ProjectiveCamera(parameters , Perspective(fov , 1e-2f , 1000.f) , screen_window , lens_radius , focal_distance) {
    /// compute differential changes in origin for perspective camera rays
    dx_camera = camera_from_raster.TransformPoint({ 1 , 0 , 0 }) -
                camera_from_raster.TransformPoint({ 0 , 0 , 0 });
    dy_camera = camera_from_raster.TransformPoint({ 0 , 1 , 0 }) - 
                camera_from_raster.TransformPoint({ 0 , 0 , 0 });

    /// copmute cos_total_width for perspective cameras
    Point2 radius; // = Point2(film->GetFilter().Radius());
    Point3 corner(-radius.x , radius.y , 0.f);
    glm::vec3 corner_cam = Normalize(glm::vec3(camera_from_raster.TransformPoint(corner)));
    cos_total_width = corner_cam.z;

    /// compute image plane area at z = 1
    Point2 res = film->FullResolution();
    Point3 min = camera_from_raster.TransformPoint({ 0 , 0 , 0 });
    Point3 max = camera_from_raster.TransformPoint({ res.x , res.y , 0 });
    min /= min.z;
    max /= max.z;
    A = glm::abs((max.x - min.x) * (max.y - min.y));

    /// compute min differentials
    FindMinimumDifferentials();
  }

  Opt<CameraRayDifferential> PerspectiveCamera::GenerateRayDifferential(const CameraSample& sample , SampledWavelengths& lambda) const {
    /// compute offset rays differentials
    Point3 film = Point3(sample.film.x , sample.film.y , 0);
    Point3 camera = camera_from_raster.TransformPoint(film);

    glm::vec3 dir = Normalize(camera);
    RayDifferential ray(Point3(0 , 0 , 0) , dir , SampleTime(sample.time) , medium);

    if (lens_radius > 0) {
      /// sample point on lens
      Point2 lens = lens_radius * SampleUniformDiskConcentric(sample.lens);
      
      /// compute point on plane of focus
      float ft = focal_distance / ray.Direction().z;
      Point3 focus = ray.At(ft);

      /// update ray for effect of lens
      ray.SetOrigin({ lens.x , lens.y , 0 });
      ray.SetDirection(Normalize({ focus - ray.Origin() }));

      /// compute offset rays for differentials
      /// compute x ray differential
      glm::vec3 dx = Normalize(camera + dx_camera);
      float dx_ft = focal_distance / dx.z;
      focus = Point3(0 , 0 , 0) + (dx_ft * dx);

      ray.SetRxOrigin({ lens.x , lens.y , 0 });
      ray.SetRxDirection(focus - ray.RxOrigin());

      /// compute y differential
      glm::vec3 dy = Normalize(camera + dy_camera);
      float dy_ft = focal_distance / dy.z;
      focus = Point3(0 , 0 , 0) + (dy_ft * dy);
      ray.SetRyOrigin({ lens.x , lens.y , 0 });
      ray.SetRyDirection(Normalize(focus -  ray.RyOrigin()));
    } else {
      ray.SetRxOrigin(ray.Origin());
      ray.SetRxOrigin(ray.Origin());
      ray.SetRxDirection(Normalize(camera) + dx_camera);
      ray.SetRyDirection(Normalize(camera) + dy_camera);
    }

    ray.SetHasDifferentials(true);
    return CameraRayDifferential{ RenderFromCamera(ray) };
  }

  Opt<CameraRay> PerspectiveCamera::GenerateRay(const CameraSample& sample , SampledWavelengths& lambda) const {
    /// compute raster and camera sample positions
    Point3 film = Point3(sample.film.x , sample.film.y , 0);
    Point3 camera = camera_from_raster.TransformPoint(film);

    Ray ray(Point3(0 , 0 , 0) , Normalize(glm::vec3(camera)) , SampleTime(sample.time) , medium);

    /// modify ray for depth of field
    if (lens_radius > 0) {
      /// sample point on lens
      Point2 plens = lens_radius * SampleUniformDiskConcentric(sample.lens);

      /// compute point on plane of focus
      float ft = focal_distance / ray.Direction().z;
      Point3 focus = ray.At(ft);

      ray.SetOrigin(Point3(plens.x , plens.y , 0));
      ray.SetDirection(Normalize(focus - ray.Origin()));
    }

    return CameraRay(RenderFromCamera(ray));
  }
      
  SphericalCamera::SphericalCamera(const CameraParameters& parameters , Mapping mapping)
      : Camera(parameters) , mapping(mapping) {
    FindMinimumDifferentials();    
  }

  Opt<CameraRay> SphericalCamera::GenerateRay(const CameraSample& sample , SampledWavelengths& lambda) const {
    /// compute spherical camera ray direction
    Point2 uv(
      sample.film.x / film->FullResolution().x ,
      sample.film.y / film->FullResolution().y
    );

    glm::vec3 dir;
    if (mapping == Mapping::EQUI_RECTANGULAR) {
      /// compute ray direction using equirectangular mapping
      float theta = pi * uv[1];
      float phi = 2 * pi * uv[0];
      dir = SphericalDirection(glm::sin(theta) , glm::cos(theta) , phi);
    } else {
      uv = WrapEqualAreaSquare(uv);
      dir = EqualAreaSquareToSphere(uv);
    }
    std::swap(dir.y , dir.z);
    Ray ray(Point3(0 , 0 , 0) , dir , SampleTime(sample.time) , medium);

    return CameraRay{ RenderFromCamera(ray) };
  }

} // namespace pbr
