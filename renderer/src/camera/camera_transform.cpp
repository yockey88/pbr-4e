/**
 * \file camera/camera_transform.cpp
 **/
#include "camera/camera_transform.hpp"
#include "math/transform.hpp"

#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/matrix_interpolation.hpp>

namespace pbr {
      
  CameraTransform::CameraTransform(const AnimatedTransform& world_from_camera) {
    ComputeCameraSpaceRendering(world_from_camera);
    ComputeCameraWorldSpaceRendering(world_from_camera);
    ComputeWorldSpaceRendering(world_from_camera);
    
    Transform render_from_world = glm::inverse(glm::identity<glm::mat4>());
    Transform rfc[2] = {
      render_from_world * world_from_camera.start_transform ,
      render_from_world * world_from_camera.end_transform
    };

    render_from_camera = AnimatedTransform(
      rfc[0] , world_from_camera.start_time , 
      rfc[1] , world_from_camera.end_time
    );
  }
      
  bool CameraTransform::CameraFromRenderHasScale() const {
    return render_from_camera.HasScale();
  }

  Point3 CameraTransform::RenderFromCamera(const Point3& p , float time) const {
    return render_from_camera.TransformPoint(p , time);
  }

  Point3 CameraTransform::CameraFromRender(const Point3& p , float time) const {
    return render_from_camera.ApplyInverse(p , time);
  }

  Point3 CameraTransform::RenderFromWorldCS(const Point3& p) const {
    return world_from_render_cs.ApplyInverse(p);
  }

  Point3 CameraTransform::RenderFromWorldCWS(const Point3& p) const {
    return world_from_render_cws.ApplyInverse(p);
  }

  Point3 CameraTransform::RenderFromWorldWS(const Point3& p) const {
    return world_from_render_ws.ApplyInverse(p);
  }

  Transform CameraTransform::RenderFromWorldCS() const {
    return Transform::Inverse(world_from_render_cs);
  }

  Transform CameraTransform::RenderFromWorldCWS() const {
    return Transform::Inverse(world_from_render_cws);
  }

  Transform CameraTransform::RenderFromWorldWS() const {
    return Transform::Inverse(world_from_render_ws);
  }
      
  Transform CameraTransform::CameraFromRender(float time) const {
    return Transform::Inverse(render_from_camera.Interpolate(time));
  }
      
  Transform CameraTransform::CameraFromWorldCS(float time) const {
    return Transform::Inverse(world_from_render_cs * render_from_camera.Interpolate(time));
  }

  Transform CameraTransform::CameraFromWorldCWS(float time) const {
    return Transform::Inverse(world_from_render_cws * render_from_camera.Interpolate(time));
  }

  Transform CameraTransform::CameraFromWorldWS(float time) const {
    return Transform::Inverse(world_from_render_ws * render_from_camera.Interpolate(time));
  }
      
  glm::vec3 CameraTransform::RenderFromCameraVector(const glm::vec3& v , float time) const {
    return render_from_camera.TransformVector(v , time);
  }
      
  glm::vec3 CameraTransform::RenderFromCameraNormal(const glm::vec3& n , float time) const {
    return render_from_camera.TransformNormal(n , time);
  }

  Ray CameraTransform::RenderFromCameraRay(const Ray& r) const {
    return render_from_camera.TransformRay(r); 
  }

  const AnimatedTransform& CameraTransform::RenderFromCamera() const {
    return render_from_camera;
  }

  const Transform& CameraTransform::WorldFromRenderCS() const {
    return world_from_render_cs;
  }

  const Transform& CameraTransform::WorldFromRenderCWS() const {
    return world_from_render_cws;
  }

  const Transform& CameraTransform::WorldFromRenderWS() const {
    return world_from_render_ws;
  }

  void CameraTransform::ComputeCameraSpaceRendering(const AnimatedTransform& world_from_camera) {
    float tmid = (world_from_camera.start_time + world_from_camera.end_time) / 2;
    world_from_render_cs = world_from_camera.Interpolate(tmid);
  }

  void CameraTransform::ComputeCameraWorldSpaceRendering(const AnimatedTransform& world_from_camera) {
    float tmid = (world_from_camera.start_time + world_from_camera.end_time) / 2;
    Point3 p_cam = world_from_camera.TransformPoint(Point3(0 , 0 , 0) , tmid);
    world_from_render_cws = Transform::Translate(p_cam);
  }

  void CameraTransform::ComputeWorldSpaceRendering(const AnimatedTransform& world_from_camera) {
    world_from_render_ws = glm::identity<glm::mat4>();
  }

} // namespace pbr
