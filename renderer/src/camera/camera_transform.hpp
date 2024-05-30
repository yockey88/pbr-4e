/**
 * \file camera/camera_transform.hpp
 **/
#ifndef PBR_CAMERA_TRANSFORM_HPP
#define PBR_CAMERA_TRANSFORM_HPP

#include "math/transform.hpp"

namespace pbr {
  
  class CameraTransform {
    public:
      CameraTransform() {}
      CameraTransform(const AnimatedTransform& world_from_camera);

      bool CameraFromRenderHasScale() const;

      Point3 RenderFromCamera(const Point3& p , float time) const;
      Point3 CameraFromRender(const Point3& p , float time) const;

      Point3 RenderFromWorldCS(const Point3& p) const;
      Point3 RenderFromWorldCWS(const Point3& p) const;
      Point3 RenderFromWorldWS(const Point3& p) const;

      Transform RenderFromWorldCS() const;
      Transform RenderFromWorldCWS() const;
      Transform RenderFromWorldWS() const;

      Transform CameraFromRender(float time) const;

      Transform CameraFromWorldCS(float time) const;
      Transform CameraFromWorldCWS(float time) const;
      Transform CameraFromWorldWS(float time) const;

      glm::vec3 RenderFromCameraVector(const glm::vec3& v , float time) const;
      glm::vec3 RenderFromCameraNormal(const glm::vec3& n , float time) const;

      Ray RenderFromCameraRay(const Ray& r) const;

      const AnimatedTransform& RenderFromCamera() const;

      const Transform& WorldFromRenderCS() const;
      const Transform& WorldFromRenderCWS() const;
      const Transform& WorldFromRenderWS() const;

    private:
      Transform world_from_render_cs;
      Transform world_from_render_cws;
      Transform world_from_render_ws;

      AnimatedTransform render_from_camera;

      void ComputeCameraSpaceRendering(const AnimatedTransform& world_from_camera);
      void ComputeCameraWorldSpaceRendering(const AnimatedTransform& world_from_camera);
      void ComputeWorldSpaceRendering(const AnimatedTransform& world_from_camera);
  };

} // namespace pbr

#endif // !PBR_CAMERA_TRANSFORM_HPP
