/**
 * \file util/mesh.hpp
 **/
#ifndef PBR_MESH_HPP
#define PBR_MESH_HPP

#include "math/transform.hpp"
#include "math/piecewise_constant2D.hpp"

namespace pbr {

  class TriangleMesh {
    public:
      TriangleMesh(const Transform& render_from_object , bool reverse_orientation , 
                   std::vector<int32_t>& indices , std::vector<Point3>& p , 
                   std::vector<glm::vec3>& s , std::vector<glm::vec3>& normals , 
                   std::vector<Point2>& uv , std::vector<int32_t>& face_indices);

      int32_t num_triangles , num_vertices;
      const int32_t* vertex_indices = nullptr;
      const Point3* vertices = nullptr;
      const glm::vec3* normals = nullptr;
      const glm::vec3* s = nullptr;
      const Point2* uvs = nullptr;
      const int32_t* face_indices = nullptr;
      bool reverse_orientation , transform_swaps_handedness;

  };

  class BilinearPatchMesh {
    public:
      BilinearPatchMesh(const Transform& render_from_object , bool reverse_orientation , 
                        std::vector<int32_t>& indices , std::vector<Point3>& p ,
                        std::vector<glm::vec2>& s , std::vector<glm::vec3>& normals ,
                        std::vector<Point2>& uv , std::vector<int32_t>& face_indices);

      bool reverse_orientation , transform_swaps_indices;
      int32_t num_patches , num_vertices;
      const int32_t* vertex_indices = nullptr;
      const Point3* p = nullptr;
      const glm::vec3* n = nullptr;
      const Point2* uv = nullptr;
      const int32_t* face_indicies = nullptr;
      PiecewiseConstant2D* image_distribution;
  };

} // namespace pbr

#endif // !PBR_MESH_HPP
