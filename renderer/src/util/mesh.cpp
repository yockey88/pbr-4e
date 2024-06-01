/**
 * \file util/mesh.cpp
 **/
#include "util/mesh.hpp"

#include "util/buffer_cache.hpp"

namespace pbr {

  TriangleMesh::TriangleMesh(const Transform& render_from_object , bool reverse_orientation , 
                             std::vector<int32_t>& indices , std::vector<Point3>& p , 
                             std::vector<glm::vec3>& s , std::vector<glm::vec3>& n , 
                             std::vector<Point2>& uv , std::vector<int32_t>& face_indices) 
      : num_triangles(indices.size() / 3) , num_vertices(p.size()) , reverse_orientation(reverse_orientation) {
    vertex_indices = int_buffer_cache->LookupOrAdd(indices);

    for (Point3& pt : p) {
      pt = render_from_object.TransformPoint(pt);
    }

    vertices = point3_buffer_cache->LookupOrAdd(p);

    transform_swaps_handedness = render_from_object.SwapsHandedness();

    if (!uv.empty()) {
      uvs = point2_buffer_cache->LookupOrAdd(uv);
    }

    if (!n.empty()) {
      for (glm::vec3& nn : n) {
        nn = render_from_object.TransformNormal(nn);
        if (reverse_orientation) {
          nn = -nn;
        }
      }
      normals = normal3_buffer_cache->LookupOrAdd(n);
    }

    if (!s.empty()) {
      for (glm::vec3& ss : s) {
        ss = render_from_object.TransformVector(ss);
      }
      this->s = vec3_buffer_cache->LookupOrAdd(s);
    }

    if (!face_indices.empty()) {
      this->face_indices = int_buffer_cache->LookupOrAdd(face_indices);
    }
  }

} // namespace pbr
