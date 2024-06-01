/**
 * \file util/buffer_cache.cpp
 **/
#include "util/buffer_cache.hpp"

namespace pbr {
  
  BufferCache<int32_t>* int_buffer_cache;
  BufferCache<Point2>* point2_buffer_cache;
  BufferCache<Point3>* point3_buffer_cache;
  BufferCache<glm::vec3>* vec3_buffer_cache;
  BufferCache<glm::vec3>* normal3_buffer_cache;
  
  void InitBufferCaches() {
    int_buffer_cache = new BufferCache<int32_t>;
    point2_buffer_cache = new BufferCache<Point2>;
    point3_buffer_cache = new BufferCache<Point3>;
    vec3_buffer_cache = new BufferCache<glm::vec3>;
    normal3_buffer_cache = new BufferCache<glm::vec3>;
  }

  void CleanupBufferCaches() {
    delete int_buffer_cache;
    delete point2_buffer_cache;
    delete point3_buffer_cache;
    delete vec3_buffer_cache;
    delete normal3_buffer_cache;
  }

} // namespace pbr
