/**
 * \file util/defines.hpp
 **/
#ifndef PBR_DEFINES_HPP
#define PBR_DEFINES_HPP

#include <string>
#include <vector>
#include <optional>
#include <memory_resource>

namespace pbr {

  template <typename T>
  using Opt = std::optional<T>;

  using Alloc = std::pmr::polymorphic_allocator<std::byte>;

  enum class RenderingCoordinateSystem {
    CAMERA ,
    CAMERA_WORLD ,
    WORLD ,
  };
  
  struct Options {
    int32_t seed = 0;
    bool quiet = false;
    bool disable_pixel_jitter = false;
    bool disable_wavelength_jitter = false;
    bool disable_texture_filtering = false;
    bool force_diffuse = false;
    bool use_gpu = false;
    bool wavefront = false;
  
    RenderingCoordinateSystem rendering_space 
      = RenderingCoordinateSystem::CAMERA_WORLD;
  
  };
  
  std::vector<std::string> GetCommandLineArgs(int argc , char* argv[]);
  
  void InitPBR(const Options& opts);
  void CleanupPBR();

} // namespace pbr

#endif // !PBR_DEFINES_HPP
