/**
 * \file camera/image.hpp
 **/
#ifndef PBR_IMAGE_HPP
#define PBR_IMAGE_HPP

#include <string>
#include <vector>
#include <map>

#include "util/defines.hpp"
#include "util/ref.hpp"
#include "math/bounds.hpp"
#include "radiometry/color.hpp"

namespace pbr {

  struct ImageMetadata : public RefCounted {
    const RGBColorSpace* GetColorSpace() const;

    Opt<float> render_time_sec;
    Opt<glm::mat4> cam_from_world , ndc_from_world;
    Opt<Bounds2> pixel_bounds;
    Opt<Point2> full_resolution;
    Opt<int32_t> samples_per_pixel;
    Opt<float> mse;
    std::map<uint64_t , std::string> strings;
    std::map<uint64_t , std::vector<std::string>> string_vectors;

    private:
      Opt<const RGBColorSpace*> color_space;
  };

  class Image : public RefCounted {
    public:
    private:
  };

} // namespace pbr

#endif // !PBR_IMAGE_HPP
