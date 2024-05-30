/**
 * \file camera/film.hpp
 **/
#ifndef PBR_FILM_HPP
#define PBR_FILM_HPP

#include "base/ref.hpp"
#include "math/vecmath.hpp"

namespace pbr {

  class Film : public RefCounted {
    public:
      Point2 FullResolution() { return Point2(0 , 0); }
  };

  class RGBFilm : public Film {};

  class GBufferFilm : public Film {};

  class SpectralFilm : public Film {};

} // namespace pbr

#endif // !PBR_FILM_HPP
