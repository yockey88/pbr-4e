/**
 * \file pbr.hpp
 **/
#ifndef PBR_HPP
#define PBR_HPP

#include "util/defines.hpp"
#include "util/float.hpp"
#include "util/ref.hpp"
#include "util/buffer_cache.hpp"
#include "math/math.hpp"
#include "math/sampling_functions.hpp"
#include "math/vecmath.hpp"
#include "math/octahedral_vector.hpp"
#include "math/bounds.hpp"
#include "math/direction_cone.hpp"
#include "math/frame.hpp"
#include "math/transform.hpp"
#include "math/ray.hpp"
#include "geometry/interaction.hpp"
#include "geometry/shape.hpp"
#include "geometry/sphere.hpp"
#include "geometry/cylinder.hpp"
#include "geometry/disk.hpp"
#include "radiometry/spectrum.hpp"
#include "radiometry/color.hpp"
#include "camera/camera_transform.hpp"
#include "camera/image.hpp"
#include "camera/film.hpp"
#include "camera/camera.hpp"

/// put this here because windows is like a fucking bomb when it comes to includes
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#endif // !PBR_HPP
