/**
 * \file geometry/shape.hpp
 **/
#ifndef PBR_SHAPE_HPP
#define PBR_SHAPE_HPP

#include "math/bounds.hpp"
#include "math/direction_cone.hpp"

namespace pbr {

  class Shape {
    public:
        virtual Bounds3 Bounds() const = 0;

        virtual DirectionCone NormalBounds() const = 0;

    private:
  }; 

} // namespace pbr

#endif // !PBR_SHAPE_HPP
