/**
 * \file math/transform.cpp
 **/
#include "math/transform.hpp"

#include <limits>
#include <optional>

#include <glm/gtc/matrix_transform.hpp>

namespace pbr {

  Transform::Transform(const glm::mat4& m)
      : model(m) {
    inverse_model = glm::inverse(m);
  }

} // namespace pbr
