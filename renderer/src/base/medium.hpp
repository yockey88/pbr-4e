/**
 * \file base/medium.hpp
 **/
#ifndef PBR_MEDIUM_HPP
#define PBR_MEDIUM_HPP

#include "util/ref.hpp"

namespace pbr {

  class Medium : public RefCounted {
    public:
      Medium() {}

      virtual ~Medium() override {}

    private:
  };

} // namespace pbr

#endif // !PBR_MEDIUM_HPP
