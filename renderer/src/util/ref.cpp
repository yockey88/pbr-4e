/**
 * \file util/ref.cpp
 **/
#include "util/ref.hpp"

#include <unordered_set>

namespace pbr {
namespace {

  std::unordered_set<void*> refs;

} // namespace anon
namespace detail {

  void RegisterReference(void* instance) {
    refs.insert(instance);
  }

  void RemoveReference(void* instance) {
    refs.erase(instance);
  }

  bool IsValidRef(void* instance) {
    return refs.find(instance) != refs.end();
  }

} // namespace detail

  void RefCounted::Increment() {
    ++count;
  }

  void RefCounted::Decrement() {
    --count;
  }

} // namespace pbr
