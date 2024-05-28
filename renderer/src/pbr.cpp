/**
 * \file pbr.cpp
 **/
#include "pbr.hpp"

namespace pbr {

std::vector<std::string> GetCommandLineArgs(int argc , char* argv[]) {
  std::vector<std::string> args{};
  for (auto i = 0; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }

  return args;
}

void InitPBR(const Options& opts) {
}

void CleanupPBR() {
}

} // namespace pbr
