/**
 * \file main.cpp
 **/
#include "pbr.hpp"

#include <iostream>

#include <Windows.h>

#include "base/ref.hpp"
#include "radiometry/spectrum.hpp"

int pbr_renderer(int argc , char* argv[]) {
#if 1
#else 
  auto args = pbr::GetCommandLineArgs(argc , argv);
  
  pbr::Options opts;
  std::vector<std::string> filenames;

  // parse command line arguments
  //

  // initialize renderer
  pbr::InitPBR(opts);

  // parse provided scene description files
  // render the scene
  // clean up after rendering
  pbr::CleanupPBR();
#endif
  return 0;
}

int WINAPI WinMain(HINSTANCE inst , HINSTANCE prev_inst , LPSTR cmdline , int cmdshow) {
  return pbr_renderer(__argc , __argv);
}
