/**
 * \file main.cpp
 **/
#include "pbr.hpp"

#include <Windows.h>

int pbr_renderer(int argc , char* argv[]) {
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

  return 0;
}

int WINAPI WinMain(HINSTANCE inst , HINSTANCE prev_inst , LPSTR cmdline , int cmdshow) {
  return pbr_renderer(__argc , __argv);
}
