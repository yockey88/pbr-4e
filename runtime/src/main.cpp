/**
 * \file main.cpp
 **/
#include "pbr.hpp"

int renderer(int argc , char* argv[]) {
  pbr::InitBufferCaches();

  pbr::CleanupBufferCaches();
  return 0;
}

int WINAPI WinMain(HINSTANCE inst , HINSTANCE prev_inst , LPSTR cmdline , int cmdshow) {
  return renderer(__argc , __argv);
}
