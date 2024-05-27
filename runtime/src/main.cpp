/**
 * \file main.cpp
 **/
#include "renderer.hpp"

#include <Windows.h>

int pbr_renderer(int argc , char* argv[]) {
  Hello();
  return 0;
}

int WINAPI WinMain(HINSTANCE inst , HINSTANCE prev_inst , LPSTR cmdline , int cmdshow) {
  return pbr_renderer(__argc , __argv);
}
