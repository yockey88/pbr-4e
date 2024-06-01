/**
 * \file main.cpp
 **/
#include "pbr.hpp"

#include <iostream>
#include <util/buffer_cache.hpp>

int renderer(int argc , char* argv[]) {
  pbr::InitBufferCaches();

  std::vector<int32_t> v {
    1 , 2 , 3 , 4 , 5 , 6 , 7
  };

  const int32_t* v_buff = pbr::int_buffer_cache->LookupOrAdd(v);

  if (pbr::int_buffer_cache->BytesUsed() != 7 * sizeof(int32_t)) {
    pbr::CleanupBufferCaches();
    std::cout << "something not right\n";
    return 1;
  }

  if (v_buff == nullptr) {
    pbr::CleanupBufferCaches();
    std::cout << "v_buff == nullptr\n";
    return 1;
  }

  for (auto i = 0; i < 7; ++i) {
    std::cout << "v_buff[" << i << "] = " << v_buff[i] << "\n";
  }

  pbr::CleanupBufferCaches();

  std::cout << "PBR shutdown successful\n" << std::endl;
  return 0;
}

int WINAPI WinMain(HINSTANCE inst , HINSTANCE prev_inst , LPSTR cmdline , int cmdshow) {
  return renderer(__argc , __argv);
}
