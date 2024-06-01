/**
 * \file util/buffer_cache.hpp
 **/
#ifndef PBR_BUFFER_CACHE_HPP
#define PBR_BUFFER_CACHE_HPP

#include <cstring>
#include <shared_mutex>
#include <span>
#include <unordered_set>

#include "math/math.hpp"

namespace pbr {

  inline uint64_t MurmurHash(const uint8_t* key , size_t len , uint64_t seed) {
    const uint64_t m = 0xc6a4a7935bd1e995ull;
    const int32_t r = 47;

    uint64_t h = seed ^ (len * m);
    const uint8_t* end = key + 8 * (len / 8);

    while (key != end) {
      uint64_t k;
      std::memcpy(&k , key , sizeof(uint64_t));
      key += 8;

      k *= m;
      k ^= k >> r;
      k *= m;

      h ^= k;
      h *= m;
    }

    switch (len & 7) {
      case 7: h ^= uint64_t(key[6]) << 48;
      case 6: h ^= uint64_t(key[5]) << 40;
      case 5: h ^= uint64_t(key[4]) << 32;
      case 4: h ^= uint64_t(key[3]) << 24;
      case 3: h ^= uint64_t(key[2]) << 16;
      case 2: h ^= uint64_t(key[1]) << 8;
      case 1: 
        h ^= uint64_t(key[0]);
        h *= m;
      default: break;
    }

    h ^= m;
    h *= m;
    h ^= h >> r;

    return h;
  }

  template <typename T>
  inline uint64_t HashBuffer(const T* ptr , size_t size , uint64_t seed = 0) {
    return MurmurHash(static_cast<const uint8_t*>(ptr) , size , seed);
  }

  template <typename T>
  class BufferCache {
    public:
      ~BufferCache() {
        for (int32_t i = 0; i < n_shards; ++i) {
          for (auto& obj : cache[i]) {
            delete[] obj.ptr;
          }
        }
      }

      const T* LookupOrAdd(const std::span<const T>& buf) {
        Buffer lookup(buf.data() , buf.size());
        int32_t shard_idx = uint32_t(lookup.hash) >> (32 - log_shards);

        mutexes[shard_idx].lock_shared();
        if (auto iter = cache[shard_idx].find(lookup); iter != cache[shard_idx].end()) {
          const T* ptr = iter->ptr;
          mutexes[shard_idx].unlock_shared();
          return ptr;
        }

        mutexes[shard_idx].unlock_shared();
        T* ptr = new T[buf.size()];
        std::copy(buf.begin() , buf.end() , ptr);
        bytes_used += buf.size() * sizeof(T);
        mutexes[shard_idx].lock();

        /// this is the case where another thread adds the buffer first
        if (auto iter = cache[shard_idx].find(lookup); iter != cache[shard_idx].end()) {
          const T* cache_ptr = iter->ptr;
          mutexes[shard_idx].unlock();

          delete[] ptr;

          return cache_ptr;
        }

        cache[shard_idx].insert(Buffer(ptr , buf.size()));
        mutexes[shard_idx].unlock();
        return ptr;
      }

      size_t BytesUsed() const { 
        return bytes_used;
      }

    private:
      struct Buffer {
        Buffer() = default;
        Buffer(const T* ptr , size_t size) : ptr(ptr) , size(size) {
          hash = HashBuffer(ptr , size);
        }

        bool operator==(const Buffer& b) const {
          return size == b.size && hash == b.hash && 
                 std::memcmp(ptr , b.ptr , size * sizeof(T)) == 0;
        }

        const T* ptr = nullptr;
        size_t size = 0 , hash;
      };

      struct  BufferHasher {
        size_t operator()(const Buffer& b) const {
          return b.hash;
        }
      };

      constexpr static int32_t log_shards = 6;
      constexpr static int32_t n_shards = 1 << log_shards;
      std::shared_mutex mutexes[n_shards];
      std::unordered_set<Buffer , BufferHasher> cache[n_shards];
      std::atomic<size_t> bytes_used{};
  };

  extern BufferCache<int32_t>* int_buffer_cache;
  extern BufferCache<Point2>* point2_buffer_cache;
  extern BufferCache<Point3>* point3_buffer_cache;
  extern BufferCache<glm::vec3>* vec3_buffer_cache;
  extern BufferCache<glm::vec3>* normal3_buffer_cache;

  void InitBufferCaches();
  void CleanupBufferCaches();

} // namespace pbr

#endif // !PBR_BUFFER_CACHE_HPP
