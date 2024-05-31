/**
 * \file util/ref.hpp
 **/
#ifndef PBR_REF_HPP
#define PBR_REF_HPP

#include <atomic>

namespace pbr {
namespace detail {

  void RegisterReference(void* instance);
  void RemoveReference(void* instance);
  bool IsValidRef(void* instance);

} // namespace detail

  class RefCounted {
    public:
      RefCounted() 
        : count(0) {}
      virtual ~RefCounted() = default;

      void Increment();
      void Decrement();

      uint64_t Count() const { return count; }
    
    private:
      mutable std::atomic<uint64_t> count;
  };

  template <typename T>
  class Ref {
    public:
      Ref()
          : object(nullptr) {}

      Ref(std::nullptr_t)
          : object(nullptr) {}

      Ref(T* p) {
        object = p;
        IncRef();
      }

      Ref(Ref<T>& other) {
        object = other.object;
        IncRef();
      }

      Ref(const Ref<T>& other) {
        object = other.object;
        IncRef();
      }

      Ref(Ref<T>&& other) noexcept {
        object = other.object;
        other.object = nullptr;
      }

      Ref& operator=(std::nullptr_t) {
        DecRef();
        object = nullptr;
        return *this;
      }
      
      Ref& operator=(const Ref<T>& other) {
        if (this == &other) {
          return *this;
        }

        other.IncRef();
        DecRef();

        object = other.object;
        return *this;
      }

      Ref& operator=(Ref<T>&& other) noexcept {
        if (this != &other) {
          object = other.object;
          other.object = nullptr;
        }
        return *this;
      }

      template<typename T2>
      Ref(const Ref<T2>& other) {
      	object = (T*)other.object;
      	IncRef();
      }
      
      template<typename T2>
      Ref(Ref<T2>&& other) {
      	object = (T*)other.object;
      	other.object = nullptr;
      }

      template <typename... Args>
      Ref(Args&&... args) {
        static_assert(std::is_constructible_v<T , Args...> , "Cannot construct a reference from given arguments");
        static_assert(std::is_base_of_v<RefCounted , T> , "Cannot construct a reference from a non-RefCounted type");
        object = new T(std::forward<Args>(args)...);
        IncRef();
      }
      
      virtual ~Ref() {
        DecRef();
      }

      template<typename T2>
      Ref& operator=(const Ref<T2>& other) {
      	other.IncRef();
      	object = other.object;
      	return *this;
      }
      
      template<typename T2>
      Ref& operator=(Ref<T2>&& other) {
      	object = other.object;
      	other.object = nullptr;
      	return *this;
      }

      operator bool() { return object != nullptr; }
      operator bool() const { return object != nullptr; }
      
      T* operator->() { return object; }
      const T* operator->() const { return object; }

      T* Raw() { return object; }
      const T* Raw() const { return object; }

      void Reset(T* object = nullptr) {
        DecRef();
        this->object = object;
      }

      template <typename U>
      Ref<U> As() const {
        return Ref<U>(*this);
      }

      static Ref<T> Clone(const Ref<T>& old_ref) {
        return Ref<T>(old_ref);
      }

      template <typename... Args>
      static Ref<T> Create(Args&&... args) {
        static_assert(std::is_constructible_v<T , Args...> , "Cannot construct a reference from given arguments");
        static_assert(std::is_base_of_v<RefCounted , T> , "Cannot create a reference to a non-refcounted object");
        return Ref<T>(new T(std::forward<Args>(args)...));
      }

      bool operator==(const Ref<T>& other) const {
        return object == other.object;
      }

      bool operator==(std::nullptr_t) const {
        return object == nullptr;
      }

      bool EqualsObj(const Ref<T>& other) const {
        return object == other.object;
      }

    private:
      mutable T* object;

      void IncRef() const {
        if (object != nullptr) {
          object->Increment();
          detail::RegisterReference(object); 
        }
      }

      void DecRef() const {
        if (object != nullptr) {
          object->Decrement();

          if (object->Count() == 0) {
            detail::RemoveReference(object);
            delete object;
            object = nullptr;
          }
        }
      }

      template <typename U>
      friend class Ref;
  };

  template <typename T , typename... Args>
  Ref<T> NewRef(Args&&... args) {
    static_assert(std::is_constructible_v<T , Args...> , "Cannot construct a reference from given arguments");
    static_assert(std::is_base_of_v<RefCounted , T> , "Cannot create a reference to a non-refcounted object");
    return Ref<T>(std::forward<Args>(args)...);
  }

} // namespace pbr

#endif // !PBR_REF_HPP
