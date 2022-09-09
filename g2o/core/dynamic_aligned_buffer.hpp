#pragma once

#include <Eigen/Core>
#include <cstddef>

namespace g2o {
// 16 byte aligned allocation functions
template <typename Type>
Type* allocate_aligned(size_t n) {
  return static_cast<Type*>(Eigen::internal::aligned_malloc(n * sizeof(Type)));
}

template <typename Type>
Type* reallocate_aligned(Type* ptr, size_t newSize, size_t oldSize) {
  return static_cast<Type*>(Eigen::internal::aligned_realloc(
      ptr, newSize * sizeof(Type), oldSize * sizeof(Type)));
}

template <typename Type>
void free_aligned(Type* block) {
  Eigen::internal::aligned_free(block);
}

template <typename Type>
struct DynamicAlignedBuffer {
  explicit DynamicAlignedBuffer(size_t size) { allocate(size); }

  DynamicAlignedBuffer(DynamicAlignedBuffer const&) = delete;
  DynamicAlignedBuffer  & operator=(DynamicAlignedBuffer const&) = delete;

  ~DynamicAlignedBuffer() { free(); }

  Type* request(size_t n) {
    if (n <= size_) return ptr_;

    ptr_ = reallocate_aligned<Type>(ptr_, n, size_);
    size_ = ptr_ ? n : 0;

    return ptr_;
  }

 private:
  void allocate(size_t size) {
    ptr_ = allocate_aligned<Type>(size);
    if (ptr_ != nullptr) size_ = size;
  }

  void free() {
    if (ptr_ != nullptr) {
      free_aligned<Type>(ptr_);
      size_ = 0;
      ptr_ = nullptr;
    }
  }

  std::size_t size_ = 0;
  Type* ptr_ = nullptr;
};

template <typename T>
struct AlignedDeleter {
  void operator()(T* block) { free_aligned(block); }
};

}  // namespace g2o
