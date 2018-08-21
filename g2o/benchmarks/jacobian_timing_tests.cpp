#include <benchmark/benchmark.h>
#include "/usr/local/include/eigen3/Eigen/Core"

// Test several different ways of evaluating Jacobians to see the impact of different ways of implementing stuff.

    // 16 byte aligned allocation functions
    template<typename Type>
    Type* allocate_aligned(size_t n)
    {
        return (Type*)Eigen::internal::aligned_malloc(n * sizeof(Type));
    }

    template<typename Type>
    Type* reallocate_aligned(Type* ptr, size_t newSize, size_t oldSize)
    {
        return (Type*)Eigen::internal::aligned_realloc(ptr, newSize * sizeof(Type), oldSize * sizeof(Type));
    }

    template<typename Type>
    void free_aligned(Type* block)
    {
        Eigen::internal::aligned_free(block);
    }

    template<typename Type>
    struct dynamic_aligned_buffer
    {
        dynamic_aligned_buffer(size_t size)
            : m_size{ 0 }, m_ptr{ nullptr }
        {
            allocate(size);
        }

        ~dynamic_aligned_buffer()
        {
            free();
        }

        Type* request(size_t n)
        {
            if (n <= m_size)
                return m_ptr;

            m_ptr = reallocate_aligned<Type>(m_ptr, n, m_size);
            m_size = m_ptr ? n : 0;

            return m_ptr;
        }

    private:
        void allocate(size_t size)
        {
            m_ptr = allocate_aligned<Type>(size);
            if (m_ptr != nullptr)
                m_size = size;
        }

        void free()
        {
            if (m_ptr != nullptr)
            {
                free_aligned<Type>(m_ptr);
                m_size = 0;
                m_ptr = nullptr;
            }
        }

        std::size_t m_size;
        Type* m_ptr;
    };

    template<typename T>
    struct aligned_deleter
    {
        void operator()(T* block)
        {
            free_aligned(block);
        }
    };


template<typename number_t, int D>
void BM_FixedArray(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      number_t add_vi[D] = 
        {
        }
        ;
      number_t* v  = &add_vi[0];
      for (int i = 0; i < D; ++i)
        add_vi[i]++;
    }
}


template<typename number_t, int D>
void BM_FixedPointerArray(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      number_t add_vi[D] = 
        {
        }
        ;
      number_t* v  = &add_vi[0];
      for (int i = 0; i < D; ++i)
        (*v++)++;
    }
}

template<typename number_t, int D>
void BM_StaticEigenMatrix(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      Eigen::Matrix<number_t, D, 1> add_vi = Eigen::Matrix<number_t, D, 1>::Zero();
      for (int i = 0; i < D; ++i)
        add_vi[i]++;
    }
}

template<typename number_t, int D>
void BM_StaticEigenPointerMatrix(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      Eigen::Matrix<number_t, D, 1> add_vi = Eigen::Matrix<number_t, D, 1>::Zero();
      number_t* v  = add_vi.data();
      for (int i = 0; i < D; ++i)
        (*v++)++;
    }
}


template<typename number_t>
void BM_DynamicEigenMatrix(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      Eigen::Matrix<number_t, Eigen::Dynamic, 1> add_vi(state.range(0));
      add_vi.setZero();
       for (int i = 0; i < state.range(0); ++i)
        add_vi[i]++;
    }

}

template<typename number_t>
void BM_DynamicEigenPointerMatrix(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      Eigen::Matrix<number_t, Eigen::Dynamic, 1> add_vi(state.range(0));
      add_vi.setZero();
      number_t* v  = add_vi.data();
      for (int i = 0; i < state.range(0); ++i)
        (*v++)++;
    }
}

template<typename number_t, int D>
void BM_DynamicAlignedBuffer(benchmark::State& state)
{
  
  while (state.KeepRunning())
    {
      dynamic_aligned_buffer<number_t> buffer{ 12 };
      number_t* add_vi = buffer.request(D);
      for (int i = 0; i < D; ++i)
        add_vi[i]++;
    }
}


template<typename number_t, int D>
void BM_DynamicAlignedPointerBuffer(benchmark::State& state)
{
  
  while (state.KeepRunning())
    {
      dynamic_aligned_buffer<number_t> buffer{ 12 };
      number_t* add_vi = buffer.request(D);
      number_t* v= add_vi;
      
       v= add_vi;
      for (int i = 0; i < D; ++i)
        (*v++)++;
    }
}

template<typename number_t>
void BM_StaticDynamicPointerHybrid(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      if (state.range(0) < 20)
        {
          number_t add_vi[20] = 
            {
            }
            ;
          number_t* v  = &add_vi[0];
          for (int i = 0; i < state.range(0); ++i)
            add_vi[i]++;
        }
      else
        {
          Eigen::Matrix<number_t, Eigen::Dynamic, 1> add_vi(state.range(0));
          add_vi.setZero();
          number_t* v  = add_vi.data();
          for (int i = 0; i < state.range(0); ++i)
            (*v++)++;
        }
    }
}

#define DECLARE_TESTS(N, D) \
BENCHMARK_TEMPLATE2(BM_FixedArray, N, D);\
BENCHMARK_TEMPLATE2(BM_FixedPointerArray, N, D);\
BENCHMARK_TEMPLATE2(BM_StaticEigenMatrix, N, D);\
BENCHMARK_TEMPLATE2(BM_StaticEigenPointerMatrix, N, D);\
 BENCHMARK_TEMPLATE(BM_DynamicEigenMatrix, N)->Arg(D);      \
 BENCHMARK_TEMPLATE(BM_DynamicEigenPointerMatrix, N)->Arg(D);  \
BENCHMARK_TEMPLATE2(BM_DynamicAlignedBuffer, N, D);\
BENCHMARK_TEMPLATE2(BM_DynamicAlignedPointerBuffer, N, D); \
 BENCHMARK_TEMPLATE(BM_StaticDynamicPointerHybrid, N)->Arg(D);  \
 

DECLARE_TESTS(double, 15)
DECLARE_TESTS(double, 200)

DECLARE_TESTS(float, 3)
DECLARE_TESTS(float, 200)

BENCHMARK_MAIN();
