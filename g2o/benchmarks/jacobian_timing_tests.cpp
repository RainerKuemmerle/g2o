#include <benchmark/benchmark.h>

#include "g2o/core/dynamic_aligned_buffer.hpp"

// Test several different ways of evaluating Jacobians to see the impact of different ways of implementing stuff.


template<typename number_t, int D>
void BM_FixedArray(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      number_t add_vi[D] = 
        {
        }
        ;
      for (int i = 0; i < D; ++i)
        {
          benchmark::DoNotOptimize(add_vi[i]=1);
          benchmark::DoNotOptimize(add_vi[i]=-1);
          benchmark::DoNotOptimize(add_vi[i]=0);
        }
    }
}


template<typename number_t, int D>
void BM_FixedArrayPointer(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      number_t add_vi[D] = 
        {
        }
        ;
      number_t* v  = &add_vi[0];
      for (int i = 0; i < D; ++i)
        {
          benchmark::DoNotOptimize((*v)=1);
          benchmark::DoNotOptimize((*v)=-1);
          benchmark::DoNotOptimize((*v++)=0);
        }
    }
}



template<typename number_t>
void BM_VariableArray(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      const int d = state.range(0);
      
      number_t add_vi[d];
      //std::fill(add_vi, add_vi + d, number_t(0.0));

      for (int i = 0; i < d; ++i)
        {
          benchmark::DoNotOptimize(add_vi[i]=0);
        }

      for (int i = 0; i < d; ++i)
        {
          benchmark::DoNotOptimize(add_vi[i]=1);
          benchmark::DoNotOptimize(add_vi[i]=-1);
          benchmark::DoNotOptimize(add_vi[i]=0);
        }
    }
}


template<typename number_t>
void BM_VariableArrayPointer(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      const int d = state.range(0);
      number_t add_vi[d];
      
      std::fill(add_vi, add_vi + d, number_t(0.0));

      number_t* v  = &add_vi[0];

      for (int i = 0; i < d; ++i)
        {
          benchmark::DoNotOptimize((*v)=1);
          benchmark::DoNotOptimize((*v)=-1);
          benchmark::DoNotOptimize((*v++)=0);
        }
    }
}

template<typename number_t, int D>
void BM_StaticEigenMatrix(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      Eigen::Matrix<number_t, D, 1> add_vi = Eigen::Matrix<number_t, D, 1>::Zero();
      for (int i = 0; i < D; ++i)
        {
          benchmark::DoNotOptimize(add_vi[i]=1);
          benchmark::DoNotOptimize(add_vi[i]=-1);
          benchmark::DoNotOptimize(add_vi[i]=0);
        }
    }
}

template<typename number_t, int D>
void BM_StaticEigenMatrixPointer(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      Eigen::Matrix<number_t, D, 1> add_vi = Eigen::Matrix<number_t, D, 1>::Zero();
      number_t* v  = add_vi.data();
      for (int i = 0; i < D; ++i)
        {
          benchmark::DoNotOptimize((*v)=1);
          benchmark::DoNotOptimize((*v)=-1);
          benchmark::DoNotOptimize((*v++)=0);
        }
    }
}


template<typename number_t>
void BM_DynamicEigenMatrix(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      const int d = state.range(0);
      Eigen::Matrix<number_t, Eigen::Dynamic, 1> add_vi(d);
      add_vi.setZero();
      for (int i = 0; i < d; ++i)
         {
          benchmark::DoNotOptimize(add_vi[i]=1);
          benchmark::DoNotOptimize(add_vi[i]=-1);
          benchmark::DoNotOptimize(add_vi[i]=0);
        }
    }
}

template<typename number_t>
void BM_DynamicEigenMatrixPointer(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      const int d = state.range(0);
      Eigen::Matrix<number_t, Eigen::Dynamic, 1> add_vi(d);
      add_vi.setZero();
      number_t* v  = add_vi.data();
      for (int i = 0; i < d; ++i)
        {
          benchmark::DoNotOptimize((*v)=1);
          benchmark::DoNotOptimize((*v)=-1);
          benchmark::DoNotOptimize((*v++)=0);
        }
    }
}

template<typename number_t>
void BM_DynamicAlignedBuffer(benchmark::State& state)
{
  
  while (state.KeepRunning())
    {
      const int d = state.range(0);
      static g2o::dynamic_aligned_buffer<number_t> buffer{ size_t(d)};
      number_t* add_vi = buffer.request(d);
      std::fill(add_vi, add_vi + d, number_t(0.0));
      for (int i = 0; i < d; ++i)
        {
          benchmark::DoNotOptimize(add_vi[i]=1);
          benchmark::DoNotOptimize(add_vi[i]=-1);
          benchmark::DoNotOptimize(add_vi[i]=0);
        }
    }
}


template<typename number_t>
void BM_DynamicAlignedBufferPointer(benchmark::State& state)
{
  
  while (state.KeepRunning())
    {
      const int d = state.range(0);
      static g2o::dynamic_aligned_buffer<number_t> buffer{ size_t(d) };
      number_t* add_vi = buffer.request(d);
      std::fill(add_vi, add_vi + d, number_t(0.0));
      number_t* v= add_vi;
      
      for (int i = 0; i < d; ++i)
        {
          benchmark::DoNotOptimize((*v)=1);
          benchmark::DoNotOptimize((*v)=-1);
          benchmark::DoNotOptimize((*v++)=0);
        }
    }
}

template<typename number_t>
void BM_StaticDynamicDynamicAlignedBufferHybrid(benchmark::State& state)
{
  while (state.KeepRunning())
    {
      const int d = state.range(0);
      if (d <= 10)
        {
          number_t add_vi[10] = 
            {
            }
            ;
          for (int i = 0; i < d; ++i)
            {
              benchmark::DoNotOptimize(add_vi[i]=1);
              benchmark::DoNotOptimize(add_vi[i]=-1);
              benchmark::DoNotOptimize(add_vi[i]=0);
            }
        }
      else
        {
          static g2o::dynamic_aligned_buffer<number_t> buffer{ size_t(d) };
          number_t* add_vi = buffer.request(d);
          std::fill(add_vi, add_vi + d, number_t(0.0));
          for (int i = 0; i < d; ++i)
            {
              benchmark::DoNotOptimize(add_vi[i]=1);
              benchmark::DoNotOptimize(add_vi[i]=-1);
              benchmark::DoNotOptimize(add_vi[i]=0);
            }
        }
    }
}
#define DECLARE_TESTS(N, D)                                     \
  BENCHMARK_TEMPLATE2(BM_FixedArray, N, D);                     \
  BENCHMARK_TEMPLATE2(BM_FixedArrayPointer, N, D);              \
  BENCHMARK_TEMPLATE(BM_VariableArray, N)->Arg(D);                     \
  BENCHMARK_TEMPLATE(BM_VariableArrayPointer, N)->Arg(D);         \
  BENCHMARK_TEMPLATE2(BM_StaticEigenMatrix, N, D);              \
  BENCHMARK_TEMPLATE2(BM_StaticEigenMatrixPointer, N, D);       \
  BENCHMARK_TEMPLATE(BM_DynamicEigenMatrix, N)->Arg(D);         \
  BENCHMARK_TEMPLATE(BM_DynamicEigenMatrixPointer, N)->Arg(D);  \
  BENCHMARK_TEMPLATE2(BM_DynamicAlignedBuffer, N, D);           \
  BENCHMARK_TEMPLATE2(BM_DynamicAlignedPointerBuffer, N, D);    \
  BENCHMARK_TEMPLATE(BM_StaticDynamicPointerHybrid, N)->Arg(D);
/*

DECLARE_TESTS(double, 1)
DECLARE_TESTS(double, 2)
DECLARE_TESTS(double, 3)
DECLARE_TESTS(double, 4)
DECLARE_TESTS(double, 5)
DECLARE_TESTS(double, 6)
DECLARE_TESTS(double, 7)
DECLARE_TESTS(double, 8)
DECLARE_TESTS(double, 9)
DECLARE_TESTS(double, 10)
DECLARE_TESTS(double, 200)

*/

#define BENCHMARK_FIXED_DIMENSION(M, F) \
BENCHMARK_TEMPLATE2(M, F, 1);   \
BENCHMARK_TEMPLATE2(M, F, 2);   \
BENCHMARK_TEMPLATE2(M, F, 3);   \
BENCHMARK_TEMPLATE2(M, F, 4);   \
BENCHMARK_TEMPLATE2(M, F, 5);   \
BENCHMARK_TEMPLATE2(M, F, 6);   \
BENCHMARK_TEMPLATE2(M, F, 7);   \
BENCHMARK_TEMPLATE2(M, F, 8);   \
BENCHMARK_TEMPLATE2(M, F, 9);   \
 BENCHMARK_TEMPLATE2(M, F, 10);\
BENCHMARK_TEMPLATE2(M, F, 20);\
BENCHMARK_TEMPLATE2(M, F, 30);\
BENCHMARK_TEMPLATE2(M, F, 40);\
BENCHMARK_TEMPLATE2(M, F, 50);\
BENCHMARK_TEMPLATE2(M, F, 100);

#define BENCHMARK_VARIABLE_DIMENSION(M, F) \
  BENCHMARK_TEMPLATE(M, F)->Arg(1);       \
BENCHMARK_TEMPLATE(M, F)->Arg(2);   \
BENCHMARK_TEMPLATE(M, F)->Arg(3);   \
BENCHMARK_TEMPLATE(M, F)->Arg(4);   \
BENCHMARK_TEMPLATE(M, F)->Arg(5);   \
BENCHMARK_TEMPLATE(M, F)->Arg(6);   \
BENCHMARK_TEMPLATE(M, F)->Arg(7);   \
BENCHMARK_TEMPLATE(M, F)->Arg(8);   \
BENCHMARK_TEMPLATE(M, F)->Arg(9);   \
BENCHMARK_TEMPLATE(M, F)->Arg(10);\
BENCHMARK_TEMPLATE(M, F)->Arg(20);\
BENCHMARK_TEMPLATE(M, F)->Arg(30);\
BENCHMARK_TEMPLATE(M, F)->Arg(40);\
BENCHMARK_TEMPLATE(M, F)->Arg(50);\
BENCHMARK_TEMPLATE(M, F)->Arg(100);

BENCHMARK_FIXED_DIMENSION(BM_FixedArray, double)
BENCHMARK_FIXED_DIMENSION(BM_FixedArrayPointer, double)

BENCHMARK_FIXED_DIMENSION(BM_StaticEigenMatrix, double)
BENCHMARK_FIXED_DIMENSION(BM_StaticEigenMatrixPointer, double)

BENCHMARK_VARIABLE_DIMENSION(BM_VariableArray, double)
BENCHMARK_VARIABLE_DIMENSION(BM_VariableArrayPointer, double)

BENCHMARK_VARIABLE_DIMENSION(BM_DynamicEigenMatrix, double)
BENCHMARK_VARIABLE_DIMENSION(BM_DynamicEigenMatrixPointer, double)

BENCHMARK_VARIABLE_DIMENSION(BM_DynamicAlignedBuffer, double)
BENCHMARK_VARIABLE_DIMENSION(BM_DynamicAlignedBufferPointer, double)

BENCHMARK_VARIABLE_DIMENSION(BM_StaticDynamicDynamicAlignedBufferHybrid, double)

/*

BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(1);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(2);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(3);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(4);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(5);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(6);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(7);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(8);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(9);   
BENCHMARK_TEMPLATE(BM_VariableArray, double)->Arg(10);   

BENCHMARK_TEMPLATE2(BM_FixedArray, double, 1);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 2);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 3);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 4);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 5);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 6);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 7);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 8);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 9);   
BENCHMARK_TEMPLATE2(BM_FixedArray, double, 10); 
*/

BENCHMARK_MAIN();
