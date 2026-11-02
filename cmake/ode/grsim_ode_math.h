#ifndef GRSIM_ODE_MATH_H
#define GRSIM_ODE_MATH_H

#include <cstddef>

#if defined(GRSIM_ODE_USE_BLAS) && GRSIM_ODE_USE_BLAS
# ifdef __APPLE__
#  include <Accelerate/Accelerate.h>
# else
#  include <cblas.h>
# endif
#endif


#if defined(GRSIM_ODE_USE_BLAS) && GRSIM_ODE_USE_BLAS && defined(__x86_64__) && \
    (defined(__GNUC__) || defined(__clang__)) && !defined(__APPLE__)
#include <immintrin.h>
#define GRSIM_ODE_HAS_AVX2 1
namespace grsim_ode {
// Four-row solves read ODE's existing row-major factors directly. Runtime
// dispatch keeps baseline x86 CPUs and the platform BLAS fallback usable.
__attribute__((target("avx2"))) inline double horizontalSum(__m256d value)
{
    const __m128d pair = _mm_add_pd(_mm256_castpd256_pd128(value),
                                  _mm256_extractf128_pd(value, 1));
    return _mm_cvtsd_f64(_mm_add_sd(pair, _mm_unpackhi_pd(pair, pair)));
}
__attribute__((target("avx2"))) inline void solveLowerAvx2(
    const double* matrix, double* rhs, unsigned count, unsigned rowSkip)
{
    unsigned row = 0;
    for (; row + 4 <= count; row += 4) {
        const double* first = matrix + std::size_t(row) * rowSkip;
        __m256d sum0 = _mm256_setzero_pd(), sum1 = sum0, sum2 = sum0, sum3 = sum0;
        for (unsigned col = 0; col < row; col += 4) {
            const __m256d values = _mm256_loadu_pd(rhs + col);
            sum0 = _mm256_add_pd(sum0, _mm256_mul_pd(_mm256_loadu_pd(first + col), values));
            sum1 = _mm256_add_pd(sum1, _mm256_mul_pd(_mm256_loadu_pd(first + rowSkip + col), values));
            sum2 = _mm256_add_pd(sum2, _mm256_mul_pd(_mm256_loadu_pd(first + 2 * rowSkip + col), values));
            sum3 = _mm256_add_pd(sum3, _mm256_mul_pd(_mm256_loadu_pd(first + 3 * rowSkip + col), values));
        }
        rhs[row] -= horizontalSum(sum0);
        rhs[row+1] = rhs[row+1] - horizontalSum(sum1) - first[rowSkip+row] * rhs[row];
        rhs[row+2] = rhs[row+2] - horizontalSum(sum2) -
            first[2*rowSkip+row] * rhs[row] - first[2*rowSkip+row+1] * rhs[row+1];
        rhs[row+3] = rhs[row+3] - horizontalSum(sum3) -
            first[3*rowSkip+row] * rhs[row] - first[3*rowSkip+row+1] * rhs[row+1] -
            first[3*rowSkip+row+2] * rhs[row+2];
    }
    for (; row < count; ++row) {
        double value = rhs[row];
        for (unsigned col = 0; col < row; ++col)
            value -= matrix[std::size_t(row)*rowSkip+col] * rhs[col];
        rhs[row] = value;
    }
}
__attribute__((target("avx2"))) inline void solveLowerTransposedAvx2(
    const double* matrix, double* rhs, unsigned count, unsigned rowSkip)
{
    unsigned end = count;
    while (end % 4) {
        const unsigned row = --end;
        double value = rhs[row];
        for (unsigned col = row + 1; col < count; ++col)
            value -= matrix[std::size_t(col)*rowSkip+row] * rhs[col];
        rhs[row] = value;
    }
    // Each vector spans four adjacent columns of the lower triangle.
    while (end) {
        const unsigned row = end - 4;
        __m256d sum0 = _mm256_setzero_pd(), sum1 = sum0, sum2 = sum0, sum3 = sum0;
        unsigned col = end;
        for (; col + 4 <= count; col += 4) {
            sum0 = _mm256_add_pd(sum0, _mm256_mul_pd(
                _mm256_loadu_pd(matrix+std::size_t(col)*rowSkip+row), _mm256_set1_pd(rhs[col])));
            sum1 = _mm256_add_pd(sum1, _mm256_mul_pd(
                _mm256_loadu_pd(matrix+std::size_t(col+1)*rowSkip+row), _mm256_set1_pd(rhs[col+1])));
            sum2 = _mm256_add_pd(sum2, _mm256_mul_pd(
                _mm256_loadu_pd(matrix+std::size_t(col+2)*rowSkip+row), _mm256_set1_pd(rhs[col+2])));
            sum3 = _mm256_add_pd(sum3, _mm256_mul_pd(
                _mm256_loadu_pd(matrix+std::size_t(col+3)*rowSkip+row), _mm256_set1_pd(rhs[col+3])));
        }
        sum0 = _mm256_add_pd(_mm256_add_pd(sum0,sum1),_mm256_add_pd(sum2,sum3));
        for (; col < count; ++col)
            sum0 = _mm256_add_pd(sum0, _mm256_mul_pd(
                _mm256_loadu_pd(matrix+std::size_t(col)*rowSkip+row), _mm256_set1_pd(rhs[col])));
        double sums[4]; _mm256_storeu_pd(sums,sum0);
        rhs[row+3] -= sums[3];
        rhs[row+2] = rhs[row+2] - sums[2] - matrix[std::size_t(row+3)*rowSkip+row+2] * rhs[row+3];
        rhs[row+1] = rhs[row+1] - sums[1] -
            matrix[std::size_t(row+2)*rowSkip+row+1] * rhs[row+2] -
            matrix[std::size_t(row+3)*rowSkip+row+1] * rhs[row+3];
        rhs[row] = rhs[row] - sums[0] - matrix[std::size_t(row+1)*rowSkip+row] * rhs[row+1] -
            matrix[std::size_t(row+2)*rowSkip+row] * rhs[row+2] -
            matrix[std::size_t(row+3)*rowSkip+row] * rhs[row+3];
        end = row;
    }
}
}
#else
#define GRSIM_ODE_HAS_AVX2 0
#endif

namespace grsim_ode {
inline const char* triangularBackend()
{
#if defined(GRSIM_ODE_USE_BLAS) && GRSIM_ODE_USE_BLAS
# if GRSIM_ODE_HAS_AVX2
    if (__builtin_cpu_supports("avx2")) return "AVX2";
# endif
# ifdef __APPLE__
    return "Accelerate";
# else
    return "OpenBLAS";
# endif
#else
    return "ODE";
#endif
}
// These synchronous CPU kernels run on the calling simulation/ODE worker.
inline void initializeMath()
{
#if defined(GRSIM_ODE_USE_BLAS) && GRSIM_ODE_USE_BLAS && !defined(__APPLE__)
    // ODE already schedules independent islands. Avoid nested BLAS parallelism.
    static const bool initialized = []() { openblas_set_num_threads(1); return true; }();
    (void)initialized;
#endif
}
inline bool solveUnitLower(const double* matrix, double* rhs, unsigned count,
                           unsigned rowSkip, unsigned rhsStride, bool transposed)
{
#if defined(GRSIM_ODE_USE_BLAS) && GRSIM_ODE_USE_BLAS
    initializeMath();
    if (count >= 64) {
#if GRSIM_ODE_HAS_AVX2
        if (rhsStride == 1 && __builtin_cpu_supports("avx2")) {
            if (transposed) solveLowerTransposedAvx2(matrix,rhs,count,rowSkip);
            else solveLowerAvx2(matrix,rhs,count,rowSkip);
            return true;
        }
#endif
        cblas_dtrsv(CblasRowMajor, CblasLower, transposed ? CblasTrans : CblasNoTrans,
                    CblasUnit, count, matrix, rowSkip, rhs, rhsStride);
        return true;
    }
#endif
    return false;
}
inline bool dot(const double* first, const double* second, unsigned count,
                unsigned secondStride, double& result)
{
#if defined(GRSIM_ODE_USE_BLAS) && GRSIM_ODE_USE_BLAS
    initializeMath();
    if (count >= 64) {
        result = cblas_ddot(count, first, 1, second, secondStride);
        return true;
    }
#endif
    return false;
}
}
#endif
