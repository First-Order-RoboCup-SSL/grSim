#ifndef GRSIM_ODE_MATH_H
#define GRSIM_ODE_MATH_H

#if defined(GRSIM_ODE_USE_BLAS) && GRSIM_ODE_USE_BLAS
# ifdef __APPLE__
#  include <Accelerate/Accelerate.h>
# else
#  include <cblas.h>
# endif
#endif

namespace grsim_ode {
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
    if (count >= 64) {
        initializeMath();
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
    if (count >= 64) {
        initializeMath();
        result = cblas_ddot(count, first, 1, second, secondStride);
        return true;
    }
#endif
    return false;
}
}
#endif
