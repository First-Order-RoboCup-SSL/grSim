#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "grsim_ode_math.h"

using dReal = double;
using sizeint = std::size_t;
static void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
#define dIASSERT(value) require((value), #value)
namespace reference {
#include "reference_fastlsolve_impl.h"
#include "reference_fastltsolve_impl.h"
#include "reference_fastdot_impl.h"
}
namespace optimized {
#include "optimized_fastlsolve_impl.h"
#include "optimized_fastltsolve_impl.h"
#include "optimized_fastdot_impl.h"
}
#undef dIASSERT

static double randomValue(std::uint64_t& state)
{
    state ^= state << 13; state ^= state >> 7; state ^= state << 17;
    return double(state % 20001)/10000-1;
}

template<unsigned stride>
static double checkSolve(unsigned count, bool transposed, bool sparse, unsigned seed)
{
    const unsigned rowSkip = count+1+seed%4;
    const double guard = 1234567.25;
    std::uint64_t state = 166+seed;
    // Diagonal/upper/padding are deliberately NaN: a unit triangular solve
    // must not use them. Exercise non-aligned row pitches and strided RHS data.
    std::vector<double> matrix(sizeint(count)*rowSkip,
                               std::numeric_limits<double>::quiet_NaN());
    for (unsigned row=0; row<count; ++row) for (unsigned col=0; col<row; ++col)
        matrix[sizeint(row)*rowSkip+col] =
            sparse && (row/8)%3 != (col/8)%3 ? 0 : 0.02*randomValue(state);
    std::vector<double> original(sizeint(count)*stride+4,guard);
    for (unsigned i=0; i<count; ++i)
        original[2+sizeint(i)*stride] = i%7 ? randomValue(state) : 0;
    auto expected=original, actual=original;
    double* a=actual.data()+2;
    double* e=expected.data()+2;
    if (transposed) {
        reference::solveL1Transposed<stride>(matrix.data(),e,count,rowSkip);
        optimized::solveL1Transposed<stride>(matrix.data(),a,count,rowSkip);
    } else {
        reference::solveL1Straight<stride>(matrix.data(),e,count,rowSkip);
        optimized::solveL1Straight<stride>(matrix.data(),a,count,rowSkip);
    }
    double norm=1, difference=0;
    for (unsigned i=0; i<count; ++i) {
        require(std::isfinite(a[sizeint(i)*stride]),"nonfinite triangular solution");
        norm=std::max(norm,std::abs(e[sizeint(i)*stride]));
        difference=std::max(difference,std::abs(a[sizeint(i)*stride]-e[sizeint(i)*stride]));
    }
    const double tolerance=64*std::numeric_limits<double>::epsilon()*count;
    require(difference/norm <= tolerance,"triangular solve differs from pinned ODE reference");
    for (unsigned i=0; i<actual.size(); ++i)
        if (i<2 || (i-2)%stride || (i-2)/stride>=count)
            require(actual[i]==guard,"triangular solve overwrote RHS padding");
    // Independently check the equations in long double.
    for (unsigned row=0; row<count; ++row) {
        long double sum=a[sizeint(row)*stride], magnitude=std::abs(sum);
        for (unsigned col=0; col<count; ++col) {
            if ((!transposed && col<row) || (transposed && col>row)) {
                const double coefficient=transposed ? matrix[sizeint(col)*rowSkip+row] :
                                                        matrix[sizeint(row)*rowSkip+col];
                const long double term=static_cast<long double>(coefficient)*a[sizeint(col)*stride];
                sum+=term; magnitude+=std::abs(term);
            }
        }
        const long double rhs=original[2+sizeint(row)*stride];
        require(std::abs(sum-rhs) <= tolerance*std::max(1.L,magnitude+std::abs(rhs)),
                "triangular solution violates original equations");
    }
    return difference/norm;
}

template<unsigned stride>
static void checkDot(unsigned count)
{
    std::uint64_t state=166+count;
    std::vector<double> a(count), b(sizeint(count)*stride+2,777.25);
    long double sum=0, magnitude=0;
    for (unsigned i=0; i<count; ++i) {
        a[i]=i%5 ? randomValue(state) : 0;
        b[sizeint(i)*stride]=randomValue(state);
        const long double product=static_cast<long double>(a[i])*b[sizeint(i)*stride];
        sum+=product; magnitude+=std::abs(product);
    }
    const double actual=optimized::calculateLargeVectorDot<stride>(a.data(),b.data(),count);
    const double expected=reference::calculateLargeVectorDot<stride>(a.data(),b.data(),count);
    const double tolerance=64*std::numeric_limits<double>::epsilon()*count;
    require(std::abs(actual-expected) <= tolerance*std::max(1.L,magnitude),
            "dot differs from pinned ODE reference");
    require(std::abs(actual-sum) <= tolerance*std::max(1.L,magnitude),
            "dot violates long-double reference");
}

int main()
{
    try {
        // First backend calls are concurrent, as with separate ODE worlds/islands.
        std::string errors[8];
        std::vector<std::thread> threads;
        for (unsigned i=0; i<8; ++i) threads.emplace_back([&,i]() {
            try {checkSolve<1>(65,i%2,true,i);checkDot<2>(129);}
            catch (const std::exception& e) {errors[i]=e.what();}
        });
        for (auto& thread:threads) thread.join();
        for (const auto& error:errors) require(error.empty(),error.c_str());
        unsigned solves=0;
        double largest=0;
        for (unsigned n : {1U,2U,3U,4U,5U,7U,12U,15U,16U,31U,32U,33U,63U,64U,65U,
                           66U,67U,95U,96U,97U,127U,128U,129U,191U,255U,256U,257U}) {
            for (bool transposed : {false,true}) for (bool sparse : {false,true}) {
                largest=std::max(largest,checkSolve<1>(n,transposed,sparse,n));
                largest=std::max(largest,checkSolve<2>(n,transposed,sparse,n+1));
                solves+=2;
            }
            checkDot<1>(n);checkDot<2>(n);
        }
        double rhs[4]={1,2,3,4}, result=777;
        require(!grsim_ode::solveUnitLower(rhs,rhs,1,1,1,false) && rhs[0]==1,
                "small solve must retain ODE fallback");
        require(!grsim_ode::dot(rhs,rhs,1,1,result) && result==777,
                "small dot must retain ODE fallback");
#if GRSIM_ODE_USE_BLAS && !defined(__APPLE__)
        require(openblas_get_num_threads()==1,"BLAS must not nest an ODE worker pool");
#endif
        std::printf("PASS ODE CPU math: %u triangular cases, strided dots, 8 concurrent callers; "
                    "max normalized reference difference %.3g\n",solves,largest);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr,"FAIL ODE CPU math: %s\n",error.what());
        return 1;
    }
}
