include(ExternalProject)
find_package(Threads REQUIRED)
find_package(Git REQUIRED)
option(GRSIM_ODE_OPTIMIZED_MATH "Use native CPU math for large bundled ODE kernels" ON)
set(GRSIM_ODE_MATH_AVAILABLE FALSE)
set(GRSIM_ODE_MATH_NAME ODE)
set(GRSIM_ODE_MATH_LIBRARIES "")
set(GRSIM_ODE_MATH_INCLUDE_DIR "")
if(GRSIM_ODE_OPTIMIZED_MATH)
  if(APPLE)
    find_library(GRSIM_ACCELERATE_LIBRARY Accelerate)
    if(GRSIM_ACCELERATE_LIBRARY)
      set(GRSIM_ODE_MATH_AVAILABLE TRUE)
      set(GRSIM_ODE_MATH_NAME Accelerate)
      set(GRSIM_ODE_MATH_LIBRARIES "${GRSIM_ACCELERATE_LIBRARY}")
    endif()
  else()
    find_path(GRSIM_OPENBLAS_INCLUDE_DIR cblas.h PATH_SUFFIXES openblas openblas-pthread)
    find_library(GRSIM_OPENBLAS_LIBRARY NAMES openblas PATH_SUFFIXES openblas-pthread)
    if(GRSIM_OPENBLAS_INCLUDE_DIR AND GRSIM_OPENBLAS_LIBRARY)
      set(GRSIM_ODE_MATH_AVAILABLE TRUE)
      set(GRSIM_ODE_MATH_NAME OpenBLAS)
      set(GRSIM_ODE_MATH_LIBRARIES "${GRSIM_OPENBLAS_LIBRARY}")
      set(GRSIM_ODE_MATH_INCLUDE_DIR "${GRSIM_OPENBLAS_INCLUDE_DIR}")
    endif()
  endif()
endif()
mark_as_advanced(GRSIM_OPENBLAS_INCLUDE_DIR GRSIM_OPENBLAS_LIBRARY GRSIM_ACCELERATE_LIBRARY)
set(grsim_ode_patch_dir "${CMAKE_CURRENT_LIST_DIR}/../ode")

set(ode_build_type "${CMAKE_BUILD_TYPE}")
if(NOT ode_build_type)
  set(ode_build_type Release)
endif()

# Pin the 0.16.6 release, including its threading interface and collision TLS.
ExternalProject_Add(ode_external
    GIT_REPOSITORY    https://github.com/odedevs/ode.git
    GIT_TAG           a5fc0bc282703ab34d7d3d2500bafae7428eeaa9
    UPDATE_DISCONNECTED TRUE
    CMAKE_ARGS
                      ${GRSIM_EXTERNAL_PLATFORM_ARGS}
                      # Keep compatibility with the dependency on CMake 4.
                      -DCMAKE_POLICY_VERSION_MINIMUM=3.5
                      -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
                      -DCMAKE_INSTALL_LIBDIR:PATH=${CMAKE_INSTALL_LIBDIR}
                      -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
                      -DCMAKE_C_COMPILER:PATH=${CMAKE_C_COMPILER}
                      -DCMAKE_CXX_COMPILER:PATH=${CMAKE_CXX_COMPILER}
                      -DCMAKE_MAKE_PROGRAM:PATH=${CMAKE_MAKE_PROGRAM}
                      -DCMAKE_BUILD_TYPE:STRING=${ode_build_type}
                      -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
                      -DBUILD_SHARED_LIBS=OFF
                      -DGRSIM_ODE_USE_BLAS:BOOL=${GRSIM_ODE_MATH_AVAILABLE}
                      -DGRSIM_ODE_BLAS_LIBRARY:FILEPATH=${GRSIM_ODE_MATH_LIBRARIES}
                      -DGRSIM_ODE_BLAS_INCLUDE_DIR:PATH=${GRSIM_ODE_MATH_INCLUDE_DIR}
                      -DODE_DOUBLE_PRECISION=ON
                      -DODE_WITH_DEMOS=OFF
                      -DODE_WITH_TESTS=OFF
                      -DODE_WITH_OU=ON
                      -DODE_NO_THREADING_INTF=OFF
                      -DODE_NO_BUILTIN_THREADING_IMPL=OFF
                      -DODE_WIN32_LIB_OUTPUT_NAME_BASED_ON_FLOAT_SIZE=OFF
                      -DODE_WIN32_STATIC_LIB_POSTFIX_DEBUG:STRING=
                      -DODE_WIN32_STATIC_LIB_POSTFIX_MINSIZEREL:STRING=
                      -DODE_WIN32_STATIC_LIB_POSTFIX_RELEASE:STRING=
                      -DODE_WIN32_STATIC_LIB_POSTFIX_RELWITHDEBINFO:STRING=
  STEP_TARGETS install
)

ExternalProject_Add_Step(ode_external grsim_math_patch
  COMMAND "${CMAKE_COMMAND}" "-DSOURCE_DIR=<SOURCE_DIR>"
          "-DPATCH_FILE=${grsim_ode_patch_dir}/cpu-math.patch"
          "-DHELPER_FILE=${grsim_ode_patch_dir}/grsim_ode_math.h"
          "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
          -P "${grsim_ode_patch_dir}/ApplyPatch.cmake"
  DEPENDEES patch
  DEPENDERS configure
  DEPENDS "${grsim_ode_patch_dir}/cpu-math.patch"
          "${grsim_ode_patch_dir}/grsim_ode_math.h"
          "${grsim_ode_patch_dir}/ApplyPatch.cmake")
ExternalProject_Get_Property(ode_external source_dir)
set(GRSIM_BUNDLED_ODE_SOURCE_DIR "${source_dir}")

set(ODE_LIB_SUBPATH "${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}ode${CMAKE_STATIC_LIBRARY_SUFFIX}")

# the byproducts are available after the install step
ExternalProject_Add_Step(ode_external out
  DEPENDEES install
  BYPRODUCTS
      "<INSTALL_DIR>/${ODE_LIB_SUBPATH}"
)

ExternalProject_Get_Property(ode_external install_dir)
# Imported include directories must exist before the external installation.
file(MAKE_DIRECTORY "${install_dir}/include")
add_library(ode::ode STATIC IMPORTED)
set_target_properties(ode::ode PROPERTIES
  IMPORTED_LOCATION "${install_dir}/${ODE_LIB_SUBPATH}"
  INTERFACE_INCLUDE_DIRECTORIES "${install_dir}/include"
  INTERFACE_COMPILE_DEFINITIONS "dIDEDOUBLE;GRSIM_ODE_MATH_BACKEND=\"${GRSIM_ODE_MATH_NAME}\""
  INTERFACE_LINK_LIBRARIES "Threads::Threads;${GRSIM_ODE_MATH_LIBRARIES}"
)
add_dependencies(ode::ode ode_external)
list(APPEND libs ode::ode)
message(STATUS "Building bundled ODE 0.16.6 (${ode_build_type}, double precision, threading enabled)")

message(STATUS "Bundled ODE large CPU math kernels: ${GRSIM_ODE_MATH_NAME}")
