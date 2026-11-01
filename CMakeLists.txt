cmake_minimum_required(VERSION 3.5)


## Project branding, version and package mantainer
project(grSim)
set(VERSION "1.0.0a2")
#set(VENDOR "Parsian")
set(MAINTAINER "Mani Monajjemi <mani.monajjemi@gmail.com>")


# some utils and helper vars
string(TOLOWER ${CMAKE_PROJECT_NAME} CMAKE_PROJECT_NAME_LOWER)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
standard_paths(${PROJECT_SOURCE_DIR} bin lib)

# policy regarding how to handle generated stuff, OLD behavior would ignore generated files
# (which includes the generated protobuf cpp files)
if (POLICY CMP0071)
    cmake_policy(SET CMP0071 NEW)
endif()

# policy regarding when to rebuild stuff when external projects downloaded with URL changes
if (POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

include(GNUInstallDirs)

# Keep native Apple builds and their external dependencies on the same target.
set(GRSIM_EXTERNAL_PLATFORM_ARGS)
if(APPLE)
    list(APPEND GRSIM_EXTERNAL_PLATFORM_ARGS
        "-DCMAKE_OSX_ARCHITECTURES:STRING=${CMAKE_OSX_ARCHITECTURES}"
        "-DCMAKE_OSX_DEPLOYMENT_TARGET:STRING=${CMAKE_OSX_DEPLOYMENT_TARGET}")
endif()
option(GRSIM_MACOSX_BUNDLE "Build grSim as a macOS application bundle." ${APPLE})

set(app ${CMAKE_PROJECT_NAME})
# create the target before the sources list is known so that we can call
# add_dependencies(<target> external_proj)
add_executable(${app} "")

# definitions for knowing the OS from the code
if(MSVC)
    add_definitions(-DHAVE_MSVC)
endif()
if(WIN32)
    add_definitions(-DHAVE_WINDOWS)
endif()
if(UNIX)
    add_definitions(-DHAVE_UNIX)
    if(APPLE)
        add_definitions(-DHAVE_MACOSX)
    else()
        #TODO: fix this, say we have FreeBSD, that's not linux
        add_definitions(-DHAVE_LINUX)
    endif()
endif()

# set explicitly the c++ standard to use
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# add src dir to included directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/net)
include_directories(${PROJECT_SOURCE_DIR}/include/physics)


## Handling depenendcies

# we will append all libs to this var
set(libs)

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
list(APPEND libs ${OPENGL_LIBRARIES})
#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIR})

# Qt
if(APPLE AND EXISTS /usr/local/opt/qt)
  # Homebrew installs Qt5 (up to at least 5.9.1) in
  # /usr/local/qt5, ensure it can be found by CMake since
  # it is not in the default /usr/local prefix.
  list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt")
endif()
find_package(Qt5 COMPONENTS Core Widgets OpenGL Network REQUIRED)
list(APPEND libs Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Network)

# ODE
find_package(Threads REQUIRED)
list(APPEND libs Threads::Threads)
option(BUILD_ODE "Build the bundled double-precision ODE dependency." OFF)
set(GRSIM_ODE_MIN_VERSION 0.16.6)
if(BUILD_ODE)
  include(BuildODE)
  add_dependencies(${app} ode_external)
else()
  if(WIN32)
    find_package(ODE ${GRSIM_ODE_MIN_VERSION} QUIET CONFIG)
    if(ODE_FOUND AND NOT ODE_DOUBLE_PRECISION)
      set(ODE_FOUND FALSE)
    endif()
    set(ODE_LIB_NAME ODE::ODE)
  else()
    find_package(ODE ${GRSIM_ODE_MIN_VERSION} QUIET)
    set(ODE_LIB_NAME ode::ode)
  endif()

  if(ODE_FOUND)
    message(STATUS "Using system ODE ${ODE_VERSION}")
    list(APPEND libs ${ODE_LIB_NAME})
  else()
    # Older and single-precision system libraries cannot satisfy this build.
    include(BuildODE)
    add_dependencies(${app} ode_external)
  endif()
endif()

# VarTypes
find_package(VarTypes)

if(NOT VARTYPES_FOUND)
  include(ExternalProject)
  
  # The bundled dependency still requests policies removed by CMake 4.
  set(VARTYPES_CMAKE_ARGS "-DVARTYPES_BUILD_STATIC=ON;-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>;-DCMAKE_POLICY_VERSION_MINIMUM=3.5;${GRSIM_EXTERNAL_PLATFORM_ARGS}")
  if(NOT "${CMAKE_TOOLCHAIN_FILE}" STREQUAL "")
    set(VARTYPES_CMAKE_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE};${VARTYPES_CMAKE_ARGS}")
  endif()

  ExternalProject_Add(vartypes_external
    GIT_REPOSITORY    https://github.com/jpfeltracco/vartypes
    GIT_TAG           origin/jpfeltracco/build_static
    UPDATE_DISCONNECTED TRUE
    CMAKE_ARGS        "${VARTYPES_CMAKE_ARGS}"
    STEP_TARGETS install
  )
  set(VARTYPES_LIB_SUBPATH "${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}vartypes${CMAKE_STATIC_LIBRARY_SUFFIX}")

  # the byproducts are available after the install step
  ExternalProject_Add_Step(vartypes_external out
    DEPENDEES install
    BYPRODUCTS
        "<INSTALL_DIR>/${VARTYPES_LIB_SUBPATH}"
  )

  add_dependencies(${app} vartypes_external)

  ExternalProject_Get_Property(vartypes_external install_dir)
  add_dependencies(${app} vartypes_external)

  set(VARTYPES_INCLUDE_DIRS "${install_dir}/include")
  set(VARTYPES_LIBRARIES "${install_dir}/${VARTYPES_LIB_SUBPATH}")
endif()

target_include_directories(${app} PRIVATE ${VARTYPES_INCLUDE_DIRS})
list(APPEND libs ${VARTYPES_LIBRARIES})

# Protobuf
include(FindOrBuildProtobuf)

if(TARGET protobuf_external)
    add_dependencies(${app} protobuf_external)
endif()

include_directories(${Protobuf_INCLUDE_DIRS})
list(APPEND libs ${Protobuf_LIBRARIES})

set (Protobuf_IMPORT_DIRS ${Protobuf_INCLUDE_DIR})
protobuf_generate_cpp(PROTO_CPP PROTO_H
    src/proto/grSim_Commands.proto
    src/proto/grSim_Packet.proto
    src/proto/grSim_Replacement.proto
    src/proto/grSim_Robotstatus.proto
    src/proto/ssl_gc_common.proto
    src/proto/ssl_simulation_config.proto
    src/proto/ssl_simulation_control.proto
    src/proto/ssl_simulation_error.proto
    src/proto/ssl_simulation_robot_control.proto
    src/proto/ssl_simulation_robot_feedback.proto
    src/proto/ssl_simulation_synchronous.proto
    src/proto/ssl_vision_detection.proto
    src/proto/ssl_vision_geometry.proto
    src/proto/ssl_vision_wrapper.proto
)

qt5_add_resources(RESOURCES
    resources/textures.qrc
)

set(RESOURCES
    ${RESOURCES}
    resources/grsim.rc
)

set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/glwidget.cpp
    src/graphics.cpp
    src/physics/pworld.cpp
    src/physics/pobject.cpp
    src/physics/pball.cpp
    src/physics/pground.cpp
    src/physics/pfixedbox.cpp
    src/physics/pcylinder.cpp
    src/physics/pchassiscollision.cpp
    src/physics/pbox.cpp
    src/physics/pray.cpp
    src/net/robocup_ssl_server.cpp
    src/net/robocup_ssl_client.cpp
    src/sslworld.cpp
    src/simulation.cpp
    src/scenerenderer.cpp
    src/robot.cpp
    src/configwidget.cpp
    src/statuswidget.cpp
    src/logger.cpp
    src/robotwidget.cpp
    src/getpositionwidget.cpp
)

set(HEADERS
    include/mainwindow.h
    include/glwidget.h
    include/graphics.h
    include/physics/pworld.h
    include/physics/pobject.h
    include/physics/pball.h
    include/physics/pground.h
    include/physics/pfixedbox.h
    include/physics/pcylinder.h
    include/physics/pchassiscollision.h
    include/physics/pbox.h
    include/physics/pray.h
    include/net/robocup_ssl_server.h
    include/net/robocup_ssl_client.h
    include/sslworld.h
    include/simulation.h
    include/simulationsettings.h
    include/scenesnapshot.h
    include/scenerenderer.h
    include/robot.h
    include/configwidget.h
    include/statuswidget.h
    include/logger.h
    include/robotwidget.h
    include/getpositionwidget.h
    include/common.h
    include/config.h
)

# files to be compiled
set(srcs
    ${PROTO_CPP}
    ${PROTO_H}
    ${RESOURCES}
    ${HEADERS}
    ${SOURCES}
)


set(CORE_SOURCES ${srcs})
list(REMOVE_ITEM CORE_SOURCES src/main.cpp)
add_library(grSim_core STATIC ${CORE_SOURCES})
set_target_properties(grSim_core PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(grSim_core PUBLIC $<TARGET_PROPERTY:${app},INCLUDE_DIRECTORIES>)
target_link_libraries(grSim_core PUBLIC ${libs})
foreach(dependency vartypes_external protobuf_external ode_external)
    if(TARGET ${dependency})
        add_dependencies(grSim_core ${dependency})
    endif()
endforeach()
target_sources(${app} PRIVATE src/main.cpp)
if(APPLE AND GRSIM_MACOSX_BUNDLE)
    file(GLOB CONFIG_FILES "config/*.ini")
    set_source_files_properties(${CONFIG_FILES} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources/config")
    set(app_icon "${PROJECT_SOURCE_DIR}/resources/icons/grsim.icns")
    set_source_files_properties("${app_icon}" PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
    target_sources(${app} PRIVATE ${CONFIG_FILES} "${app_icon}")
    set_target_properties(${app} PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_GUI_IDENTIFIER "org.robocup-ssl.grsim"
        MACOSX_BUNDLE_ICON_FILE "grsim.icns"
        MACOSX_BUNDLE_BUNDLE_VERSION "${VERSION}"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "${VERSION}")
endif()
install(TARGETS ${app} BUNDLE DESTINATION . RUNTIME DESTINATION bin)
target_link_libraries(${app} grSim_core)

option(BUILD_TESTING "Build simulator regression tests." OFF)
if(BUILD_TESTING)
    enable_testing()
    # Exercise the actual application executable, including a relocated bundle.
    target_sources(${app} PRIVATE tests/application_smoke.cpp)
    target_compile_definitions(${app} PRIVATE GRSIM_BUILD_SMOKE_TEST)
    add_test(NAME application_smoke COMMAND $<TARGET_FILE:${app}> --smoke-test)
    set_tests_properties(application_smoke PROPERTIES TIMEOUT 20 LABELS "opengl")
    if(UNIX AND NOT APPLE)
        set_tests_properties(application_smoke PROPERTIES
            ENVIRONMENT "QT_QPA_PLATFORM=xcb;LIBGL_ALWAYS_SOFTWARE=1")
    endif()
    add_executable(grSim_timing_test tests/timing_regression.cpp)
    target_link_libraries(grSim_timing_test grSim_core)
    add_test(NAME timing_regression COMMAND grSim_timing_test)
    set_tests_properties(timing_regression PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
        TIMEOUT 20)
    add_test(NAME timing_observer_regression COMMAND grSim_timing_test --check-observer vision)
    add_test(NAME timing_feedback_regression COMMAND grSim_timing_test --check-observer feedback)
    set_tests_properties(timing_observer_regression timing_feedback_regression PROPERTIES TIMEOUT 10)
    add_executable(grSim_rendering_test tests/rendering_regression.cpp)
    target_link_libraries(grSim_rendering_test grSim_core)
    add_test(NAME rendering_regression COMMAND grSim_rendering_test)
    set_tests_properties(rendering_regression PROPERTIES TIMEOUT 30 LABELS "opengl")
    if(UNIX AND NOT APPLE)
        set_tests_properties(rendering_regression PROPERTIES
            ENVIRONMENT "QT_QPA_PLATFORM=xcb;LIBGL_ALWAYS_SOFTWARE=1")
        add_test(NAME rendering_regression_hidpi COMMAND grSim_rendering_test)
        set_tests_properties(rendering_regression_hidpi PROPERTIES
            ENVIRONMENT "QT_QPA_PLATFORM=xcb;LIBGL_ALWAYS_SOFTWARE=1;QT_SCALE_FACTOR=2"
            TIMEOUT 30 LABELS "opengl")
    endif()
    add_test(NAME timing_rendering_regression COMMAND grSim_timing_test --rendering)
    set_tests_properties(timing_rendering_regression PROPERTIES TIMEOUT 30 LABELS "opengl")
    if(UNIX AND NOT APPLE)
        set_tests_properties(timing_rendering_regression PROPERTIES
            ENVIRONMENT "QT_QPA_PLATFORM=xcb;LIBGL_ALWAYS_SOFTWARE=1")
    endif()
    add_executable(grSim_worker_test tests/worker_regression.cpp)
    target_link_libraries(grSim_worker_test grSim_core)
    add_test(NAME worker_regression COMMAND grSim_worker_test)
    set_tests_properties(worker_regression PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 30)
    add_executable(grSim_settings_test tests/settings_regression.cpp)
    target_link_libraries(grSim_settings_test grSim_core)
    add_test(NAME settings_regression COMMAND grSim_settings_test)
    set_tests_properties(settings_regression PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 10)
    add_executable(grSim_configuration_test tests/configuration_regression.cpp)
    target_link_libraries(grSim_configuration_test grSim_core)
    add_test(NAME configuration_regression COMMAND grSim_configuration_test)
    set_tests_properties(configuration_regression PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 20)
    add_executable(grSim_collision_test tests/collision_regression.cpp)
    target_link_libraries(grSim_collision_test grSim_core)
    add_test(NAME collision_regression COMMAND grSim_collision_test)
    set_tests_properties(collision_regression PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 20)
    add_executable(grSim_geometry_test tests/geometry_regression.cpp)
    target_link_libraries(grSim_geometry_test grSim_core)
    add_test(NAME geometry_regression COMMAND grSim_geometry_test)
    set_tests_properties(geometry_regression PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 15)
    add_test(NAME worker_regression_ode_threads COMMAND grSim_worker_test)
    set_tests_properties(worker_regression_ode_threads PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen;GRSIM_ODE_THREADS=2" TIMEOUT 30)
    add_executable(grSim_ode_test tests/ode_regression.cpp)
    target_link_libraries(grSim_ode_test grSim_core)
    add_test(NAME ode_regression COMMAND grSim_ode_test)
    set_tests_properties(ode_regression PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 180)
    add_executable(grSim_collision_filtering_test tests/collision_filtering_regression.cpp)
    target_link_libraries(grSim_collision_filtering_test grSim_core)
    add_test(NAME collision_filtering_regression COMMAND grSim_collision_filtering_test)
    set_tests_properties(collision_filtering_regression PROPERTIES TIMEOUT 30)
    add_executable(grSim_physics_defaults_test tests/physics_defaults_regression.cpp)
    target_link_libraries(grSim_physics_defaults_test grSim_core)
    add_test(NAME physics_defaults_regression COMMAND grSim_physics_defaults_test)
    set_tests_properties(physics_defaults_regression PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 30)
    add_executable(grSim_physics_experiments tests/physics_experiments.cpp)
    target_link_libraries(grSim_physics_experiments grSim_core)
    add_executable(grSim_ode_performance_experiments tests/ode_performance_experiments.cpp)
    target_link_libraries(grSim_ode_performance_experiments grSim_core)
    find_program(GRSIM_PYTHON3_EXECUTABLE python3)
    if(GRSIM_PYTHON3_EXECUTABLE)
        add_test(NAME collision_filtering_trajectories
            COMMAND ${GRSIM_PYTHON3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/check_collision_filtering.py
                    $<TARGET_FILE:grSim_ode_performance_experiments>)
        set_tests_properties(collision_filtering_trajectories PROPERTIES TIMEOUT 180)
        add_test(NAME physics_analytic_regression
            COMMAND ${GRSIM_PYTHON3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/check_physics_experiments.py
                    $<TARGET_FILE:grSim_physics_experiments>)
        set_tests_properties(physics_analytic_regression PROPERTIES TIMEOUT 60)
    endif()
endif()

if(APPLE AND GRSIM_MACOSX_BUNDLE)
  # Deploy Qt into the installed bundle with macdeployqt before distribution.
  set(CPACK_GENERATOR "DragNDrop" "TGZ")
elseif(WIN32 AND CMAKE_WIN32_EXECUTABLE)
  # use CMAKE_WIN32_EXECUTABLE if you want to build a windows exe
  install(DIRECTORY config DESTINATION .)
  install(DIRECTORY bin DESTINATION .
          FILES_MATCHING PATTERN "*.dll")
  set(CPACK_PACKAGE_EXECUTABLES ${app} ${app})
else()
  install(DIRECTORY config DESTINATION share/${app})
  install(FILES resources/grsim.desktop DESTINATION share/applications)
  install(FILES resources/icons/grsim.svg DESTINATION share/icons/hicolor/scalable/apps)
endif()

option(BUILD_CLIENTS "Choose this option if you want to build the example Qt client." ON)
if(BUILD_CLIENTS)
    add_subdirectory(clients/qt)
endif()

file(COPY README.md LICENSE.md DESTINATION ${CMAKE_BINARY_DIR})
file(RENAME ${CMAKE_BINARY_DIR}/README.md ${CMAKE_BINARY_DIR}/README.txt)
file(RENAME ${CMAKE_BINARY_DIR}/LICENSE.md ${CMAKE_BINARY_DIR}/LICENSE.txt)

## Packaging
if(UNIX)
    execute_process(COMMAND uname -p OUTPUT_VARIABLE ARCH)
    string(STRIP ${ARCH} ARCH)
    if(APPLE)
        set(ARCH "macos-${CMAKE_SYSTEM_PROCESSOR}")
    endif()
elseif(WIN32)
    set(ARCH "win32")
    set(CPACK_GENERATOR ZIP NSIS)
endif()
set(CPACK_OUTPUT_FILE_PREFIX ${PROJECT_SOURCE_DIR}/dist)
set(CPACK_PACKAGE_CONTACT ${MAINTAINER})
if(VENDOR)
    set(CPACK_PACKAGE_VENDOR ${VENDOR})
    string(TOLOWER ${CPACK_PACKAGE_VENDOR} FLAVOR)
endif()
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_BINARY_DIR}/LICENSE.txt")
set(CPACK_RESOURCE_FILE_README "${CMAKE_BINARY_DIR}/README.txt")
#set(CPACK_RESOURCE_FILE_WELCOME "${CMAKE_SOURCE_DIR}/WELCOME.txt")
set(CPACK_PACKAGE_VERSION ${VERSION})
# Debian based specific
set(CPACK_DEBIAN_PACKAGE_DEPENDS "vartypes (>=0.7.0)")
# Derive shared-library dependencies, including ODE only for a system build.
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
if(FLAVOR)
    set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME_LOWER}_${CPACK_PACKAGE_VERSION}-${FLAVOR}_${ARCH}")
else()
    set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME_LOWER}_${CPACK_PACKAGE_VERSION}_${ARCH}")
endif()
include(CPack)


