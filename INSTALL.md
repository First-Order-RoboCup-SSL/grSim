# GrSim - INSTALL

## Overview

We developed grSim on Ubuntu OS. (Ubuntu 14.04+ tested and is recommended). It is  important that the graphics card driver is installed properly (the official Ubuntu packages for nVidia and AMD(ATI) graphics cards are available). grSim will compile and run in both 32 and 64 bits Linux and Mac OS, and in 64 bit Windows. 

GrSim is written in C++, in order to compile it, you will need a working toolchain and a c++ compiler.

## Dependencies

GrSim depends on:

- [CMake](https://cmake.org/) version 3.5+
- [pkg-config](https://www.freedesktop.org/wiki/Software/pkg-config/)
- [OpenGL](https://www.opengl.org)
- [Qt5 Development Libraries](https://www.qt.io)
- [Open Dynamics Engine (ODE)](https://www.ode.org/) 0.16.6 or newer, double precision
- [VarTypes Library](https://github.com/jpfeltracco/vartypes) forked from [Szi's Vartypes](https://github.com/szi/vartypes)
- [Google Protobuf](https://github.com/google/protobuf)
- [Boost development libraries](http://www.boost.org/) (needed by VarTypes)

grSim uses a compatible system ODE when available. Otherwise, CMake downloads and
builds the pinned 0.16.6 release in double precision. Pass `-DBUILD_ODE=ON` to
select this bundled build explicitly. It also enables ODE's threading interface
and collision thread-local storage for independent concurrent worlds. The first
build needs Git and access to the upstream repository. Installing a distribution's
ODE package is optional when using the bundled build.

## Run from pre-build packages
### Installing from Arch Linux package manager

A package of grSim is avaliable on the [Arch User Repository](https://aur.archlinux.org/packages/grsim-git/), you can install it with your preferred AUR manager. Using `yay` it can be done with:
```bash
yay -S grsim-git
```

### Using docker image
You can get latest grSim from [Docker Hub](https://hub.docker.com/r/robocupssl/grsim) with:
```shell
docker pull robocupssl/grsim:latest
```

The container can be run in two flavors:
1. Headless: `docker run robocupssl/grsim`
1. With VNC: `docker run --net=host -eVNC_PASSWORD=vnc -eVNC_GEOMETRY=1920x1080 robocupssl/grsim vnc`
    1. Then launch your VNC client app (e.g. [Remmina](https://remmina.org/)).
    1. Connect to `localhot:5900`.
    1. Enter a password (default:`vnc`) to login.

## Building and installing from the source code

### Installing Dependencies

#### Arch Linux

If you are running Arch Linux or an Arch Linux based distribution, install the dependencies with:
```
$ sudo pacman -S base-devel boost hicolor-icon-theme \
                 mesa ode protobuf qt5-base cmake git
```

#### Ubuntu / Debian

For Debian, or derivative
```
sudo apt install git build-essential cmake pkg-config qtbase5-dev \
                   libqt5opengl5-dev libgl1-mesa-dev libglu1-mesa-dev \
                   libprotobuf-dev protobuf-compiler libode-dev libboost-dev libopenblas-dev
```

#### Mac OS X

For Mac OS X, you will need to have installed:

- [Xcode](https://developer.apple.com/xcode/) or Xcode Command Line Tools 8.0 or newer;
- [Homebrew](http://brew.sh/) package manager.

Then install the dependencies needed:

```bash
brew install cmake pkg-config qt@5
export CMAKE_PREFIX_PATH="$(brew --prefix qt@5)${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
```

Keep this export in the shell used to build grSim so CMake and its dependency
builds can find Qt on both Intel and Apple Silicon. ODE, VarTypes, and a
compatible Protobuf are built automatically when needed.

If you run into build issues, you may need to run this first:

```bash
brew update
brew doctor
```

#### Windows (64 bits)

For Windows, you will need to have installed:

- [CMake](https://cmake.org/) (tested with version 3.17.2 ). Download and install cmake for windows.
- [Visual Studio](https://visualstudio.microsoft.com/) (tested with version 16.7.0). During installation make sure to include workload `Desktop development with C++` and `C++ MFC for latest v142 build tools (x86 x64)`
- [vcpkg](https://github.com/microsoft/vcpkg) package manager. Follow installation instructions on their github website.

To install the dependencies, open a terminal in vcpkg installation folder and run the following command (it will take very long to run):

```bash
$ ./vcpkg install qt5:x64-windows ode:x64-windows protobuf:x64-windows
```

### Building

First clone grSim into your preferred location.

```bash
$ cd /path/to/grsim_ws
$ git clone https://github.com/RoboCup-SSL/grSim.git
$ cd grSim
```

Create a build directory within the project (this is ignored by .gitignore):

```bash
$ mkdir build
$ cd build
```

#### Linux and Mac OS X

Run CMake to generate the makefile (note: if you proceed with the installation, grSim will be installed into directory chosen, by default `/usr/local`):

```bash
$ cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release -DBUILD_ODE=ON ..
```

Then compile the program:

```bash
$ make
```

The executable will be located on the `bin` directory.

#### Windows

Run CMake to generate a solution in visual studio and the build the solution (note: modify the command below to reflect your vcpkg installation folder).

```bash
$ cmake -DCMAKE_TOOLCHAIN_FILE=${PATH_TO_VCPKG}\\scripts\\buildsystems\\vcpkg.cmake ..
$ cmake --build . --config Release
```

The executable will be located on the `bin` directory.

### Installing (Linux and Mac OS X)

At least, if you want to install grSim on your system, run the follow:

```bash
$ sudo make install
```

grSim will be — by default — installed on the `/usr/local` directory.


## Troubleshooting

If you face any problem regarding of updating the grsim version, you can try removing the `grsim.xml`.
If grSim crashes almost instantly with some ODE error the issue might by your ODE version.
Try adding `-DBUILD_ODE=ON` to build the pinned ODE dependency from source.

## Ball physics defaults

Ground rolling/spin resistance applies through ball-ground contacts. Under
Physics → Ball, **Ball rolling deceleration (m/s^2)** defaults to 0.35 m/s².
Spin resistance and held-ball traction retain the Ball-ground friction control;
the published rolling model follows the rolling-deceleration setting.
Slip and linear/angular damping default to zero. A one-time migration replaces
the old unused defaults (slip 1 and damping 0.004); other custom values are kept.
Nonzero body damping affects flight as well as ground motion.

## Experimental ODE helper threads

Physics and networking already run on a simulation worker independently of the
display. ODE helper threads are a separate experiment and are disabled by default.
To test two helpers with the bundled build, launch from the repository root with:

```bash
GRSIM_ODE_THREADS=2 ./bin/grSim  # Linux
GRSIM_ODE_THREADS=2 ./bin/grSim.app/Contents/MacOS/grSim  # macOS bundle
```

`GRSIM_ODE_THREADS` accepts 0 to 16 helpers. Zero runs ODE stepping on the simulation
worker alone. Collision detection stays on that worker; helpers process independent
constraint islands. Robot contacts can join islands and reduce parallelism. The
solver and preparation within each island stay serial in this experiment.

Helper scheduling can change subsequent contact ordering, so crowded trajectories
can diverge. Helper mode does not guarantee equivalent trajectories or better
performance. Leave it disabled for reproducible simulations unless evaluated
against the intended workload.

## CPU math kernels in bundled ODE

The bundled ODE build uses double-precision Accelerate on macOS and OpenBLAS
when available on other platforms for triangular solves and dot products with
at least 64 entries. Small kernels retain ODE's implementation. These synchronous
CPU calls keep the direct solver, collision shapes, materials and five substeps.
OpenBLAS runs with one active math thread per calling ODE worker.

On Ubuntu, install libopenblas-dev before configuring. CMake prints the selected
backend; the performance harness reports it as math_backend. Set
GRSIM_ODE_OPTIMIZED_MATH=OFF when configuring to compare the scalar fallback.
A compatible system ODE is used without this bundled patch.

## Notes on the performance

When running grSim, check the FPS in the status bar. If it is running at **60 FPS** or higher, everything is ok. Otherwise check the graphics card's driver installation and OpenGL settings.


### WSL graphics device

The status bar reports the renderer from the active OpenGL context. If WSL uses
llvmpipe/software rendering and you want to select an NVIDIA adapter, launch
from the repository root with:

```bash
env -u LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER=d3d12 \
  MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ./bin/grSim
```

Check that the Graphics indicator names your NVIDIA GPU. This selects the
graphics device for this launch; physics still runs on its separate CPU worker.


## Apple Silicon application (macOS 15 or later)

The macOS CI jobs build native arm64 applications on macOS 15 and 26. Both jobs
run the simulator regressions and the real application's rendering/UDP smoke
check. The macOS 15 job uploads a ZIP containing a relocated grSim.app with Qt
and robot configuration files. A separate macOS 26 job downloads and runs that
exact ZIP without checking out or building the source. Download the artifact from the successful Build workflow,
extract it, and open grSim.app.

To build locally with Xcode Command Line Tools and Homebrew:

    brew install cmake pkg-config qt@5
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$(brew --prefix qt@5)" \
      -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=15.0 \
      -DBUILD_ODE=ON -DBUILD_TESTING=ON -DGRSIM_MACOSX_BUNDLE=ON
    cmake --build build -j 3
    ctest --test-dir build --output-on-failure
    bash tests/package_macos.sh build

The packaged app uses ad-hoc signing; Developer ID signing and notarization
are not configured. Browser downloads may require
[Open Anyway](https://support.apple.com/en-us/102445). The runtime check exercises normal/Low Spec OpenGL images,
robot configuration and textures, advancing vision packets, robot commands and
feedback, motion and clean worker shutdown. It does not establish visual fidelity
or performance on every Apple Silicon model. Intel/universal builds are not
claimed. Normal builds can disable the bundle with GRSIM_MACOSX_BUNDLE=OFF.

The timing observer reports raw packet latency and its own scheduling pauses.
Linux/default regression latency remains below 75 ms. Shared native macOS CI
uses an explicit 200 ms budget because observer pauses above 100 ms were measured
on macOS 26; it still rejects the 500 ms vision freeze and unanswered-command
negative controls. This is a CI acceptance budget, not a real-time guarantee.
