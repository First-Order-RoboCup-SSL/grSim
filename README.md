[![Build Status](https://github.com/RoboCup-SSL/grSim/workflows/Build/badge.svg)](https://github.com/RoboCup-SSL/grSim/actions?query=workflow%3ABuild+branch%3Amaster) [![CodeFactor](https://www.codefactor.io/repository/github/robocup-ssl/grsim/badge/master)](https://www.codefactor.io/repository/github/robocup-ssl/grsim/overview/master)

grSim
=======================

[RoboCup Small Size League](https://ssl.robocup.org/) Simulator.

![grSim on Ubuntu](docs/img/screenshot01.jpg?raw=true "grSim on Ubuntu")

- [Install instructions](INSTALL.md)
- [Authors](AUTHORS.md)
- [Changelog](CHANGELOG.md)
- License: [GNU General Public License (GPLv3)](LICENSE.md)

System Requirements
-----------------------

grSim will likely run on a modern dual-core PC with a decent graphics card. A typical configuration is:

- Dual Core CPU (2.0 Ghz+)
- 1GB of RAM
- 256MB nVidia or ATI graphics card

Note that it may run on lower-end equipment though good performance is not guaranteed.


Software Requirements
---------------------

grSim compiles on Linux (tested on Ubuntu and Arch Linux variants only) and Mac OS. It depends on the following libraries:

- [CMake](https://cmake.org/) version 3.5+
- [pkg-config](https://freedesktop.org/wiki/Software/pkg-config/)
- [OpenGL](https://www.opengl.org)
- [Qt5 Development Libraries](https://www.qt.io)
- [Open Dynamics Engine (ODE)](https://www.ode.org/) 0.16.6+, double precision, with a pinned source fallback
- [VarTypes Library](https://github.com/jpfeltracco/vartypes) forked from [Szi's Vartypes](https://github.com/szi/vartypes)
- [Google Protobuf](https://github.com/google/protobuf)
- [Boost development libraries](http://www.boost.org/) (needed by VarTypes)

Please consult the [install instructions](INSTALL.md) for more details.

Usage
-----

Receiving data from the grSim is similar to receiving data from the [SSL-Vision](https://github.com/RoboCup-SSL/ssl-vision) using [Google Protobuf](https://github.com/google/protobuf) library.
Sending data to the simulator is also possible using Google Protobuf. Sample clients are included in [clients](./clients) folder. There are two clients available, *qt-based* and *Java-based*. The native client is compiled during the grSim compilation. To compile the Java client, please consult the corresponding `README` file.

Qt [example project](https://github.com/robocin/ssl-client) to receive and send data to the simulator.

Physics and command handling run independently of the display. The render FPS cap
does not change the configured physics timestep. If physics cannot keep up with
real time, a small amber **Lag** warning appears in the top-right corner of the
view. The simulator retains pending physics steps and gives them priority over
rendering. The warning clears after it catches up.

Regression tests
----------------

Build the tests with `cmake -S . -B build -DBUILD_TESTING=ON`, then
`cmake --build build`. Run `ctest --output-on-failure` from the build directory.
The timing tests use loopback UDP and an independent observer process. Rendering
tests need a working OpenGL display; Linux CI uses Mesa and
`xvfb-run -a ctest --output-on-failure`. Timing is checked both headlessly and
with real OpenGL drawing, a 90 ms delay per rendered frame, and a 500 ms GUI
stall. The observer checks packet freshness at the end and requires every sent
command to receive feedback; a separate fixture verifies that a silent final
500 ms fails.

Capture checks detect blank images through size, opacity, and color diversity;
they do not verify textures, field markings, or visual fidelity. OpenGL feedback
checks separately verify that Low Spec reduces textured wheel and chassis
geometry. Settings tests load historical XML keys and check save/reload behavior.
Configuration tests delay GUI notifications while remote robot specifications
and GUI edits interleave, including queued preset selections.
Geometry tests check both divisions' 2026 margins and substitution widths in
outgoing vision packets, plus wall placement after live settings edits.
Collision tests check side and rear ball contact, dribbler access, roller
retention while stationary and moving, and flat/chip kicks for separate team
sizes and rotated robots, including ball acquisition with recessed dribblers.
Robot-pair regressions also cover side/rear pushing, rotated and different-sized
robots, with serial ODE and optional helpers. Robot contact spheres match the
outer shell radius and attach directly to the chassis, avoiding an extra fixed
body while preserving the previous combined mass and inertia. The kinematic
ball/hull fixtures isolate wheel-ground and rolling constraints; dynamic physics
and robot-pair tests retain the production defaults.
The ball-contact hull keeps the body outline and cuts its front back when needed
to reach the configured dribbler. Render quality does not change this shape.
Tests use temporary configuration values without reading or writing personal
simulator settings. In the configuration tree, **Desired FPS** caps rendering and
**ODE time step** sets the fixed physics step; their names remain stable for saved
settings compatibility.

The ODE regressions cover ball resistance, cylinder inertia, settings migration,
collision-filtered contacts/trajectories, helper stepping and concurrent worlds.
Helpers are disabled by default; see the
[setup and limitations](INSTALL.md#experimental-ode-helper-threads).
Ball physics defaults and migration are described in the
[installation guide](INSTALL.md#ball-physics-defaults).

For a repeatable physics-only benchmark after building the tests (6 v 6 by default):

    python3 tests/benchmark_ode.py bin/grSim_ode_test --repeats 3 --steps 300 --output /tmp/grsim-ode-benchmark.json

For sustained opposing-team contact on a Division B field:

    QT_QPA_PLATFORM=offscreen ./bin/grSim_ode_performance_experiments --scenario pushing-6 --helpers 3 --islands 3 --steps 300

The performance harness also provides spaced-6 and crowded-6; each uses twelve
robots total. Add `--audit 1` to report maximum planar robot-shell overlap and
contact/constraint diagnostics. The six-a-side regression rejects overlap above
2 mm. These benchmarks exclude rendering, networking and real-time
throttling. Compare builds using the same geometry, physics settings and workload.

Star History
------
[![Star History Chart](https://api.star-history.com/svg?repos=robocup-ssl/grsim&type=Date)](https://star-history.com/#robocup-ssl/grsim&Date)

Citing
------

If you use this in your research, please cite the original paper:
```
@inproceedings{Monajjemi2011grSimR,
  title={grSim - RoboCup Small Size Robot Soccer Simulator},
  author={Valiallah Monajjemi and A. Koochakzadeh and S. S. Ghidary},
  booktitle={RoboCup},
  year={2011}
}
```

If you wish to cite this repo with it's modifications specifically, please cite:

```
@misc{grsim2021,
  author = {Mohammad Mahdi Rahimi and Jan Segre and Valiallah Monajjemi and A. Koochakzadeh and Sepehr MohaimenianPour and Nicolai Ommer and  Avatar
Kazunori Kimura and Jeremy Feltracco and Kenta Sato and Atousa Ahsani},
  title = {GRSIM},
  year = {2021},
  publisher = {GitHub},
  note = {GitHub repository},
  howpublished = {\url{https://github.com/RoboCup-SSL/grSim/}}
}
```
