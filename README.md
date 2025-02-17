# Monocular Pose Solver

## Overview
Uses the following libraries:
- [Eigen](https://eigen.tuxfamily.org): a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms
- [Monocular Pose Estimator](https://github.com/uzh-rpg/rpg_monocular_pose_estimator) from the ETH Robotics Perception Group (RPG): a monocular pose estimation system based on infrared LEDs


## License
- The **Eigen** library is [licensed under MPL2](https://eigen.tuxfamily.org/index.php?title=Main_Page#License).<br>
  Note that currently, a few features rely on third-party code licensed under the LGPL (see [details](https://eigen.tuxfamily.org/index.php?title=Main_Page#License)).
- The **RPG Monocular Pose Estimator** library is licensed under GPL v3.<br>
  Note however the [statement](https://github.com/uzh-rpg/rpg_monocular_pose_estimator#disclaimer-and-license) on the github page to "please contact the authors for a commercial license", which is in direct contradiction to GPL v3.
- Licensing for **own code** is to be defined.

## Build Instructions
The project is set up to be compiled by MSVC compiler (`cl.exe`) from within Visual Studio Code.
1. Launch the 'x64 Native Tools Command Prompt' (for 64bit) or the 'x86 Native Tools Command Prompt' (for 32bit). It's critical to select the architecture that is compatible with the python interpreter, otherwise the DLL loading will fail.
2. Launch Visual Studio Code with `code .`
2. Use the keyboard shortcut \[Ctrl-Shift-B\] to build, without launching or starting the debugger (which would fail anyway in this case)

