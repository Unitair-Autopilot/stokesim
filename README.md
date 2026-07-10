# stokesim

Lightweight simulator for air/marine-crafts (planes, copters, boats, submarines...) operating simultaneously in a single environment. Each craft can have any configuration of thrusters (motor-driven propellers), control surfaces (servo-driven thin rigid plates), and sensors (IMU, magnetometer, pitot, etc...). A multipoint computational model is used where the craft and its control surfaces are treated as mechanically coupled bodies independently interacting with the surrounding fluid. Fluid interaction is based on incompressible momentum fluxes.

This simulator is focused on utility for roboticists. It captures a large amount of nonlinear phenomena even when constant aero-coefficients are used, so it can be more readily leveraged by modern motion planning and control algorithms. Many vehicles can be simulated simultaneously in real-time (or faster) for swarm algorithm development. This simulator is not meant to capture very fine details like non-rigid structural dynamics or flow-field effects like wake turbulence; it is not meant to be an airframe design tool. It is a GNC and AI development tool with higher fidelity vehicle and environment dynamics than are provided by similar tools.

Check out [the wiki](https://github.com/Unitair-Autopilot/stokesim/wiki/Walk-Through) for more details.

#### Dependencies

- [C++](https://isocpp.org/wiki/faq/cpp11) (11 or 14)
  - Recommended compiler: [GCC](https://gcc.gnu.org/) [`sudo apt install build-essential`]
  - Recommended build-system: [CMake](https://cmake.org/) [`sudo apt install cmake`]
  - [Eigen 3](https://github.com/eigenteam/eigen-git-mirror) [`sudo apt install libeigen3-dev`]
  - [yaml-cpp](https://github.com/jbeder/yaml-cpp) [`sudo apt install libyaml-cpp-dev`]
- [Python](https://www.python.org/) (2.7 or 3.8)
  - [PyYAML](https://pyyaml.org/) [`sudo apt install python-yaml`]
  - [NumPy](http://www.numpy.org/) [`sudo apt install python-numpy`]
  - [matplotlib](https://matplotlib.org/) [`sudo apt install python-matplotlib`]
  - [inputs](https://pypi.org/project/inputs/) [`pip install --user inputs`] (optional)
- [Doxygen](http://www.stack.nl/~dimitri/doxygen/) (>=1.8.13) [`sudo apt install doxygen`]

Note: On Ubuntu >= 20 (or any system using Python3) the above `python-*` packages will be named `python3-*` instead, and you will also want `sudo apt install python-is-python3`.

#### Build

Tested on Ubuntu 16.04, 18.04, and 20.04:
```sh
sudo apt install build-essential cmake
sudo apt install libeigen3-dev libyaml-cpp-dev
sudo apt install python-yaml python-numpy python-matplotlib  # or `python3-*` on Ubuntu >= 20
sudo apt install doxygen
pip install --user inputs  # optional
mkdir -p build
cd build
cmake ..  # toggle any options you need at the top of CMakeLists.txt
make -j
sudo make install  # default copies files to /usr/local/{lib, include, bin}
sudo ldconfig /usr/local/lib  # or wherever you chose to install the library
```

#### Testing Minimally

Run the executable in the stokesim build directory using the "example_minimal" config, and then plot the result using the logger plugin's plotter tool:
```sh
./stokesim ../config/example_minimal.yml
../plugins/logger/plotter logs/coolplane_log.txt
```

#### Testing with ROS

The simulator core of stokesim does not require [ROS](http://www.ros.org/) to operate, but ROS can be a nice way to get the simulator data out to other programs like visualizers. The "stokesros" example plugin does just that. Suppose you have ROS installed and an Xbox controller plugged into a USB port. Do the following in separate terminals (each navigated to the stokesim root directory) to fly some aircraft yourself:
```sh
roscore
```
```sh
rviz -d plugins/stokesros/example_basic.rviz  # this RViz config is tailored for visualizing the example_basic scenario
```
```sh
./build/stokesim config/example_basic.yml  # unlike example_minimal.yml, the example_basic config makes use of stokesros
```
```sh
python plugins/stokesros/interface_node config/example_basic.yml  # this rospy node provides an interface between a joystick and stokesros
```
Then press "start" on the Xbox controller and try flying (right trigger is throttle, you can figure out the rest). Note that there is no autopilot running- good luck!

For viewing satellite overlay in RViz, you'll want to `catkin build` the rviz_satellite ROS package (and don't forget to freshly `source catkin_ws/devel/setup.bash`). The `example_basic_sat.rviz` RViz config in `plugins/stokesros` makes use of URI `http://tile.stamen.com/toner/{z}/{x}/{y}.png` for querying imagery, with a zoom level of 16 (a bad choice of zoom level can result in no image showing up).

#### Disclosure Notice

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
This material is based upon work supported by the Department of the Air Force under Air Force Contract No. FA8702-15-D-0001 or FA8702-25-D-B002. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Department of the Air Force.
© 2024 Massachusetts Institute of Technology.

Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)
The software/firmware is provided to you on an As-Is basis
Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
