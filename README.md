# Quadcopter Exploration

This project is based on the dynamic model and controller designed in [this project](https://github.com/bobzwik/Quadcopter_SimCon), which uses PyDy to derive the equations of motion.

## Run the model locally

Use CPython 3.10, 3.11 or 3.12. On Ubuntu/Debian install the matching venv package (for the distribution Python, `sudo apt install python3-venv`). On Linux or macOS, run these commands from the repository root.

```bash
./install_local.sh
./run_local.sh --scenario room --seconds 8 --output build/room.csv --plot build/room.png
./run_local.sh --scenario tunnel --seconds 8 --output build/tunnel.csv
```

On a desktop, add `--view` to either command for an animated 3D replay. The tunnel view follows the quad and draws nearby voxels. Close the window to return to the shell. Matplotlib needs an interactive backend, such as Tk (`sudo apt install python3-tk` on Ubuntu/Debian). The CSV and `--plot` also work over SSH without a display.

The installer creates `.venv` and installs pinned packages. The runner uses that environment and works offline. Set `QUAD_PYTHON=python3.12` before installation to select an interpreter. Output paths are relative to your current directory, so the launcher also works by absolute path from elsewhere. The PNG shows the horizontal track and altitude. Both examples run on a CPU without a display or Isaac Sim.

On Windows PowerShell, use the Python entry point directly.

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe Simulation\run_local.py --scenario room --seconds 8 --plot build/room.png
```

| Scenario | What to expect |
| --- | --- |
| `room` | The controller climbs to 0.9 m, moves 1.2 m east, then 1.2 m north. At 8 s the position is approximately (1.20, 1.19, 0.90) m. |
| `tunnel` | The original waypoint-following controller and artificial potential field use the bundled voxel cloud. Run for 95 s to reproduce the original simulation duration. The model starts at the tunnel centre height. |

Each model tick is 0.005 s. The CSV includes time zero, every fourth tick, and the final tick. Columns are `time_s,east_m,north_m,up_m,qw,qx,qy,qz`. The unit quaternion rotates a forward-left-up body vector into the east-north-up world. The exporter handles the native frame selected in `Simulation/config.py`. Height is relative to the model origin. The original physical constants remain in `Simulation/quadFiles/initQuad.py`.

The launcher integrates the controller and dynamics on every run. `--plot` saves a still image. Choose `--view` for the Matplotlib 3D window, or follow the VisPy commands below for the OpenGL tunnel.

For an in-process adapter, put this repository root on the Python import path, use `from Simulation.flight import Flight`, construct `Flight("room")`, call `step()` once per model tick, then read `pose()` for a position tuple and a scalar-first quaternion tuple. Construct a new `Flight` to reset. The numerical integration, controller and wind model are shared with `Simulation/run_3D_simulation.py`. Its historical VisPy desktop animation has additional OpenGL and GUI dependencies described below.

This project has as a goal to simulated a quadcopter/quadrotor exploring unknown environments (this is nothing groundbreaking, I'm doing this on my spare time as a hobby and to learn Python).

A pointcloud environment can be generated, where every point of the pointcloud can be considered as an obstacle. Currently, an Artificial Potential Field algorithm is used to navigate through the pointcloud ( each point exerts a force on the drone in order to stop it from colliding with it). More exploration algorithms are to come.

[![Watch the video](http://img.youtube.com/vi/WuDDGpTPt2g/0.jpg)](https://youtu.be/WuDDGpTPt2g)


## Pointcloud Generation
There are a few available scripts in this branch to generate simple tunnels. They take a while to execute, since a first create a very fine pointcloud, in order to make sure that once voxelized, there are no gaps between points. By voxelized, I mean that I create a corse Cartesian grid (0.25 m between "voxel" centers) and every point of the fine pointcloud is approximated to its closest "voxel".

## Artificial Potential Field Implementation
In this Artificial Potential Field, there is no **explicit** attractive force, as the flight control already commands a velocity, given a desired position. This velocity command is saturated. The velocity control then computes a desired thrust/force, which in turn indicates the desired quadrotor attitude (orientation) ([See 'master' readme and PX4 controller](https://github.com/bobzwik/Quadcopter_SimCon/tree/master))

The repulsive force is calculated using a widely used algorithm (see **Siegwart's Introduction to Autonomous Mobile Robots**). Every point of the voxelized pointcloud is considered an object. This force is then implemented into the controller through 3 different possible ways.

1. Added to the saturated "attractive" velocity. The resulting desired velocity is then saturated again (to prevent large velocity commands).

2. Added to the desired thrust/force, which is then saturated (to prevent large force commands). The saturation is defined using a specified maximum tilt of the quadrotor.

3.  Added to the desired thrust/force after its saturation.

Currently, method # 1 is used, and the repulsive gain is set accordingly.


### Useful links
* [Youtube video on Artificial Potential Field](https://youtu.be/8Vva0bnMIEI)
* [Slides on Artificial Potential Field](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)
* [More slides](https://www.dis.uniroma1.it/~oriolo/amr/slides/MotionPlanning3_Slides.pdf)
* A pdf of Siegwart's book can easily be found online ;)


## Visualization Using VisPy

Install the optional OpenGL viewer once from the fork root. Run the following on the computer with your display.

```bash
./install_local.sh --vispy
./run_local.sh --vispy --seconds 20 --no-plots
```

The runner first integrates the tunnel flight, reports progress, then opens an interactive VisPy window and plays the recorded poses in real time. Drag with the left mouse button to orbit the drone and scroll to zoom. Yellow cubes show obstacles, red cubes mark those inside the potential field, green points mark waypoints, red/blue arms show the drone, and a line traces its path. The camera follows the drone. The final pose stays on screen until you press **Esc** or close the window.

```bash
# Use point markers to draw the same tunnel
./run_local.sh --vispy --seconds 20 --renderer markers --no-plots
# Original 95-second flight, with diagnostic figures after you close the window
./run_local.sh --vispy
# Direct Python entry point, also supported
.venv/bin/python Simulation/run_3D_simulation.py --seconds 20 --renderer boxes --no-plots
```

On Windows, after creating the virtual environment above, run:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-vispy.txt
.\.venv\Scripts\python.exe Simulation\run_3D_simulation.py --seconds 20 --no-plots
```

Use `--renderer boxes` for cubes with perspective depth, or `--renderer markers` for lighter fixed-size points. Omit `--no-plots` to inspect the original Matplotlib diagnostics after playback. This entry always uses the original tunnel. To see the room, use `./run_local.sh --scenario room --seconds 8 --view`. The VisPy viewer requires a desktop session and an OpenGL driver usable by GLFW. [VisPy installation](https://vispy.org/installation.html)

## To-Do
* Implement "Extended Potential Field" to reduce oscillations
* Implement "Frontier Based Exploration" (so learn about ray-casting and possibly octomaps)
* Implement some king of "Path Planning"
* Use the `multiprocessing` Python package to parallelize different tasks (a process for the quadcopter dynamics, one for the control, one for the "Frontier Based Exploration" and one for the visualization)
