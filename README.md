- [1. About `SOLO12_SDK`](#1-about-solo12sdk)
	- [Features Overview](#features-overview)
- [2. Installation](#2-installation)
- [3. Usage](#3-usage)
	- [3.1. Physical Configuration](#31-physical-configuration)
	- [3.2. Quick Start:](#32-quick-start)
	- [3.3. User Interface:](#33-user-interface)
		- [3.3.1. States](#331-states)
		- [3.3.2. Hard Calibration (`-c` or `--calibrate`)](#332-hard-calibration--c-or---calibrate)
		- [3.3.3. Sweeping (Soft) Calibration](#333-sweeping-soft-calibration)
		- [3.3.4. Trajectory Tracking](#334-trajectory-tracking)

# 1. About `SOLO12_SDK`
`SOLO12_SDK` is a C++ interface framework for the [SOLO 12 Advanced Quadruped Platform](https://solo.pal-robotics.com/solo) developed by [PAL Robotics](https://pal-robotics.com/). This framework uses [Masterboard SDK](https://github.com/open-dynamic-robot-initiative/master-board/tree/master/sdk/master_board_sdk) developed by the [Open Dynamic Robot Initiative](https://github.com/open-dynamic-robot-initiative), which provides a C++ interface to send and receive commands from the masterboard via Ethernet or WiFi.

## Features Overview
- Estabilish connection with the masterboard via Ethernet and initialize the masterboard.
- Calibrate the encoder zero positions if necessary (i.e. after a power cycle.), then track the encoder zero pose for the visual verification of the calibration.
- Track stance pose (the initial position) in order to place to robot on the ground.
- Track desired joint trajectories that are read from file via the onboard PD controller.
- Save the measured realized joint trajectories to file.



# 2. Installation

You will need build essentials, cmake and eigen3 to build:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libeigen3-dev
```
You may use the included scripts or the VS Code tasks to configure and build:

1. Clear the existing build (if necessary): `scripts/build/clear.sh`
2. Configure cmake file: `scripts/build/configure.sh`
3. Build the project: `scripts/build/build.sh` 
4. Set executable permissions if not already done (may require elevated priviliges): `set_permissions.sh`

# 3. Usage

## 3.1. Physical Configuration
1. Make sure the power is off.
	1. If you have NOT hard calibrated before: Position each leg exactly in the encoder zero pose using the calibration braces.
	2. If you have hard calibrated before: Position each leg **approximately** in the encoder zero pose (i.e. vertical with respect to the base, knees extended).
2. Power on the robot and launch the `build/bin/main` executable.
<p align="center">
<img src=./data/assets/IMG_5731.png width="403" height="302">
</p>

## 3.2. Quick Start:

1. Navigate to `/build` directory and run the main executable: `./bin/main`
2. Start the main script and wait for the sweep sequence to finish.
3. Press `enter` and the robot will be in the hold state.
4. Press `enter` again and the robot will be tracking the test trajectory found in `/data/active/gait.csv`.

## 3.3. User Interface:

The `SOLO12_SDK` executable takes command line arguments and keyboard inputs. The available command line arguments can be viewed using the `-h` or `--help` flag.
 <p align="center">
<img src=./data/assets/flags.png width="576" height="231">
</p>

You will need the interface name for the SOLO12 robot. You can view this information using `ifconfig` in your terminal.

The user interface prints important information in real time to the terminal. The printed information options can be changed in `include/commander/config.hpp`.
 <p align="center">
<img src=./data/assets/ui_example.png width="696" height="232">
</p>

### 3.3.1. States
The interface commands three robot states: 
1. Sweep: Sweeps the legs until the index pulse positions are found (no user input is needed, however the robot needs to be freely move its joints).
2. Hold: Tracks the initial pose of the robot (this can be changed in `include/commander/config.hpp`).
3. Track: Tracks the trajectory provided in the file `/data/active/gait.csv` (this can be changed in `include/commander/config.hpp`).

The robot will follow the following state sequence after power up:
<p align="center">
Sweeping -> Hold -> Track -> Hold -> Track -> ...
</p>

If the index pulse positions are still on memory, sweeping will be skipped.

### 3.3.2. Hard Calibration (`-c` or `--calibrate`)
Hard calibration is required after hardware assembly to determine the index pulse locations with respect to the encoder zero positions. You will need to save the displayed index offset values to  `include/commander/config.hpp`.

### 3.3.3. Sweeping (Soft) Calibration
This calibration is done after every power up. The robot will go into sweeping calibration mode if it detects the index positions are not found. During this step, no user input is required, however the robot must be able to move its joints freely. The robot will sweep the legs until the index positions are found. Once all the index positions are found, the robot will go into the zero position thanks to the index pulse locations saved from the hard calibration step (needs to be done only once after hardware assembly). The robot will wait for the user to press any key to continue.

Because of the transmission ratio of 9 between the motors and the joints, the index pulse occurs every $2\pi/9$ radians or approximately $40$ degrees. This means index pulse locations are often incorrect by this amount. You may perform a power cycle, or you can use the `-o` or `--offset-index` flag. For details, see `-h` or `--help` flag.

### 3.3.4. Trajectory Tracking
By default, the robot will track the trajectory provided in the file `/data/active/gait.csv` (this can be changed in `include/commander/config.hpp`). The trajectory file will be read during initialization and then after every time the trajectory is completed.

Once the trajectory is complete, the measured joint trajectories will be saved to `/build/data/track_data/realized_control_data.csv` (this can be changed in `include/commander/config.hpp`).
