# About `SOLO12_SDK`
`SOLO12_SDK` is a C++ interface framework for the [SOLO 12 Advanced Quadruped Platform](https://solo.pal-robotics.com/solo) developed by [PAL Robotics](https://pal-robotics.com/). This framework uses [Masterboard SDK](https://github.com/open-dynamic-robot-initiative/master-board/tree/master/sdk/master_board_sdk) developed by the [Open Dynamic Robot Initiative](https://github.com/open-dynamic-robot-initiative), which provides a C++ interface to send and receive commands from the masterboard via Ethernet or WiFi.

## `SOLO12_SDK` Overview:
1. Estabilishes connection with the masterboard via Ethernet and initializes the masterboard.
2. Calibrates the encoder zero positions if necessary (i.e. after a power cycle.) Tracks the encoder zero positions for the visual verification of the calibration.
3. Tracks a specified stance pose (initial position) in order to place to robot on the ground.
4. Continuously reads the desired joint trajectories from file.
5. Tracks the desired joint trajectories using the onboard PD controller.
6. Saves the measured realized joint trajectories to file.

# Installation
## Building

To build the source code, please follow these steps.

1. Navigate to `SOLO12_SDK/scripts/build` directory: `cd /scripts/build`
2. Configure cmake file: `./configure.sh`
3. Build the project (might require super user power): `sudo ./build.sh` 
4. Navigate to home directory: `cd ../../`

# Prerun Configuration
1. Make sure the robot is turned off from the power supply before doing anything.
2. Place your desired trajectory in `SOLO12_SDK/data/active` with the name `gait.csv`. 
3. Position each leg perpendicular to the body frame.
<p align="center">
<img src=./data/assets/IMG_5731.png width="250" height="250">
</p>

# Usage

## `SOLO12_SDK` user interface:

Interface framework for sending commands between the SOLO12 robot and a computer. The interface startup states are configured as, 
<p align="center">
Sweeping -> Hold -> Track
</p>
In the sweeping state, the robot calibrates itself by finding the index encoding positions on each actuator. Once the index position are found, the robot is placed in the zero position for each motor reading. Now the robot is in run mode, the run states are
<p align="center">
Hold -> Track -> Hold -> Track -> ...
</p>

Most of the hardware configuration is located in the `SOLO12_SDK/include/commander/config.hpp` header file, 

# Execution
Now there is a test trajectory in `/data/active/gait.csv`

1. Navigate to `/build/` directory: `cd build/`
2. Run main executable: `./bin/main`
3. Start the main script and wait for the sweep sequence to finish.
4. Press `enter` and the robot will be in the hold state.
5. Press `enter` again and the robot will be tracking the test trajectory found in `/data/active/gait.csv`.