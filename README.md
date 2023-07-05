# SOLO12 SDK
Interface framework for sending commands between the SOLO12 robot and a computer. The interface startup states are configured as, 
<p align="center">
Sweeping -> Hold -> Track
</p>
In the sweeping state, the robot calibrates itself by finding the index encoding positions on each actuator. Once the index position are found, the robot is placed in the zero position for each motor reading. Now the robot is in run mode, the run states are
<p align="center">
Hold -> Track -> Hold -> Track -> ...
</p>

Most of the hardware configuration is located in the `SOLO12_SDK/include/config.hpp` header file, 


#  Building
## 
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
<img src=./data/assets/solo12_startup_config.png width="250" height="300">
</p>



# Execution
Now there is a test trajectory in `/data/active/gait.csv`

1. Navigate to `/build/bin` directory: `cd build/bin`
2. Run main executable: `./main`
3. Start the main script and wait for the sweep sequence to finish.
4. Press `enter` and the robot will be in the hold state.
5. Press `enter` again and the robot will be tracking the test trajectory found in `/data/active/gait.csv`.