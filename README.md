# SOLO12 SDK
Interface framework for sending commands between SOLO12 robot and comptuer
#  Building
## 
To build the source code, please follow these steps.

1. Navigate to `SOLO12_SDK/scripts/build` directory: `cd /scripts/build`
2. Configure cmake file: `./configure.sh`
3. Build the project (might require super user power): `sudo ./build.sh` 
4. Navigate to home directory: `cd ../../`
# Execution
##
Now you can run a test trajectory found in `/data/active/gait.csv`

1. Navigate to `/build/bin` directory: `cd build/bin`
2. Run main executable: `./main`