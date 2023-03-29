#!/bin/bash

#* project's relative path with respect to this script
PROJECT_PATH=".."

#* Hide popd and pushd stdout by defining new commands.
popdq () {
	command popd "$@" > /dev/null
}
pushdq () {
	command pushd "$@" > /dev/null
}
#* Change the cwd to the script dir temporarily until the script exits for any reason.
#* (If it exists use BASH_SOURCE, otherwise fall back to $0.)
trap popdq EXIT
pushdq "$(dirname ${BASH_SOURCE[0]:-$0})"

sudo setcap cap_net_admin,cap_net_raw+ep $PROJECT_PATH/build/bin/example
sudo setcap cap_net_admin,cap_net_raw+ep $PROJECT_PATH/build/bin/example_imu_data_collection
sudo setcap cap_net_admin,cap_net_raw+ep $PROJECT_PATH/build/bin/example_pd
sudo setcap cap_net_admin,cap_net_raw+ep $PROJECT_PATH/build/bin/main

echo "$0 done."