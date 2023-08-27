#!/bin/bash
# Hide popd and pushd stdout by defining new commands.
popdq () {
        command popd "$@" > /dev/null
}
pushdq () {
        command pushd "$@" > /dev/null
}
# Change the cwd to the script dir temporarily until the script exits for any reason.
# (If it exists use BASH_SOURCE, otherwise fall back to $0.)
trap popdq EXIT
pushdq "$(dirname ${BASH_SOURCE[0]:-$0})"

# towr's relative path with respect to this script
PROJECT_PATH=../..

mkdir $PROJECT_PATH/build
pushdq $PROJECT_PATH/build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1
echo "$0 returned $?."