#!/bin/bash

#* project's relative path with respect to this script
PROJECT_PATH="../.."
CLEAR_DIR="$PROJECT_PATH/build"
#CLEAR_EXES=$PROJECT_PATH/*.exe

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

#* remove dir
if [[ -d "$CLEAR_DIR" ]]; then
rm -r -f $CLEAR_DIR

if [[ $? -eq 0 ]]; then
   echo "Removed $CLEAR_DIR"
fi
fi

#* remove exe
#rm -f $CLEAR_EXES
#if [[ $? -eq 0 ]]; then
#   echo "Removed $CLEAR_EXES"
#fi

echo "$0 done."
