#!/bin/bash

#* project's relative path with respect to this script
PROJECT_PATH=".."

echo "Usage: sync_to_shuksan.sh <TargetFolderPath (Example: usr@192.168.1.1:~/project)>"

#* change the cwd to the script dir temporarily, and hide pushd popd output
pushd () { 
	command pushd "$@" > /dev/null 
}
popd () { 
	command popd "$@" > /dev/null 
}
pushd "$(dirname ${BASH_SOURCE:0})"
trap popd EXIT #*

#* cd to project path
pushd $PROJECT_PATH

if [ "$#" -lt 1 ]; then
	echo "No target folder path provided!"
else
	echo "Syncing to $1..."
	rsync -aR \
		./LICENSE \
		./CMakeLists.txt \
		src/ \
		include/ \
		examples/ \
		data/ \
		scripts/ \
		$1/ --delete
fi
popd

echo "Done."
