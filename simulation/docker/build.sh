#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e


echo $PWD
ls $PWD

# Parse options
REBUILD=0
while getopts 'r' opt; do
  case $opt in
    r) REBUILD=1 ;;
    *) echo 'Usage: build.sh [-r] <ros_distro>' >&2
       echo '  -r  Rebuild the Docker image without cache' >&2
       exit 1
  esac
done
shift "$(( OPTIND - 1 ))"

# Check if ROS distribution is provided
if [ $# -eq 0 ] ; then
  echo 'Error: Specify the ROS distribution to use, e.g., "humble".' >&2
  exit 1
fi 

BASE_IMAGE=ros
BASE_TAG=$1
docker pull ${BASE_IMAGE}:${BASE_TAG}

NAME=drone

UserID="$(id -u $USER)"
GID="$(id -g $USER)" 

if [ "$REBUILD" -eq 1 ]; then
  docker buildx build \
    --no-cache \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    --build-arg BASE_TAG=${BASE_TAG} \
    --build-arg UID=${UserID} \
    --build-arg GID=${GID} \
    -t ${NAME}:${BASE_TAG} .
else
  docker buildx build \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    --build-arg BASE_TAG=${BASE_TAG} \
    --build-arg UID=${UserID} \
    --build-arg GID=${GID} \
    -t ${NAME}:${BASE_TAG} .
fi


docker images | grep ${NAME}