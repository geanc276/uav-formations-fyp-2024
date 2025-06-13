#!/usr/bin/env bash

set -e  # Exit immediately if a command exits with a non-zero status
set -u  # Treat unset variables as an error
set -o pipefail  # Prevent errors in a pipeline from being masked

# Function to display usage information
usage() {
    echo "Usage: $0 <DRONE_ID>"
    echo "  <DRONE_ID> : Unique identifier for the drone (e.g., 1, 2, A, B)"
    exit 1
}

# Check if DRONE_ID is provided
if [ $# -lt 1 ]; then
    echo "Error: DRONE_ID not provided."
    usage
fi

DRONE_ID="$1"
export DRONE_ID
echo "Starting setup for Drone ID: $DRONE_ID"

# Docker image details
IMAGE_NAME="drone"
IMAGE_TAG="humble"  # Example tag; adjust as needed

# Detect the host workspace path more robustly
HOST_WS_PATH=$(pwd | sed -n 's|\(.*uav-formations-fyp-2024[^/]*\).*|\1|p')

if [ -z "$HOST_WS_PATH" ]; then
    echo "Error: Unable to determine HOST_WS_PATH. Ensure the current directory is within 'uav-formations-fyp-2024'."
    exit 1
fi

CONTAINER_WS_PATH="/home/ros/drones/uav-formations-fyp-2024"
echo "HOST_WS_PATH: $HOST_WS_PATH"
ls "$HOST_WS_PATH"

# Define unique container name prefix
CONTAINER_NAME_PREFIX="drone_${DRONE_ID}"
export CONTAINER_NAME_PREFIX

# Create a unique .env file for each drone instance
ENV_FILE=".env_drone_${DRONE_ID}"
if [ ! -f "$ENV_FILE" ]; then
    cat <<EOF > "$ENV_FILE"
HOST_WS_PATH=$HOST_WS_PATH
CONTAINER_WS_PATH=$CONTAINER_WS_PATH
DRONE_ID=$DRONE_ID
EOF
    echo "Created $ENV_FILE with HOST_WS_PATH, CONTAINER_WS_PATH, and DRONE_ID."
else
    echo "$ENV_FILE already exists. Using existing environment configuration."
fi

# Define a unique Docker Compose project name to avoid conflicts
DOCKER_COMPOSE_PROJECT="drone_project_${DRONE_ID}"
export DOCKER_COMPOSE_PROJECT

echo "Using Docker Compose project name: $DOCKER_COMPOSE_PROJECT"

# Check if a container with the same name is already running and remove it
EXISTING_CONTAINER=$(docker ps -a --filter "name=${CONTAINER_NAME_PREFIX}_container" --format "{{.Names}}")
if [ -n "$EXISTING_CONTAINER" ]; then
    echo "Found existing container: $EXISTING_CONTAINER. Removing it..."
    docker container kill "$EXISTING_CONTAINER"
    docker container rm "$EXISTING_CONTAINER"
    echo "Removed existing container: $EXISTING_CONTAINER."
else
    echo "No existing container found for ${CONTAINER_NAME_PREFIX}_container."
fi

# Start the Docker Compose services using the specific .env file and project name
echo "Starting Docker Compose for Drone ID: $DRONE_ID..."
docker compose --env-file "$ENV_FILE" -p "$DOCKER_COMPOSE_PROJECT" up -d

# Wait for the container to initialize
sleep 5

# Execute a bash shell inside the specific container
TARGET_CONTAINER="${CONTAINER_NAME_PREFIX}_container"
echo "Attempting to exec into container: $TARGET_CONTAINER"
docker exec -it "$TARGET_CONTAINER" bash
