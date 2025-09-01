#!/usr/bin/env bash
set -e
if [ $# -lt 1 ]; then echo "Usage: ./run.sh <DRONE_ID>"; exit 1; fi
export DRONE_ID="$1"

HOST_WS_PATH="$HOME/Documents/uav-formations-fyp-2024"
export HOST_WS_PATH

ENV_FILE=".env_drone_${DRONE_ID}"
{
  echo "HOST_WS_PATH=${HOST_WS_PATH}"
  echo "DRONE_ID=${DRONE_ID}"
  echo "DISPLAY=${DISPLAY:-:0}"
} > "${ENV_FILE}"

export COMPOSE_PROJECT_NAME="drone_project_${DRONE_ID}"
docker rm -f "drone_${DRONE_ID}_container" 2>/dev/null || true
docker compose --env-file "${ENV_FILE}" up -d
sleep 2
docker exec -it "drone_${DRONE_ID}_container" bash
