#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t drone:humble .
echo "Image drone:humble construite (amd64)."
