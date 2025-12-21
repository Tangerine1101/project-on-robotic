#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="ros2_yolo_kazs"
IMAGE="ros2-yolo11:kazs"
HOST_PROJECT_DIR="$HOME/Documents/PlatformIO/Projects/project_on_robotic"
CONTAINER_PROJECT_DIR="/"

# If container exists (any state)
if docker ps -a --format '{{.Names}}' | grep -wq "^${CONTAINER_NAME}$"; then
  # If it's already running, open a shell
  if docker ps --format '{{.Names}}' | grep -wq "^${CONTAINER_NAME}$"; then
    echo "Container '${CONTAINER_NAME}' is already running — opening a shell..."
    docker exec -it "${CONTAINER_NAME}" bash 2>/dev/null || docker exec -it "${CONTAINER_NAME}" sh || docker attach "${CONTAINER_NAME}"
    exit 0
  fi

  # If it exists but is stopped, start and attach
  echo "Starting existing container '${CONTAINER_NAME}'..."
  docker start "${CONTAINER_NAME}"
  # Try to open a shell inside the started container
  docker exec -it "${CONTAINER_NAME}" bash 2>/dev/null || docker exec -it "${CONTAINER_NAME}" sh || docker attach "${CONTAINER_NAME}"
  exit 0
fi

# Otherwise create and run a new container
echo "Creating and running container '${CONTAINER_NAME}' from image '${IMAGE}'..."
docker run -it \
  --gpus all \
  --runtime=nvidia \
  --name "${CONTAINER_NAME}" \
  --restart unless-stopped \
  -v "${HOST_PROJECT_DIR}:${HOST_PROJECT_DIR}" \
  "${IMAGE}"