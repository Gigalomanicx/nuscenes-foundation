#!/bin/bash

set -e

# Resolve script directory and repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="rosbag2nuscenes"

USER_ID=$(id -u)
GROUP_ID=$(id -g)

# ====== Configure these paths before running ======
ROSBAG_DIR="${ROSBAG_DIR:-/path/to/rosbag2_directory}"
PARAM_FILE="${PARAM_FILE:-${REPO_ROOT}/params/bjut.yaml}"
OUTPUT_SUBDIR="${OUTPUT_SUBDIR:-output}"
NUM_WORKERS="${NUM_WORKERS:-4}"
# ==================================================

OUTPUT_DIR="${OUTPUT_DIR:-$(dirname "${ROSBAG_DIR}")/${OUTPUT_SUBDIR}}"
CONTAINER_OUTPUT="/output"

echo "=========================================="
echo "1/2 Building image: ${IMAGE_NAME}"
echo "   Context: ${REPO_ROOT}"
echo "=========================================="
docker build -f "${SCRIPT_DIR}/Dockerfile" -t "${IMAGE_NAME}" "${REPO_ROOT}"

echo ""
echo "=========================================="
echo "2/2 Running conversion"
echo "  ROSBAG_DIR:  ${ROSBAG_DIR}"
echo "  PARAM_FILE:  ${PARAM_FILE}"
echo "  OUTPUT_DIR:  ${OUTPUT_DIR}"
echo "  NUM_WORKERS: ${NUM_WORKERS}"
echo "=========================================="

mkdir -p "${OUTPUT_DIR}"
chmod u+rwx "${OUTPUT_DIR}"

docker run --rm \
    --user "${USER_ID}:${GROUP_ID}" \
    --mount type=bind,src="${ROSBAG_DIR}",target=/data/rosbag \
    --mount type=bind,src="${PARAM_FILE}",target=/params/config.yaml \
    --mount type=bind,src="${OUTPUT_DIR}",target="${CONTAINER_OUTPUT}" \
    "${IMAGE_NAME}" \
    /data/rosbag \
    /params/config.yaml \
    "${CONTAINER_OUTPUT}" \
    "${NUM_WORKERS}" \
    2>&1

echo ""
echo "=========================================="
echo "Done. Output saved to: ${OUTPUT_DIR}"
echo "=========================================="
