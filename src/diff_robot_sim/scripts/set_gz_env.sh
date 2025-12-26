#!/usr/bin/env bash
# Helper to set Gazebo/Ogre paths when using the mamba env on macOS.
# Safe to source from bash or zsh.

# Do not exit the parent shell on minor issues.
set +e

# Resolve script directory in both bash and zsh
_SRC="${BASH_SOURCE[0]:-${(%):-%N}}"
SCRIPT_DIR="$(cd "$(dirname "$_SRC")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONDA_PREFIX=${CONDA_PREFIX:-"/Users/akko/.local/share/mamba/envs/ros_jazzy_full"}

# Only set resource paths to find local models/worlds; leave OGRE defaults alone.
export GZ_SIM_RESOURCE_PATH="${ROOT_DIR}/models:${ROOT_DIR}/worlds:${GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"

echo "Set Gazebo resource paths for conda env:"
echo "  GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
