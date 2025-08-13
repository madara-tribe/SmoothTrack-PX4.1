#!/bin/bash

set -e

# Step 1: Install psmisc if not installed
if ! command -v fuser &> /dev/null; then
  echo "[INFO] Installing psmisc (for fuser)..."
  sudo apt update
  sudo apt install -y psmisc
fi

# Step 2: Check if /dev/video4 is being used
PIDS=$(fuser /dev/video4 2>/dev/null)

if [ -z "$PIDS" ]; then
  echo "[INFO] /dev/video4 is free."
else
  echo "[INFO] /dev/video4 is used by: $PIDS"
  for PID in $PIDS; do
    echo "[INFO] Killing PID $PID..."
    sudo kill -9 "$PID"
  done
  echo "[INFO] All processes using /dev/video4 have been terminated."
fi

