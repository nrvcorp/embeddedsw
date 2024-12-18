#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Access the parent directory of the script
PARENT_DIR=$(dirname "$SCRIPT_DIR")

# Navigate to xdma_dvs and xdma_zcu106, and make the necessary changes
cd "$PARENT_DIR/xdma_dvs" || { echo "xdma_dvs directory not found"; exit 1; }
make clean
make install

cd "$PARENT_DIR/xdma_zcu106" || { echo "xdma_zcu106 directory not found"; exit 1; }
make clean
make install

echo "loading driver"
bash "$PARENT_DIR/tests/load_driver_camera_npu_modified.sh" || { echo "Failed to load driver"; exit 1; }

# Launch code editor if necessary
# code --no-sandbox --user-data-dir /home/cappfpga01/src/1.NRV/vscode-server