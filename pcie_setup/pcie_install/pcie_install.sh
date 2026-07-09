#!/bin/bash
#
# Installs reboot_run_root.service so that run_root.sh runs at boot.
# The service runs as root, so NO password is required or accepted here.
# (The old --password flag was removed: passing secrets on the command line
#  leaks them into shell history and the process list, and writing them into
#  run_root.sh committed the root password to git. Never do that.)

set -euo pipefail

# Get the directory this script lives in.
CURRENT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Current Directory: $CURRENT_DIR"

# Generate run_root.sh locally (it is intentionally NOT tracked in git so no
# machine-specific content can ever be committed). It runs as root via the
# systemd service, so it needs no password and no sudo.
RUN_ROOT="$CURRENT_DIR/run_root.sh"
cat > "$RUN_ROOT" <<'EOF'
#!/bin/bash
#
# Launched at boot by reboot_run_root.service, which already runs as root.
# Because the service runs with root privileges, NO password or `sudo` is
# needed here. Never store credentials in this file.

# Resolve this script's directory so start_pcie.sh is found regardless of CWD.
SCRIPT_DIR="$(dirname "$(realpath "$0")")"

exec /bin/bash "$SCRIPT_DIR/start_pcie.sh"
EOF
chmod 755 "$RUN_ROOT"
echo "Generated $RUN_ROOT"

# Point the service's ExecStart at run_root.sh in this directory.
SERVICE_FILE_SRC="$CURRENT_DIR/reboot_run_root.service"
if [ -f "$SERVICE_FILE_SRC" ]; then
    NEW_STRING="ExecStart=/bin/bash $CURRENT_DIR/run_root.sh"
    sed -i "5s|.*|$NEW_STRING|" "$SERVICE_FILE_SRC"
else
    echo "Error: $SERVICE_FILE_SRC not found!"
    exit 1
fi

# Define the service file name
SERVICE_FILE="reboot_run_root.service"

# Define the target directory and file path
TARGET_DIR="/etc/systemd/system"
TARGET_FILE="$TARGET_DIR/$SERVICE_FILE"

# Delete any existing service with the same name
if systemctl list-units --full --all | grep -q "$SERVICE_FILE"; then
    echo "Deleting existing service: $SERVICE_FILE"
    sudo systemctl stop "$SERVICE_FILE"    # Stop the service if it's running
    sudo systemctl disable "$SERVICE_FILE" # Disable the service
    sudo rm -f "$TARGET_FILE"              # Remove the existing service file
else
    echo "No existing service named $SERVICE_FILE found."
fi

# Copy the service file to /etc/systemd/system
echo "Copying $SERVICE_FILE to $TARGET_DIR"
sudo cp "$SERVICE_FILE_SRC" "$TARGET_FILE"

# Change the permissions of the service file
echo "Setting permissions for $TARGET_FILE"
sudo chmod 644 "$TARGET_FILE"

# Reload the systemd manager configuration
echo "Reloading systemd daemon"
sudo systemctl daemon-reload

# Enable the service to run at boot
echo "Enabling $SERVICE_FILE to run at boot"
sudo systemctl enable "$SERVICE_FILE"

# Start the service
echo "Starting $SERVICE_FILE"
sudo systemctl start "$SERVICE_FILE"

# Check the status of the service
echo "Checking status of $SERVICE_FILE"
sudo systemctl status "$SERVICE_FILE" --no-pager || true

echo "PCIE install finished"
