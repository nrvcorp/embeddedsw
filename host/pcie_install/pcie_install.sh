#!/bin/bash

# Initialize password variable
PASSWORD=""

# Function to print usage information
usage() {
    echo "Usage: $0 --password <password>"
    exit 1
}

# Parse the command-line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --password)
            PASSWORD="$2"
            shift 2
            ;;
        *)
            usage
            ;;
    esac
done

# Check if password is provided
if [ -z "$PASSWORD" ]; then
    usage
fi

# Get the current directory
CURRENT_DIR=$(pwd)

# Display the information
echo "Password: $PASSWORD"
echo "Current Directory: $CURRENT_DIR"

# Modify run_root.sh
RUN_ROOT="$CURRENT_DIR/run_root.sh"
if [ -f "$RUN_ROOT" ]; then
    NEW_STRING="ROOT_PWD=\"$PASSWORD\""
    sed -i "1s|.*|$NEW_STRING|" "$RUN_ROOT"
    NEW_STRING="sudo sh $CURRENT_DIR/start_pcie.sh"
    sed -i "3s|.*|$NEW_STRING|" "$RUN_ROOT"
else
    echo "Error: $RUN_ROOT not found!"
    exit 1
fi

# Modify reboot_run_root.service
SERVICE_FILE="$CURRENT_DIR/reboot_run_root.service"
if [ -f "$SERVICE_FILE" ]; then
    NEW_STRING="ExecStart=/bin/bash $CURRENT_DIR/run_root.sh"
    sed -i "5s|.*|$NEW_STRING|" "$SERVICE_FILE"
else
    echo "Error: $SERVICE_FILE not found!"
    exit 1
fi

# Print success message
echo "Password '$PASSWORD' processed in directory: $CURRENT_DIR"



# Define the service file name
SERVICE_FILE="reboot_run_root.service"

# Check if the service file exists in the current directory
if [ ! -f "$SERVICE_FILE" ]; then
    echo "Error: $SERVICE_FILE not found in the current directory."
    exit 1
fi

# Define the target directory and file path
TARGET_DIR="/etc/systemd/system"
TARGET_FILE="$TARGET_DIR/$SERVICE_FILE"

# Delete any existing service with the same name
if systemctl list-units --full --all | grep -q "$SERVICE_FILE"; then
    echo "Deleting existing service: $SERVICE_FILE"
    sudo systemctl stop "$SERVICE_FILE"    # Stop the service if it's running
    sudo systemctl disable "$SERVICE_FILE" # Disable the service
    sudo rm "$TARGET_FILE"                  # Remove the existing service file
else
    echo "No existing service named $SERVICE_FILE found."
fi

# Copy the service file to /etc/systemd/system
echo "Copying $SERVICE_FILE to $TARGET_DIR"
sudo cp "$SERVICE_FILE" "$TARGET_DIR"

# Change the permissions of the service file
echo "Setting permissions for $TARGET_FILE"
sudo chmod 755 "$TARGET_FILE"

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
sudo systemctl status "$SERVICE_FILE"

echo "PCIE install finished"