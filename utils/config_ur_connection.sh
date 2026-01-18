#!/bin/bash

# Setup script for Universal Robots network connection
# Creates a static IP connection for communicating with UR robots

CONNECTION_NAME="UR"
IP_ADDRESS="192.168.56.1/24"
ROBOT_IP="192.168.56.101"

echo "=== UR Robot Network Connection Setup ==="
echo ""

# Check if running with sudo/root for system-wide install
if [[ $EUID -ne 0 ]]; then
    echo "Note: Running without root. Connection will be created for current user."
    echo "      Run with sudo for system-wide installation."
    echo ""
fi

# Check if connection already exists
if nmcli connection show "$CONNECTION_NAME" &>/dev/null; then
    echo "Connection '$CONNECTION_NAME' already exists."
    read -p "Do you want to delete and recreate it? [y/N]: " response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "Removing existing connection..."
        nmcli connection delete "$CONNECTION_NAME"
    else
        echo "Keeping existing connection. Exiting."
        exit 0
    fi
fi

# Create the connection
echo "Creating connection '$CONNECTION_NAME'..."
nmcli connection add \
    con-name "$CONNECTION_NAME" \
    type ethernet \
    ipv4.method manual \
    ipv4.addresses "$IP_ADDRESS" \
    ipv6.method disabled \
    connection.autoconnect no

if [[ $? -eq 0 ]]; then
    echo ""
    echo "✓ Connection '$CONNECTION_NAME' created successfully!"
    echo ""
    echo "Configuration:"
    echo "  - IP Address: $IP_ADDRESS"
    echo "  - Robot IP:   $ROBOT_IP (expected)"
    echo "  - Autoconnect: disabled"
    echo ""
    echo "Usage:"
    echo "  Connect:    nmcli connection up $CONNECTION_NAME"
    echo "  Disconnect: nmcli connection down $CONNECTION_NAME"
    echo "  Test:       ping $ROBOT_IP"
    echo ""
    
    # Ask if user wants to bring up the connection now
    read -p "Bring up the connection now? [y/N]: " activate
    if [[ "$activate" =~ ^[Yy]$ ]]; then
        echo "Activating connection..."
        nmcli connection up "$CONNECTION_NAME"
        if [[ $? -eq 0 ]]; then
            echo ""
            echo "✓ Connection active. Testing connectivity..."
            echo "Pinging $ROBOT_IP (3 attempts)..."
            ping -c 3 "$ROBOT_IP"
        else
            echo "✗ Failed to activate connection."
            echo "  Make sure Ethernet cable is connected."
        fi
    fi
else
    echo "✗ Failed to create connection."
    exit 1
fi
