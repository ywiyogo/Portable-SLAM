#!/bin/bash

# Function to check if user is in group
is_user_in_group() {
    local user="$1"
    local group="$2"
    groups "$user" | grep -q "\b$group\b"
}

# Current user
USER=$(whoami)

# Check if user is in i2c group
if is_user_in_group "$USER" "i2c"; then
    echo "$USER is already in the i2c group. Please continue."
else
    echo "$USER is not in the i2c group. Adding now..."
    sudo usermod -aG i2c "$USER"
    echo "User added to i2c group. Applying group changes..."
    newgrp i2c
    echo "Group changes applied."
fi
