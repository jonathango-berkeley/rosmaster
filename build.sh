#!/bin/bash

# Exit immediately if any command fails
set -e

# Print each command (optional, for debugging)
set -x

# Step 1: Pull the latest changes
git pull

# Step 2: Build the workspace
colcon build

# Step 3: Source the build
source install/setup.bash

echo "Update, build, and source complete!"
