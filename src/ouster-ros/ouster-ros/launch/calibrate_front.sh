#!/bin/bash
# Helper script to run camera calibration for the front camera on the host laptop.

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Configure CycloneDDS network binding
export ROS_DOMAIN_ID=27
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enx4cea416cc906</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

SQUARE_SIZE=${1:-0.03} # Default checkerboard square size is 3cm (0.03m)

echo "--------------------------------------------------------"
echo "Starting local image decompression for front camera..."
echo "--------------------------------------------------------"
ros2 run image_transport republish compressed raw \
  --ros-args \
  --remap in/compressed:=/camera/front/front_camera/image_raw/compressed \
  --remap out:=/camera/front/front_camera/image_raw_local &
DECOMPRESS_PID=$!

# Wait for decompressor to start
sleep 2

echo "--------------------------------------------------------"
echo "Launching camera calibration GUI..."
echo "Using calibration target size: 8x6 corners (9x7 squares)"
echo "Using square size: $SQUARE_SIZE meters"
echo "--------------------------------------------------------"
echo "TIPS FOR GOOD CALIBRATION:"
echo "1. Hold the checkerboard in front of the front camera."
echo "2. Move it to the left, right, top, bottom, and tilt it (skew)."
echo "3. Bring it closer (fill the image) and move it further away."
echo "4. Once the X, Y, Size, and Skew bars on the GUI turn Green,"
echo "   click the 'Calibrate' button (this might freeze the GUI for a minute)."
echo "5. Click 'Save' to dump the results to a zip file in /tmp/calibrationdata.tar.gz"
echo "6. Click 'Commit' to send the calibration directly to the camera node."
echo "--------------------------------------------------------"

ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square $SQUARE_SIZE \
  image:=/camera/front/front_camera/image_raw_local

# Clean up the background decompressor when the GUI is closed
kill $DECOMPRESS_PID
echo "Cleanup complete."
