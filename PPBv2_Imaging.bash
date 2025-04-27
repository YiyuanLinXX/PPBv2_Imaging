#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Run all nodes: GPS logger, GPS publisher, and imaging
# -----------------------------------------------------------------------------

# 1. Workspace and environment
WORKSPACE=~/PPBv2/PPBv2_Imaging
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# 2. Timestamp and output directories
TIMESTAMP=$(date +'%Y_%m_%d_%H_%M_%S')
OUTPUT_DIR="/media/Data/yl3663/PPBv2_data_${TIMESTAMP}"
GPS_LOG="$OUTPUT_DIR/gps_log.csv"

mkdir -p "$OUTPUT_DIR"
echo "Save Directory: $OUTPUT_DIR"

# 3. Common parameters
ARDUINO_PORT="/dev/ttyACM0"
ARDUINO_BAUD=9600
EXPOSURE_TIME=20000.0
GAIN=5.0
WB_RED=1.34
WB_BLUE=2.98

GPS_PORT="/dev/ttyUSB0"
GPS_BAUD=115200

# 4. Launch gps_publisher node
echo "Launching gps_publisher..."
ros2 run multi_camera_trigger gps_publisher \
  --ros-args \
    -p port:="$GPS_PORT" \
    -p baud:=$GPS_BAUD &

# 5. Launch gps_logger node
echo "Launching gps_logger..."
ros2 run multi_camera_trigger gps_logger \
  --ros-args \
    -p log_file:="$GPS_LOG" &

# 6. Launch multi_camera_trigger_node (imaging)
echo "Launching multi_camera_trigger_node..."
ros2 run multi_camera_trigger multi_camera_trigger_node \
  --ros-args \
    -p output_dir:="$OUTPUT_DIR" \
    -p arduino_port:="$ARDUINO_PORT" \
    -p arduino_baud:=$ARDUINO_BAUD \
    -p exposure_time:=$EXPOSURE_TIME \
    -p gain:=$GAIN \
    -p wb_red:=$WB_RED \
    -p wb_blue:=$WB_BLUE &

# 7. Wait for all background processes
wait
echo "All nodes started. Press Ctrl+C to stop."

