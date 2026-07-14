# The ID of the command to run.
uint8 command

# Optional string argument
string string_arg

# ==============================================================================
# Commands description :

# Enable/Disable the SLAM process
uint8 SWITCH_ON_OFF = 0

# Disable/enable SLAM maps update with new frames keypoints. This can be useful
# if we want to keep the pose graph optimized SLAM map
# or any initially loaded map untouched
uint8 DISABLE_SLAM_MAP_UPDATE = 8
uint8 ENABLE_SLAM_MAP_EXPANSION = 9
uint8 ENABLE_SLAM_MAP_UPDATE = 10

# Reset slam trajectory from a trajectory file
uint8 RESET_TRAJECTORY = 11

# Reset ODOM so current pose is identity
# This allows to minimize numerical instability in odometry
# This command should be used regularly in outside environment (each 200m step for example)
uint8 RESET_ODOM = 13

# Save the current trajectory tracking base frame as csv file
uint8 SAVE_TRAJECTORY = 14
# Save the current trajectory tracking the main Lidar frame as csv file
uint8 SAVE_LIDAR_TRAJECTORY = 15

# Stop the slam and optimize pose graph
# The pose graph optimization is done via g2o, fusing SLAM, GPS and or landmarks detections.
# If logging is enabled, SLAM maps will also
# be updated. This allows to correct SLAM drift and to link more precisely
# SLAM odometry frame to GPS world/map frame.
# WARNING : this process is not real time.
uint8 OPTIMIZE_GRAPH = 20

# Add loop closure indices from an external file
# containing query frame indices and revisited frame indices
# The file should be a csv file with fields queryIdx and revisitedIdx
uint8 LOAD_LOOP_INDICES = 23

# Enable/Disable an external sensor
# If enabled, sensor data are received and stored to then be processed.
# External sensor data can be used in the local optimization or in
# a pose graph optimization in a postprocess manner
# Use 'string_arg' to tell which sensor should be activated
# options are :
#   * WHEEL_ENCODER : "0"
#   * IMU : "1"
#   * LANDMARK_DETECTOR : "2"
#   * GPS : "3"
#   * POSE : "4"
#   * CAMERA : "5"
uint8 SWITCH_SENSOR = 25

# Launch a calibration process using the current SLAM trajectory
# and a file containing an external trajectory captured at the same time
# The file should be a csv file with fields time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2
# xi being the first element of the ith column of the rotation matrix
# and the first line should contain the ID of the frame tracked in the file's trajectory
# It publishes a static TF between the base frame and the
# frame tracked by the external trajectory
# Only an overlapping part of trajectories is necessary to compute
# the calibration
uint8 CALIBRATE_WITH_POSES = 30

# Load poses from an external file when using the
# package in replay (post-process)
# This command takes an argument as a file path to a file.
# The file should be CSV with fields : time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2
# xi being the first element of the ith column of the rotation matrix
# These poses can be used for de-skewing the input points,
# to supply a prior motion to the registration algorithm
# and as a contraint in the SLAM optimization.
# It can also be used to optimize a pose graph.
# The calibration between "ext_pose_sensor" and the
# SLAM tracked pose should be in the TF tree
# It can also be estimated using the calibrate command
uint8 LOAD_POSES = 40
