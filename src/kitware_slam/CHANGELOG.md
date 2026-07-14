# SLAM changes history

## *v3.1 (2026/06/19)*
v3.1 primarily focuses on enhancing the ParaView wrapper and extending data ingestion workflows. This release introduces multi-LiDAR support, improved external sensor handling, MCAP and ROS integration, and better georeferenced map generation in paraview wrapper. On the SLAM core library side, we added a new pose graph optimization constraint type and a trajectory-distance-based loop closure detector.

### Core lib

**Major new features:**

* Improve support for GNSS-denied environments using external sensor data (!488, !495)
* Add a new trajectory-distance-based loop closure detector (!507)

**Minor new features:**

* Add reliability checks for external sensor data (!508)

**Bug fixes:**

* Fix external pose handling (!480, !530, !535)
* Fix multithreading issues in undistortion when using external poses (!527, !532)

### ROS2 wrapping

**Minor new features:**

* Replace SLAM command topics with ROS services (!478)
* Add support for saving and loading maps for dynamic environments (!487)

### ParaView wrapping

**Major new features:**

* Add multi-LiDAR support (!484, !489, !541, !542)
* Add support for external sensor data input (!504, !505)
* Add support for georeferenced map generation (!529, !531)
* Add a new filter to build keypoint maps from an input trajectory (!526)
* Add interactive pose graph constraints from the interface (!513)

**Minor new features:**

* Improve handling of large-scale map and trajectory coordinates (!486, !503)
* Improve output management and data visualization (!537, !511)

**Fix**

* Fix vtk memory leaks (!494)
* Fix timestamp handling for input data from different sources, including PCAP, MCAP, and ROS messages (!514, !516, !517)

**Compilation / CMake related changes / CI:**

* Make SLAM compativle with ParaView built without Qt (!490)
* Export CMake install for vtk modules (!492, !493)
* Add missing dependencies required by new features (!509)

## *v3.0 (2025/04/01)*

The key update in v3.0 is the transition of the ROS1/ROS2 wrapper. The ROS2 wrapper has now been merged into the master branch. A new protected branch is created for ROS1 wrapper and this branch will no longer be maintained after EOL of ROS Noetic (May, 2025).

This version also introduces several new features and improvements:
- SLAM Toolbar for ParaView Wrapping: A new toolbar with a preset dialog has been added to enhance the UI.
- Mini SLAM Filter in ParaView Wrapping: A lightweight SLAM filter is now available in the ParaView integration.
- Obstacle Extraction in ROS Wrapping: An obstacle extraction feature has been implemented to improve functionality.

Additionally, numerous bug fixes and stability enhancements have been made, making v3.0 more robust and reliable.

The full list of changes is summarized below.

### Core lib

**Minor new features:**

* Allow to apply the offset between external pose and slam odom after PGO (!452)

**Bug fixes:**

* Fix loop closure (!430)
* Add sanity check for Nan value in point cloud (!445)
* Add sanity check for doubled measurements (!468)
* Fix log of estimated calibration matrix (!469)

**Compilation / CMake related changes / CI:**

* Update code for new g2o version (!431)
* Fix CI tests (!432, !433, !475, !476)
* Supply docker files, images and instructions (!435)
* Update to C++ 17 (!461)
* Update superbuild (!466)
* Fix build issue with g2o and eigen (!441)

### ROS2 wrapping

**Major new features:**

* Merge ROS2 wrapper into master branch (!448, !464, !463)
* Use service to save maps instead of message command (!471)
* Allow to set pose and reset odom during slam process (!457)
* Add new feature to extract obstacles in aggregated frame (!428, !436, !439)

**Feature modification**

* Rename launch files (!454)

**Bug fixes:**

* Add sanity check for Nan value in point cloud (!449, !450)
* Fix time factor in converters (!455)
* Fix for multi-threading (!459, !460, !472)
* Fix tf broadcast for external sensor (!453, !472)

**Doc:**

* Update install guide and slam guide (!467)

### ParaView wrapping

**Major new features:**

* Add slam tool bar and preset dialog (!405, !427)
* Add mini SLAM filter (!444)
* Support MCAP format (!451)

**Fix**

* Fix for slam initialization (!425)
* Fix to set manually input array  (!434)
* Fix warning and timer log (!443)
* Fix double frame issue (!442)

## *v2.2 (2024/04/18)*

The main novelty for v2.2 is the availability of the ROS2 wrapper in feat/ROS2 branch.
This release mainly brings available features from the library to the wrappers and improves the wrappers interfaces. Now, in ROS and Paraview, the loop closure is available, the loop closure detection has user supervision, IMU, GPS, external poses and wheel encoder are available and the automatic calibration is updated and fixed.
The connexion to any driver node is improved in ROS.
The PV wrapping is refactored to be able to add user supervision.
In the library, the keypoints extractor is deeply refactored and a new extractor is added to be able to handle dense LiDAR sensors in the future.
This new version also solves a lot of bugs and is more stable.

The changes are summarized below.

### Core lib

**Major new features:**

* Wheel odometer can have a calibration, a reference point and works in live (!323, !320)
* Add a translation constraint for the wheel encoder (!336, !320)
* Add lever arm constraint in calibration with ext trajectory (!362, !365)
* Use sliding window to estimate the calibration matrix (!362, !365)

**Feature modification:**

* Rebuild map with log if the decaying threshold is increased (!416)

**Refactoring and code architecture changes:**

* Refactor the rolling grid (!388, !400)
* Add new keypoint extractor for dense sensors. The output keypoints should be the same but the extractor will be updated in the future. (!324, !399, !398, !402, !403)

**Bug fixes:**

* Add sanity check to clear log (!281)
* Fix loop closure and loop detection (!303, !322, !376, !377, !417, !418)
* Fix wheel encoder constraint (!323)
* Fix initial poses for initial maps (!338, !339, !419, !421)
* Fix PCL transform for empty pc (!341)
* Fix clearpoint infinite loop (!358)
* Fix calibration for GPS (!389, !392)
* Fix empty map bug after clearing (!416)

**Compilation / CMake related changes / CI:**

* Update superbuild (!295)
* Clean docker (!301)
* Remove bad deps from ROS jobs in CI (!301)
* Fix big path build issue in Windows (!293)
* Add git MR template (!335, !346)
* Refact ROS tests (!383)
* Fix frame dropping in ROS2 tests (!361, !380, !381)
* Create ROS2 tests in CI (!357)
* Upgrade dependencies (!390, !391)
* Uniformize cmake messages (!396)
* Build PV wrapper on docker on CI (!411)
* Update CI doc (!413)

### ROS wrapping

**Major new features:**

* Creation of ROS2 wrapping and multiple related fixes (!256, !279, !282, !289, !290, !292, !294, !296, !297, !305, !307, !313, !319, !309, !328, !343, !312, !352, !317, !351, !353)
* Autocompute laser id and time when not provided (!284, !304, !311, !316, !321, !271, !385, !382)
* Add generic converter to use any pointcloud2 as input (!302, !325)
* Add loop closure (!326)
* Integrate wheel encoder (!323, !331, !320, !406)
* Add a slice extractor (!342, !347, !348, !344, !349)
* Integrate IMU (!334, !337)
* Allow to click in RVIZ to notify a loop closure (!360, !356)
* Add Hesai converter (!369, !372, !379)
* Allow to load external poses from CSV file (!374)

**Minor new features:**

* Handle various Velodyne drivers in ROS2 (!298, !299, !308)
* Add reset trajectory command (!333)
* Allow to remove aggregated points around the trajectory (!350, !363)
* Add missing saturation interface for external poses (!366, !367)
* Add maximum distance to sensor param in aggregation node (!368)

**Feature modification**

* If multi Lidars case, never use only one LiDAR frame (!272, !280, !306)
* In multi Lidars case, remove device_id and automatically use frame_id from the pc header (!315, !327, !318)

**Bug fixes:**

* Handle Nan and 0 points in converters (!286, !287)
* Fix broadcast for ROS2 nodes (!314)
* Fix build with PCL and boost (!293)
* Fix time in tests (!359)
* Refact/fix ROS tests (!336, !355, !384, !397, !387)

**Doc:**

* Update install guide (!283)
* Update external sensor doc (!329, !332)
* Fix documentation (!407, !408, !310)

### ParaView wrapping

**Major new features:**

* Provide loop closure indices as CSV file (!364)
* Add autostart plugin (!274)
* Add pop-up with user interaction for loop closure detection (!274)
* Add button that also stops the record (!409)

**Feature modification**

* Update the calibration interface (!362)
* Refactor interface (move parts and change default values) (!378, !414, !417)
* Add transfrom tree section to change current pose or referential (!419)
* Add button to initialize the SLAM process with maps and init poses (!419)

**Fix**

* Fix the number of threads used in kpts extractor (!371)
* Remove subproxy (!412)
* Fix crash if IdentifyInputArrays fails (!420)

## *v2.1 (2023/07/30)*

This release mainly brings 4 new features : loop closure automatic detection, failure detection and recovery mode, calibration estimation and the handling of Livox sensors in ROS. It also contains many fixes relatively to previous version on pose graph optimization and on the build and install cmake process. The link with boost is also fixed. Performances are also improved modifying the keypoints extraction and the submap extraction. The ROS interface is improved.

The changes are summarized below.

### Core lib

**Major new features:**

* Add automatic loop closure detection (!218, !250)
* Allow to upload a list of loop closures (!250)
* Add failure detector and recovery mode (!251, !255)
* Find calibration with external trajectory  (!236, !263, !233)
* Add pose graph optimization with ext poses (!233, !270)
* Add calibration computation for GPS (!260)

**Refactoring and code architecture changes:**

* refactor loop closure parameters (!240)
* refactor tworld and odom frame management (!233)

**Performance improvements:**

* Fix the number of left/right neighbors even when radius is used (!256)
* Sort only partially the kpts values (!256)
* Allow to extract only implied map voxels (!273)

**Bug fixes:**

* Fix bugs in IMU integration (!234)
* Fix pose graph with tags (!235)
* Fix min neighbor radius (!239)
* Fix infinite loop in windows (!247)
* Fix compilation with clang (!252)
* Fix blobs use (!256)
* Fix kpts out of bounds in voxel grid (!256)
* Make keypoints extraction deterministic for debugging purposes (!256)
* Fix UndistortWithLogStates with base to lidar calibration (!275)
* Fix and update GPS use (!260)

**Compilation / CMake related changes / CI:**

* Remove unwanted warnings in find_dependency (!232)
* Clean CMakeLists and cmake.in (!238, !241, !242)
* Fix CI ROS test2 (!246)
* Fix boost linking (!277)

### ROS wrapping

**Major new features:**

* Handle livox sensors (!256)
* Create an option to build the wrapping using an external LidarSlam library (!259)
* Update and link aggregation node to viz plugin (!266)
* Update interface to ouster driver (!265)
* Refact rviz plugin (!266)

**Bug fixes:**

* Fix build issues and update build documentation (!237, !249, !256)

### ParaView wrapping

**Major new features:**

* Adapt to new LidarView version (!254, !253)

## *v2.0 (2023/03/23)*

This release mainly brings 4 new features : loop closure (without detection), IMU raw data integration, RGB camera integration and interpolation model choice. It also contains some refactoring, notably in the interpolation steps. An external superbuild is added as submodule. This new superbuild is cross-platform and was used to update the CI. Note that the poses accessibility from the library and in the wrapping has been improved.

The changes are summarized below.

### Core lib

**Major new features:**

* Add a loop closure (detection is not handled) (!192, !235, !240)
* Add loop closure automatic detection for the current frame using teaserpp (!218)
* Integrate IMU raw data to deskew, do egomotion and add a constraint to SLAM (postprocess only) (!187, !232, !235)
* Allow to get a pose at any required timestamp using external sensors (!216, !219, !221)
* Allow to get poses since last frame at a specific rate (!216, !217, !219)
* Add interpolation models choice (!206, !247)
* Integrate RGB camera data to SLAM optimization (through optical flow constraint) (!180)
* Allow to get base frame from GPS measurements in external sensors API (!235)

**Refactoring and code architecture changes:**

* Add missing const and remove unused functions (!212, !203)
* Remove the MotionModel class and create an interpolator class (!206)
* Remove WithinFrameMotion object (!206)
* Uniformize external sensors (!212)
* refactor AddExternalSensor function in PoseGraphOptimizer (!235)

**Performance improvements:**

* Add keypoints sampling and input frame sampling (!169)

**Bug fixes:**

* Fix behavior of InitTworldWithPoseMeasurment function (!216)
* Fix segfault that was occuring sometimes because of the PoseManager (!222)
* Fix min neighbor radius application (!239)
* Fix GPS vertex indices in pose graph (!240)

**Compilation / CMake related changes / CI:**

* Fix rpath for Linux systems in case of local dependencies (!215)
* Fix g2o conditional compilation (!192)
* Add GTSAM as optional dependency (!187)
* Add OpenCV as optional dependency (!180)
* Fix link with PCL + add PCL variable for compilation (!225)
* Remove Ceres warning of deprecated functions (!227, !237)
* CMakify the library so it can be built, installed and called as a target on cmake (!228, !229, !232, !238, !241, !242)
* Add a cross platform external superbuild with all required dependencies (update the CI to use it) (!226)
* Fix CI ROS test 2 (!246)
* Fix CI on windows (!243)

### ROS wrapping

**Major new features:**

* Allow to provide a pose output frequency (!187)
* Allow to set the interpolation mode (!206)
* Allow to provide RGB camera messages (!180)
* Update build documentation (!237)

**Bug fixes:**

* Fix LidarIsPosix parameter (!180)
* Fix Compilation (!237)

### ParaView wrapping

**Major new features:**

* Allow to load a calibration file for the Lidar (!212)
* Allow to load a trajectory to recompute the maps (!210)
* Add button to rebuild the maps based on the trajectory (!192)
* Add manual loop closure (!192)
* Allow to provide raw IMU data (!187)
* Allow to provide a pose output frequency (!187)
* Add button to optimize the trajectory using IMU pose graph (!187)
* Allow to set the interpolation mode (!206)
* Adapt Cmake architecture to new LidarView version (!238)

**Bug fixes:**

* Fix maps blinking when update step is changed (!205)
* Fix conflict when setting LoggingTimeout (!207)
* Fix offline SLAM filter (!208)

**Refactoring and code architecture changes:**

* Uniformize debug output (!212)

## *v1.6 (2022/10/21)*

This release mainly brings 3 new features : landmarks handling, pose graph optimization (big refactoring from old GPS pose graph optimization), external poses use. It also contains some refactoring, notably in the external sensor structures and in the keypoints extraction. A superbuild is added. The CI is updated with new gitlab version and new dependencies versions.

The changes are summarized below.

### Core lib

**Major new features:**

* Handle landmarks and use them in local SLAM optimization (!150)
* Add pose graph as postprocess (!162, !178)
* Restore GPS use to update a pose graph (!172)
* Use external pose in a tight optimization (!173)
* Add radius to define neighborhood (!193, !200)

**Refactoring and code architecture changes:**

* Important keypoints refactoring and improve interface (!169)
* Important external sensors refactoring (!188, !195, !198)
* Clean rolling grid function (!203)

**Performance improvements:**

* Add keypoints sampling and input frame sampling (!169)

**Bug fixes:**

* Fix rolling maps structure (!171)
* Stop spinner before destruction (!182)
* Fix windows file loading (!182)
* Allow to load a config file for Ouster (!182)

**Compilation / CMake related changes / CI:**

* Fix Ceres compilation (!168)
* Create superbuild (!170, !174, !186)
* Fix CI (!170, !191)

### ROS wrapping

* Add interface for Ouster sensors (!182)
* Add Reset command (!184)
* Add aggregation node with service to aggregate and downsample all frames (!189, !199)
* Add Rviz panel plugin to give an interface for ROS user (!177)
* Clean code (!197)

### ParaView wrapping

## *v1.5 (2021/12/13)*

This release mainly brings new features (keyframes structure, integration of IMU/wheel odometry constraints, moving objects rejection, confidence estimators integration, CI implementation) and contains some refactoring, notably in the map structure and in the keypoint types management.

The changes are summarized below.

### Core lib

**Major new features:**

* Simplify some parameters for model fitting (matches validation) (!98)
* Auto compute angular resolution for SSKE (!102)
* Add invalidation parameter for oblique surfaces (!115)
* Add 2D optimization mode (!118)
* Add only keyframes points to maps (!117)
* Add wheel odometry and IMU constraints to optimization (!114)
* Add an optional overlap estimator (!127 and !131)
* Add motion limitations to detect failures (!129)
* Add a mapping mode + a sampling mode (!147)
* Add an option to reject moving objects (!147)
* Add an option to add temporal keypoints (!147)

**Refactoring and code architecture changes:**

* Change keypoint invalidation criteria to use max oblique plane (!93)
* Gather all keypoint types relative structures and functions (!103)
* Split Matching and optimization (!101)
* Fix refactoring bugs (!108)
* Update README and LidarView tutorial (!113)
* Clean, correct and refactor keypoints matching (!136, !140)
* Replace the pcl voxel grid by a custom one (!147)

**Performance improvements:**

* Add NbPoints attribute to RollingGrid to accelerate Size method (!128)
* Store rolling grid as unordered_map instead of stacked vectors (!130)
* Accelerate outputs transformations (!132)
* Avoid rebuilding KD-trees (!135)
* Change susbsampling strategy in overlap computation (!138)

**Bug fixes:**

* Fix keypoints extraction (!100)
* Fix warning, laser id mapping and undistortion time range (!105)
* Correct debug array error (!109)
* Fix Keypoints extraction and matching bugs and rename variables (!112)
* Add criterion to add keyframe (!124)
* Add checks to estimated resolution angle (!125)
* Check timestamp before ego motion extrapolation (!142, !151)
* Add a temporal window to estimate velocity (!141 and !143)
* Fix build in debug mode (!148)

**Compilation / CMake related changes / CI:**

* Add missing include to allow compilation on Win10 MSVC 2015 (!107)
* Allow to build LidarSlam lib as STATIC (!119 and !120)
* Add CI build jobs (!123)
* Specify CMake project version (!144)
* Fix build in debug mode (!148)
* Update to fit PV5.9 requirements (!157)
* Add regression tests on CI (!159)

### ROS wrapping

* Fix and clean ROS/catkin dependencies, compilation and installation (!121)
* Set initial maps and pose (!122)
* Include confidence in ROS documentation (!134)
* Allow to change the mapping mode online (!147)
* Add computation time to confidence estimation output (!159)
* Add config file for indoor context (!159)
* Complete documentation (!161)

### ParaView wrapping

* Optional calibration in PV wrapping (!106)
* Remove label field in xml to solve python access to variables (!116)
* Rename ENABLE_slam to LIDARVIEW_BUILD_SLAM (!126)
* Reformat XML proxy (!139)
* Allow to choose the mapping mode + the sampling mode (!147)
* Allow to load an initial map (!149)
* Update to fit PV5.9 requirements (!152, !153, !155, !157, !158, !163, !164)
* Add specific support for Hesai sensor (!165)

## *v1.4 (2021/04/01)*

This release brings important changes in code architecture (change SLAM point definition, add namespaces, reorganize ROS wrapping),
and adds new LiDAR sensors support (Velodyne and Ouster on PV, Velodyne and RoboSense on ROS).
It also focuses on improved performance and eases user experience.

Major changes are summarized below.

### Core lib

**Major new features:**

* Change SLAM point definition to add more point-wise fields (!64 and !65)
* Use real point wise time and compute output pose at frame timestamp (!82)
* Add multi lidar support, currently only available from ROS wrapping (!83 and !84)

**Refactoring and code architecture changes:**

* Standardize convention for pose formatting to XYZRPY (!66)
* Add `LidarSlam` and `Utils` namespaces and reorganize code structure (!72 and !73)
* Laser ID mapping arg becomes optional (!77)
* Move laser ID mapping parameter to wrappings (!79)
* Add `Utilities.cxx` to avoid defining entire helpers functions in header (!74 and !89)
* SLAM steps refactoring (!81)
* Major undistortion refactoring, remove `OPTIMIZED` mode and split `APPROXIMATED` to `ONCE` and `REFINED` (!87)

**Performance improvements:**

* Accelerate PCA computation and use float precision in keypoints extraction and registration (!86)
* Accelerate x3 linear transform interpolation (!88)
* Improve residuals weighting: fix duplicated weighting, use TukeyLoss robustifier, add debug info (!90)
* Simplify auto diff ceres (!95)

**Bug fixes:**

* Correct nb of laser rings detection (!70)
* Fix Ceres residuals conflict with LV's and deprecate unused residuals (!71)
* Fix ICP saturation range (!94)

### ROS wrapping

The ROS wrapping is adapted to the core lib improvements, such as the SLAM point definition, the optional laser ID mapping setting and other modified parameters and options.
It also now supports the multi-lidar mode to SLAM with several LiDAR devices.

- Update ROS wrapping to use ROS Velodyne driver 1.6 (!67)
- Allow inheritance from LidarSlamNode (!69)
- Add independent `lidar_conversions` package to perform pointclouds conversion to the expected SLAM point type (!68 and !75)
- Add Robosense RSLidar conversion node, and improve approximate point-wise timestamp computation (!76)
- Set point-wise `device_id` field in lidar_conversions nodes (!80)
- Update ROS params, doc and rename output frame topic (!85)

### ParaView wrapping

The PV wrapping is adapted to the core lib improvements, such as the SLAM point definition, the optional laser ID mapping setting and other modified parameters and options.

- Adapt PV wrapping to Ouster, and add manual setting for other LiDAR data (!56)

## *v1.3 (2020/11/20)*

This release brings various minor fixes or features to improve stability, performance or code readibility.

Major changes are reported below.

### Core lib

* Fix compilation on Windows with MSVC 2015 (!39)
* Simplify `KDTreePCLAdaptor` usage and clean its code (!41)
* Misc bug fixes and code refactoring in ICP functions (!43)
* Fix ego motion extrapolation when using Pose Graph Optimization (!44)
* Ignore Ego-Motion extrapolation, approximated undistortion and Localization results if error occurs (!49 and !52)
* Replace max variance error by max position and orientation errors (!47)
* Fix rolling grid and maps update (!50, !55, !58)
* Add missing `std` prefix in `PointCloudStorage` that prevente compilation on some machines (!42)
* Add colors to verbose display (!53)
* Major refactoring of ICP and LM optimization parts into separate classes (!54)
* Reset timers values when resetting SLAM (!63)

### ROS wrapping

* Publish SLAM pose as `gps_common/GPSFix` instead of `sensor_msgs/NavSatFix` (!45)
* Publish SLAM-estimated speed in GPS-like message (!46)

### ParaView wrapping

* Add verbose timings to check VTK <-> PCL conversions and perform less maps updates (!48)
* Fix frame stamp to last point measurement time (!49)

## *v1.2 (2020/06/26)*

This new release brings important improvements (in terms of processing speed as well as precision) such as *undistortion* or *motion extrapolation*. It also greatly improves user interface for easier parameters settings.

Major changes are reported below.

### Core lib

New features:
* Run SLAM in intermediary BASE coordinates system : user can set a rigid transform between BASE and LIDAR coordinates systems, in order to compensate lidar sensor's position and orientation relatively to the moving base plateform. Outputs are expressed in BASE frame, defaulting to LIDAR frame.
* Estimate realtive motion since last frame using previous motion extrapolation, and set Ego-Motion step as optional. This greatly improves processing speed, as well as stability for smooth movements (e.g. vehicles).
* Enable undistortion to correct rolling shutter distortion. 2 modes are available. This greatly improves precision.

Major bug fixes or improvements:
* Major acceleration of Ceres cost functions
* Remove undistortion in Ego-Motion step : this step is only used as an initialization for Localization step, and thus just need to be fast and approximate.
* Simplify `Slam` class by deleting unused attributes or refactoring them.
* Fix compilation warnings

Other noticeable changes changes:
* Updated copyright and Apache 2.0 license
* Rename Mapping step to Localization
* Add `Utilities.h` file to centralize all helper functions

ROS and ParaView wrappings have been adapted to support these previously mentionned changes.

### ROS wrapping

* Use *tf2* for sensors calibrations.
* Refactor output publishers and set TF publication as optional : user can now choose which outputs he wants to be published.
* Enable output of extracted keypoints from current frame.

### ParaView wrapping

* SLAM filters can now be applied on the selected input `vtkPolyData` frame, and not anymore on the `vtkTable` calibration. This is a more intuitive behavior.
* Filters proxies display bugs are fixed (duplicated titles or lines, parameters orders, hide unavailable parameters, ...).
* Update filters/parameters names and documentation.
* Add *How to SLAM with LidarView* tutorial.
* Maps and keypoints become optional outputs to avoid unecessary conversions and save time.
* Orientation is also saved as `AxisAngle` representation.
* Rename *DisplayMode* to *AdvancedReturnMode* and disable it by default.

## *v1.1 (2020/04/09)*

Add several functionalities to **v1.0**, such as compressed pointclouds logging, latency compensation, multi-threading or ParaView/LidarView plugin.

This release includes lots of bug fixes, code cleaning/refactoring or small and various improvements, but only major changes are reported below.

### Core lib

New features:

* Add transform extrapolation to compensate latency due to computation duration
* Add OpenMP multi-threading support for faster processing
* Enable keypoints maps saving/loading to/from PCD files
* Enable memory consumption display
* Enable logging keypoints and maps pointclouds as octree or PCD files to reduce memory usage

Major bug fixes or improvements:

* Major clean up, acceleration and fixes of `SpinningSensorKeypointExtractor`
* Fix maps update after PGO
* Fix Eigen alignment issues

General or CMake related changes:

* Rename *`slamlib`* to *`LidarSlam`*
* Add installation step for headers and libs
* Defaults to C++14 standard and RelWithDebInfo build type
* Ceres becomes a public dependency, G2O (and thus PGO) becomes optional
* Use modern CMake to link against Eigen and OpenMP targets if possible
* Move CI to docker

### ROS wrapping

* New SLAM functionalities support
* Add `SpinningSensorKeypointExtractor` parameters initialization from ROS parameter server
* Enable full 6D GPS pose conversions and pitch/heading computation from motion

### ParaView wrapping

* Working version of *`LidarSlamPlugin`*, ParaView plugin to enable using basic SLAM as a `vtkPolyData` algorithm
* This SLAM and ParaView plugin are now included in LidarView superbuild to be used directly in LidarView-based applications.

## *v1.0 (2019/12/18)*

First release of LiDAR SLAM as an independent project.
As this is the first 'official' version, most changes are not reported since **v0.0**, and only a small subset of useful or major changes is listed below.

### Core lib

* Numerous misc bug fixes and improvements
* Major code cleaning and refactoring
* Add CI for core SLAM lib
* Add pose graph optimization (PGO)
* Add optional logging of keypoints and trajectory
* Add verbosity modes to display state, steps durations and results
* Replace 6 DoF state vector by `Eigen::Isometry3d`
* Major acceleration of `RollingGrid`
* Add documentation for dependencies and installation

### ROS wrapping

* First version of ROS package `lidar_slam`, supporting all core SLAM lib functionalities
* Add SLAM parameters setting from ROS parameter server
* Optional SLAM/GPS global calibration from trajectories
* Add ROS package `gps_conversions` to manage conversions to standard `gps_common::GPSFix` message and process UTM/WGS84 transformations
* Compute GPS heading from movement when it is not available.
* Add documentation for usage

## *v0.0 (2019/11/11)*

Raw SLAM extracted from LidarView.