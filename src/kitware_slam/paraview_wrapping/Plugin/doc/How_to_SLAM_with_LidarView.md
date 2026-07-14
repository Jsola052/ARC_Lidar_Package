# How to SLAM with LidarView ?

This document presents some tips on how to use SLAM algorithm in LidarView, or one of its derived distribution. Even if this SLAM is embedded in a Paraview plugin and is therefore directly usable in Paraview, we will focus on its use in LidarView (as we consider here LiDAR data, LidarView is a better choice for most use-cases and display).

Since 2020, this SLAM plugin is natively included and available in [LidarView](https://lidarview.kitware.com/).

- [How to SLAM with LidarView ?](#how-to-slam-with-lidarview-)
  - [Installing LidarView or one of its derivative with SLAM support](#installing-lidarview-or-one-of-its-derivative-with-slam-support)
  - [Using SLAM in LidarView](#using-slam-in-lidarview)
    - [Apply SLAM on pcap and mcap files](#apply-slam-on-pcap-and-mcap-files)
    - [Apply SLAM on ROS2 pointcloud2 message](#apply-slam-on-ros2-pointcloud2-message)
    - [Multiple LiDAR sensor support](#multiple-lidar-sensor-support)
  - [Main functionalities](#main-functionalities)
    - [Reset](#reset)
    - [Center to current pose](#center-to-current-pose)
    - [Init](#init)
    - [External sensors](#external-sensors)
      - [Data](#data)
      - [Calibration](#calibration)
      - [Use](#use)
      - [Align to external sensor reference](#align-to-external-sensor-reference)
    - [Pose graph optimization](#pose-graph-optimization)
      - [PGO with external sensor](#pgo-with-external-sensor)
      - [PGO using manual anchors](#pgo-using-manual-anchors)
      - [PGO with loop closure](#pgo-with-loop-closure)
  - [Advanced functionalities](#advanced-functionalities)
    - [Clear maps and log](#clear-maps-and-log)
    - [SLAM in 2D](#slam-in-2d)
    - [Save/Reload trajectory](#savereload-trajectory)
    - [Save keypoint maps](#save-keypoint-maps)
    - [Save aggregated frames](#save-aggregated-frames)
      - [Visualize aggregated frames in viewer](#visualize-aggregated-frames-in-viewer)
      - [Directly aggregate all points in a LAS file](#directly-aggregate-all-points-in-a-las-file)
  - [SLAM parameters fine tuning](#slam-parameters-fine-tuning)
    - [Use case context](#use-case-context)
    - [Goal](#goal)
    - [Increasing the processing speed](#increasing-the-processing-speed)
    - [Accuracy/performances compromise](#accuracyperformances-compromise)
  - [SLAM evaluation](#slam-evaluation)
    - [Confidence estimators](#confidence-estimators)
    - [Failure detection](#failure-detection)


## Installing LidarView or one of its derivative with SLAM support

Pre-built binaries of LidarView with this SLAM plugin are available for download [here](https://gitlab.kitware.com/LidarView/lidarview/-/releases).

As these binaries may not always be up-to-date with the latest SLAM release, you may want to compile LidarView with SLAM support from source.
Follow [LidarView-Superbuild Guide](https://gitlab.kitware.com/LidarView/lidarview-superbuild/-/blob/master/README.md) instructions to build LidarView on Windows or Linux.

*__IMPORTANT__: to enable SLAM support, ensure your CMake configuration has these options set to `True` :*
```
-DENABLE_slam=True
```

For pose graph uses (see more details below with loop closure), g2o library is mandatory :
```
-DENABLE_g2o=True
```

For IMU uses (see more details below), GTSAM library is mandatory :
```
-DENABLE_gtsam=True
```

`LidarSlamPlugin` should be automatically loaded at LidarView's startup. If not, ensure **Tools** > **Manage Plugins** > **Load New**. Browse to your LidarView install directory and select the `libLidarSlamPlugin.so`.

## Using SLAM in LidarView

### Apply SLAM on pcap and mcap files

LidarView's SLAM has been tested on `.pcap` files acquired from several common LiDAR sensors, including:
- Velodyne (VLP-16, VLP-32c, HDL-32, HDL-64, VLS-128)
- Ouster (OS0/1/2-32/64/128)
- Hesai (PandarXT16, PandarXT32, Pandar128)
- Livox (HAP, Mid360)
- Leishen (C16, C32)
- Robosense (Ruby plus, Helios, bpearl)

Please note that your default LidarView application may not include all vendors-specific interpreters required to decode data from all these LiDAR sensors.

Starting from version 5.0, LidarView also supports the `.mcap` format, which is a format widely used in robotics applications. . So SLAM can therefore be applied to `.mcap` recordings as well.

1. Open LidarView. Choose "**Lidar Viewer mode**" for basic usage or "**Advanced mode**" if you wish to finely tune the parameters.

2. Open a previously recorded `.pcap` or `.mcap` file (or set up a stream source) associated with its LiDAR calibration file.
If you chose **Advanced mode** for LidarView : you can select the input of the SLAM in the **Pipeline browser** (top right) if multiple LiDAR frames are open. In this tutorial, **Frame** is selected by default.

3. Click the **Add a SLAM filter** button.

![Add SLAM](add_slam.png)

4. Tune the parameters for your use case :
  - Click the "**Open SLAM presets settings**" button.

![Presets button](presets.png)

  - Select the presets corresponding to your use case in the 3 fields (environment/LiDAR model/LiDAR support)

![Presets](presets.gif)

  - If you are using **Advanced mode**, you can fine-tune SLAM parameters in the **Properties** panel (bottom right). See the [SLAM parameters tuning section](#slam-parameters-tuning).
  - Once configured, hit **Apply**.

5. In **Advanced mode**, you can also adjust the visualisation in the **Display** panel (bottom right) to fit your needs (e.g. modify point color or size).

### Apply SLAM on ROS2 pointcloud2 message

Starting from version 5.3, LidarView supports ROS2 subscriptions. To create a ROS2 subscriber:
- Go to **Sources** -> **ROS 2 PointCloud DDS Subscriber**
- Set **Topic Name** to receive PointCloud2 message from a ROS bag or live ROS2 system.
- Apply the SLAM filter to this subscriber source.

### Multiple LiDAR sensor support

Multi-sensor fusion in SLAM increases the overall field of view and improves environment coverage. In LidarView, you can apply the SLAM filter to multiple LiDAR sensors simultaneously. At each timestep in the LidarView pipeline, the system checks whether a new frame has been received from each sensor and whether its timestamp has been updated since the last processed frame. The frames are forwarded to the SLAM module only when all expected inputs are available and have been updated, or when a timeout is reached.

To use this feature:
- Apply a transform to each LiDAR sensor so that its point cloud is expressed in a common base frame
- Select all frames and apply the **Group Datasets** filter to combine them into a multiblock dataset in LidarView
- Apply the SLAM filter to the grouped dataset.

For ROS bag data, create one PointCloud2 subscriber source per LiDAR topic before applying the SLAM filter. If the point timestamps are relative to the frame timestamp, enable the **Point time relative to frame** option.

## Main functionalities

### Reset

A button allows to reset the SLAM at a current frame :
* the SLAM is reset to initial state : pose and map. The default pose is identity and the default map is empty.
* The current frame is processed from the init pose in the init map
* the external sensor data are cleared and refilled
* the parameters are kept unchanged

![Reset](reset.png)

### Center to current pose

To reduce numerical instability when working with large coordinates, this button resets the SLAM odometry to the current pose:
* The current pose is redefined as the origin (x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0)
* All logged poses and maps are recalculated relative to this new reference frame
* In outdoor environments, it is recommended to reset ODOM regularly, every few kilometers, to maintain accuracy
This feature is particularly useful when working with large coordinates from external sensors such as GNSS or INS

### Init

A button allows to apply an initial state (pose and map) before slamming. If a trajectory already exists, a reset action is required from the user to initialize. This reset will remove the current map.

![Initialization](init.png)
![Initialization pop up](init_popup.png)

### External sensors

External sensors can be used to improve SLAM robustness solving geometric ambiguities which lead to SLAM failure cases and/or to improve SLAM accuracy, reducing the drift.

#### Data

External sensor information (wheel odometer OR IMU OR external poses) can be used in the SLAM process.
They must be provided in a CSV file and can come with a calibration matrix file (see [next section](#calibration)).
The possible fields of the CSV file are :

- *Time*: POSIX time (Unix timestamp in seconds), synchronized with the Lidar frame timestamps or at least to the packets reception time -> mandatory

- *odom*: Wheel odometry, in meters -> optional

- *acc_x/acc_y/acc_z*: Acceleration from IMU, in meters/second^2 -> optional

- *w_x/w_y/w_z*: Angle velocities from IMU, in radians/second -> optional

- *X/Y/Z/Rx(Roll)/Ry(Pitch)/Rz(Yaw)*: Absolute pose measurements in meters and radians (Rz * Ry * Rx order) -> optional
- *errX/errY/errZ/errRoll/errPitch/errYaw*: Estimated error in meters and radians -> optional

#### Calibration

The calibration file must lay in the same directory as the CSV file and must be named *calibration_external_sensor.mat*. This calibration file must contain the 4x4 calibration matrix representing the transform from external poses sensor to Base frame (i.e. the tracked frame).

**Example** :
```
1 0 0 0.5
0 -1 0 2
0 0 -1 0.2
0 0 0 1
```

If the calibration file is not provided, it is set to identity (i.e. the information is supposed to be represented in **BASE** frame) and a warning is triggered.
The calibration can be estimated automatically using trustworthy trajectory portions (see Advanced functionalities).

#### Use

Click on the "**Open add external sensor dialog**".

![External sensor](ext_sensor.png)
![External sensor pop up](ext_sensor_popup.png)

The external sensor can be used to add a constraint to the local SLAM optimization. The user must enter a weight corresponding to the external sensor used. This weight must be set experimentally knowing that it will be the confidence factor of the external sensor constraint relatively to all the keypoint matches. If the weight is null, the constraint is not added to the optimization. The constraints are :

- For the odometer a translation constraint between two successive SLAM poses or an absolute translation constraint which can be used in specific contexts such as mine exploration.
- For the IMU (when only the accelerations are provided), a gravity constraint between all frames. To do so, the acceleration of the base frame is considered as null, the acceleration measured by the IMU should only represent the gravity.
- For the external poses, a relative transform constraint between frames.

External poses (obtained by preintegrating the IMU or from another source) can also be used to :
- Estimate a prior pose : the user must choose the *External* or *External OR motion extrapolation* modes in the Ego-motion selector.
- De-skew the pointcloud : the user must choose the *External* mode in the undistortion selector.

When IMU accelerations **and** IMU angle velocities are provided along with the gravity in the world frame, those data can be preintegrated to get poses. Then, those poses can be used exactly as external poses (see features above). To perform the preintegration, a parallel graph is built between IMU and SLAM poses. This graph allows to update the biases and the integration constants. To limit the graph growth, a threshold is added to reset the graph.

**Note** : This feature needs GTSAM in order to be built (available in the superbuild).

**Warning** : the preintegration is not real time.

Finally, the user can choose which synchronization to perform (timestamps supplied by the sensor or packet reception timestamps). The time reference chosen must be the same as the one provided in the CSV file. If the packet reception is chosen, the user must be sure that their is no lag between the external sensor acquisition and the packet reception (no post-process of the data).

#### Align to external sensor reference

Aligns SLAM odometry to the reference frame of an external sensor (e.g., GNSS or INS) to produce georeferenced trajectories and maps. Available only when external sensor data is present.

Recommended workflow: When using an external reference pose sensor, complete SLAM and/or pose graph optimization first, then align all poses and maps to the georeferenced frame using this button.

### Pose graph optimization

As a postprocess, the whole trajectory can be refined at some point using external informations. Those informations can be :
* Loop closure
* GNSS positions
* External trajectory

It consists in building a pose graph linking all current SLAM poses together with SLAM transforms. Then, new constraints are created between the poses. Finally, the graph is optimized (i.e. the poses are moved) so it complies with all constraints.
The computation time of this process depends on how many poses are stored in the trajectory at the current time of launch. The number of poses added to the trajectory is limited in time by the parameter **logging Time**.

**NOTE** the pose graph optimization cannot solve SLAM failures. It can only be used to reduce the drift.

To launch the pose graph optimization click on the button **Open SLAM optimize graph dialog**.
![Pose graph](pose_graph.png)
![Pose graph pop up](pose_graph_popup.png)

#### PGO with external sensor

When using GNSS or an external trajectory, refer to [external sensors section](#external-sensors) to load the data.
At any point of the trajectory, enable **Use GPS constraint** or **Use external pose constraint**, then launch the optimization by clicking  **Optimize Graph**.

**External pose drop handling**: In the **external sensors** category, you can enable the **Auto correct external pose drop** option and set the translation and rotation error thresholds. This activates external sensor dropout detection. When a dropout is detected and the signal returns, a pose graph optimization is triggered automatically. This feature can only be used when error information is available in the loaded external trajectory file.
*Example*: If a vehicle enters a tunnel and temporarily loses the GPS signal, the system can automatically trigger a pose graph optimization once the vehicle exits and the GPS signal returns back.

#### PGO using manual anchors

Manual anchors provide a way to locally correct SLAM trajectory drift when no external sensor data is available. They operate directly on the pose graph and can be used to refine selected parts of the trajectory.

Two types of anchors are supported:
* **Manual position anchors** constrain translation only (XYZ) of a given pose.
* **Fixed pose anchors** constrain the full 6-DoF (translation + rotation) and act as hard stops: optimization corrections do not propagate backward past the anchored pose.

![manual anchors](manual_pose_constraint.gif)
**In order to use this feature:**
* Enable **Use manual position constraint** in the Pose Graph Optimization dialog.
* Optionally enable **Define anchor points** to allow fixed 6‑DoF anchors.
* An interactive cursor will appear at the coordinate origin, select the target position by dragging the cursor
* Click **Pick trajectory index** to choose the initial target position in the render view
* Then click **Append to Table** to add it to the list of constraint points waiting for optimization
* Finally click **Apply** and **Optimize Graph** to run the optimization

#### PGO with loop closure

Loop closure addresses the case when some place is revisited after a period of time.
To add a loop closure constraint to the graph, it must be first detected.
For now, the loop closure detection must be triggered by the user. 2 options are available :

* **External** : A list of pairs of frames that define loop closures can be provided as a CSV file. Those pairs contain a "query frame" and a "revisited frame" which should sample the same scene from different points of view.
Here is an example of csv file:

```
queryIdx,revisitedIdx
123,12
456,23
343,35
```
Once the CSV file is loaded, the graph can be optimized clicking on **Optimize Graph**.

![Loop closure](loop_closure.png)

* **Teaserpp** : if teaserpp loop closure detector is chosen, a loop closure will be detected for the current frame by using teaserpp registration algorithm. The detected area will be marked in the render view and a dialog pops up to require the user how to use the detected frame.
  - **Close loop now**: use loop closure constraint to optimize graph
  - **Discard**: do not use this detected loop
  - **Keep it for later**: save detected loop indice for slam to be used later
    ![LC autostart interface](loop_closure_autostart.png)

**Note**: if you chose **Advanced mode** for LidarView :
  * You can tune the loop closure parameters to fit your use case checking the box **Advanced settings** (see next picture).

![LC advanced parameters](loop_closure_advanced.png)

  * **Reset loop detection** button can be used to clean current loop detections, they won't be used in new graph optimization.

## Advanced functionalities

This section holds if you chose **Advanced mode** for LidarView.
Those actions are possible in the **Properties** panel (bottom right).

### Clear maps and log

A button allows to clear the current maps and the logged keypoints but keeps the external sensors state and the actual state (pose and ego-motion).

### SLAM in 2D

This checkbox allows to perform the whole process in 2D, optimizing only x, y and yaw in the world frame.

### Save/Reload trajectory

You can export a trajectory to inspect it or to avoid running the SLAM again if it fails at some point. Select the **Trajectory** output in the **Pipeline Browser** panel, then hit `Ctrl+s` and select `.csv` or `.poses` format.

You can then reload the trajectory.

![Reload trajectory](reload_traj.gif)

This can also be used to compare or mix some trajectories acquired through different methods.

### Save keypoint maps

Keypoint maps are the local downsampled aggregations of registered keypoints from previous frames. It provides a nice light-weight insight of the reconstructed scene, and helps supervising or interpreting the SLAM behavior and results.

To save SLAM keypoints maps, select the map output you want to save in the **Pipeline Browser** panel (top right), then hit `Ctrl+s`, and choose the output format and name in the dialog window. Common pointclouds formats are `csv`, `pcd`, `las`, `ply` or `vtp`.

If you saved it in PCD format, they can be used as initial map in the SLAM (see [init section](#init))

### Save aggregated frames

If the visualization of the maps isn't enough for your application, and you need to aggregate all points from all previous frames, this is possible too, but less straightforward.

There are two ways to export aggregated scans:
- The first one allows to aggregate previous LiDAR scans into a single pointcloud that can be visualized in LidarView or Paraview. This pointcloud can be optionally downsampled then saved on disk. As this method aggregates points before saving them, the memory consumption is important and can lead to saturation.
- The second method uses a previously saved trajectory to aggregate successive scans by directly appending data to a LAS file on disk.

#### Visualize aggregated frames in viewer

*__Note__: Be careful, the aggregation of ALL previous points may represent a huge pointcloud! With ~1 000 000 points per second and the associated measurements (intensity, time, laser ID, etc.), each minute of recording represents several GB of data! Depending on your machine specifications, this may not fit into memory and lead to an annoying crash of the application.*

1. Instantiate an **Aggregate Points From Trajectory** filter using **Frame** as point cloud entry, and the output SLAM **Trajectory** for trajectory entry. Depending on the chosen number of frames, the transformation and aggregation of pointclouds may be long. When it succeeds, you should now see all points being correctly registered. All the frames or a chosen range of frames can be aggregated. For example, if you want to aggregate points from frame #100 to frame #500, but using only points from 1 frame out of 3, specify First Frame = 100, Last Frame = 500, Frame Stride = 3. The processed frames can be manually chosen with a **Aggregate Points From Trajectory Online** filter instead of **Aggregate Points From Trajectory**. By default, the points are filtered by a voxel grid defined by a leaf size.
If the colors look strange, check that you are displaying the `intensity` array in the main toolbar.

![Aggregated frames](aggregated_frames.png)

This first step allows you to visualize all the aggregated points in LidarView. If you want to save it on disk, please follow this additional step:

2. As usual, save aggregated frames by selecting the desired output **Aggregate Points From Trajectory**, hit `Ctrl+s`, and choose the output format (LAS, CSV, PLY, VTP) and name.

#### Directly aggregate all points in a LAS file

This method directly appends points to a LAS file on disk, which has the advantage to avoid saturating the RAM.

1. Instantiate a **Aggregate Points From Trajectory** filter using the **Frame** as point cloud entry, and the output SLAM **Trajectory** for trajectory entry.

2. Save the output of the **Aggregate Points From Trajectory** by hitting `Ctrl+s`, selecting the **LAS point cloud file** format, and specifying the output file name before validating. A new dialog will appear to configure the LAS file writer, where you can modify the parameters to your needs. For example, if you want to aggregate points from frame 100 to 500, but using only points from 1 frame out of 3, specify *First Frame = 100*, *Last Frame = 500*, *Frame Stride = 3*.
The export can be quite long (from a few seconds to several minutes) as each specified frame needs to be processed.

## SLAM parameters fine tuning

*__Note__: To see all parameters, make sure that the advanced properties of the SLAM filter are enabled by toggling the little gear wheel in the **Properties** panel.*

### Use case context

The default SLAM parameters should be a good compromise to run the SLAM for most outdoor urban environments with the LiDAR sensor mounted on top of a vehicle but are not necessarily adapted to your use case. Principal factors which impact the parameter settings are :
* LiDAR support :
  * Are there occlusions?
  * Is there high frequency motion?
  * Is there a lot of overlap between frames?
* Environment
  * Is it structured?
  * Is motion fast in this environment?
  * Is it large/narrow?
* LiDAR model :
  * How many points per frame does it have?
  * What is the field of view?
  * What is the accuracy for close/far points?
  * What is the accuracy on a plane?

The presets (see [first section](#using-slam-in-lidarview)) can help you tune the SLAM to your specific use case.
If you want to inspect relative parameters check the [preset files](../UserInterface/Resources/Presets/).

To adapt the field of view, you can play with the parameters :
* **Min distance to sensor** : to remove closest points in case of occlusion or points not trustworthy enough (e.g a human following the Lidar).
* **Max distance to sensor** : to remove points that are not trustworthy enough
* **Min/Max azimuth angle** : to remove occlusions

### Goal

If you don't want to map but you want to localize in a previously acquired map, you can use the localization mode. To do so, modify the parameter **Mapping mode** to forbid the update of the map or to only add points that have never been seen, keeping initial map points. This last mode can be useful in case of a mid-term dynamic scene

### Increasing the processing speed

To increase the processing speed, consider also tweaking these parameters:

- **Number of threads**: Maximum number of threads used for parallel processing. Allowing several threads (about 4) increases SLAM processing speed, skipping less frames, and thus improving result.

- **Keypoints maps update step**: If you don't need the map display to be refreshed at each new frame, you should consider increasing this value. Setting it to 10 will only update maps every 10th frame (1 second at 600 rpm), which is far enough for a nice visualization. This will save some output conversion time.

- **LidarView play speed**: This is not specific to the SLAM filter, but LidarView is controlling the playback speed of the LiDAR recording. It can be set from the VCR toolbar. For example, *Speed = x1* will play at real speed, *Speed = x3* will play 3 times faster. If the SLAM algorithm isn't fast enough to process all incoming frames, it will drop some of them. If your LiDAR is slowly moving or with smooth motion, this not a problem. However, if it skips too many frames compared to the LiDAR motion, consider choosing the *Speed = All Frames*, which will play as fast as possible but ensuring that all frames are processed.

### Accuracy/performances compromise

The following parameters can be tuned to find a compromise between computation time and result accuracy :

* **Maximum keypoints number** : the maximum number of points that are processed at each frame. Each specific type keypoints are downsampled uniformly to reach this number.
* **Ratio of points** : One point over N will be processed. This can be tuned in case of very dense LiDAR sensors to limit computation time.
* **Submap extraction mode** : one mode or the other can lead to different performance results depending on the Lidar orientation relatively to the motion.
* **X map resolution** : the density of the map can be increased for better accuracy in some cases at the cost of an increase of computation time. **Warning**: a very dense map can lead to a loss of accuracy because of local minima issues.

## SLAM evaluation

### Confidence estimators

Some confidence metrics have been implemented to evaluate the SLAM result:
- Overlap estimation (between 0 and 1)
- Motion limits compliance relatively to input thresholds
- Error standard deviation based on covariance analysis
- Number of keypoints matched

To observe those metrics, you have to enable the *Advanced return mode* in the filter parameters. The trajectory filter output should now contain the metrics data and you can observe it as a color on trajectory points. Then, one interesting method is to display the metrics in another render view :
- Click on the icon "Split horizontal axis"
- In the new render view, choose *Line chart view*
- Select the output of the SLAM filter named **Trajectory** (click on the eye at the left of the Trajectory object). You should see some curves representing the metrics.
- In the **Display** pannel (bottom right), when working on the new renderview and having selected the Trajectory object, you should be able to select the metrics you want to observe.

![confidence metrics](confidence_metrics.gif)

### Failure detection

Another feature has been developped to fuse the confidence estimators to trigger a failure. The failure cases that can be detected include :
- Map doubling, due to an isolated high motion, a temporal big occlusion or to quick scene change (e.g. door crossing)
- Lack of degree of liberty (e.g. corridor case)
- Divergence due to a combination of external factors

To enable this feature, you should turn on *Failure detection* in the SLAM filter parameters.

In case a failure is detected, the filter enters a recovery mode. This mode fixes the map and the trajectory to an older state and automatically updates some of the parameters to allow a bigger motion and a longer computation time. The user should go back to a previous pose to try to be relocalized and get out of this mode to go on with his acquisition without breaking the map. He can also update the parameters for the specific trajectory part that has gone wrong.

**/!\ Warning** : for now, going back to a previous pose includes orientation, so, mind your acquisition direction when looking for recovery.

If this feature is disabled during recovery, the state is reset as before the recovery mode has been triggered. Therefore, if you see the SLAM is relocalized but the recovery mode is still on (the confidence thresholds have not been reached), you can disable the failure detection and reenable it later on to force going out of the recovery mode.

**/!\ Warning** : in recovery mode, some of the parameters displayed in the filter interface are not excatly the ones used :
* **Map update mode**
* **Ego-motion mode**
* **Undistortion mode**
* **Maximum number of ICP iterations**
* **Maximum distance between nearest neighbors**
* **Initial saturation distance**

 If you change one of these parameters during the recovery mode, it will be reset as before the recovery mode after relocalization. Therefore, again, the parameters used will not be the ones displayed in the filter interface.
