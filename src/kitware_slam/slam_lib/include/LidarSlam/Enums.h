//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Nicolas Cadart (Kitware SAS)
// Creation date: 2020-11-10
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#pragma once

#include <vector>
#include <string>
#include <map>

namespace LidarSlam
{

//------------------------------------------------------------------------------
//! Type of lidar point
enum LidarPointLabel
{
  DEFAULT = 0,
  FIXED = 1
};

//------------------------------------------------------------------------------
//! Type of a keypoint
enum Keypoint
{
  EDGE  = 0,   ///< edge keypoint (sharp local structure)
  INTENSITY_EDGE  = 1,   ///< intensity edge keypoint (sharp local intensity)
  PLANE = 2,   ///< plane keypoint (flat local structure)
  BLOB  = 3,   ///< blob keypoint (spherical local structure)
  nKeypointTypes
};

static const std::vector<Keypoint> KeypointTypes = { EDGE, INTENSITY_EDGE, PLANE, BLOB };
static const std::map<Keypoint, std::string> KeypointTypeNames = { {EDGE, "edge"}, {INTENSITY_EDGE, "intensity_edge"}, {PLANE, "plane"}, {BLOB, "blob"} };

//------------------------------------------------------------------------------
//! How to deal with undistortion
enum UndistortionMode
{
  //! No undistortion is performed:
  //!  - End scan pose is optimized using rigid registration of raw scan and map.
  //!  - Raw input scan is added to map.
  NONE = 0,

  //! Undistortion is performed only once using estimated ego-motion:
  //!  - Begin and end scan poses are linearly interpolated using estimated ego-motion.
  //!  - Scan is linearly undistorted between begin and end scan poses.
  //!  - Scan pose is iteratively optimized using rigid registration of undistorted scan and map.
  //!  - Undistorted scan is added to map.
  ONCE = 1,

  //! Undistortion is iteratively refined using optimized ego-motion:
  //!  - Begin and end scan poses are linearly interpolated using ego-motion.
  //!  - Scan is linearly undistorted between begin and end scan poses.
  //!  - Scan pose is optimized using rigid registration of undistorted scan and map.
  //!  - Iterate the three previous steps with updated ego-motion and poses.
  //!  - Undistorted scan is added to map.
  REFINED = 2,

  //! Undistort once with external pose information
  EXTERNAL = 3
};

//------------------------------------------------------------------------------
//! How to estimate Ego-Motion (approximate relative motion since last frame)
enum class EgoMotionMode
{
  //! No ego-motion step is performed : relative motion is Identity, new
  //! estimated Tworld is equal to previous Tworld.
  //! Fast, but may lead to unstable and imprecise Localization step if motion
  //! is important.
  NONE = 0,

  //! Previous motion is linearly extrapolated to estimate new Tworld pose
  //! from the 2 previous poses.
  //! Fast and precise if motion is roughly constant and continuous.
  MOTION_EXTRAPOLATION = 1,

  //! Estimate Trelative (and therefore Tworld) by globally registering new
  //! frame on previous frame.
  //! Slower and need textured enough environment, but do not rely on
  //! constant motion hypothesis.
  REGISTRATION = 2,

  //! Previous motion is linearly extrapolated to estimate new Tworld pose
  //! from the 2 previous poses. Then this estimation is refined by globally
  //! registering new frame on previous frame.
  //! Slower and need textured enough environment, but should be more precise
  //! and rely less on constant motion hypothesis.
  MOTION_EXTRAPOLATION_AND_REGISTRATION = 3,

  //! Use external pose as prior and none if external not available
  EXTERNAL = 4,

  //! Use external pose as prior and motion extrapolation if external not available
  EXTERNAL_OR_MOTION_EXTRAPOLATION = 5
};

enum KeypointExtractorMode
{
  //! Extract keypoints using SpinningSensorKeypointExtractor, convenient for all lidars.
  SPARSE = 0,

  //! Extract keypoints using DenseSpinningSensorKeypointExtractor, better option for lidars with 64 and 128 lasers.
  DENSE = 1,

  nbKeypointExtractorModes = 2
};

static const std::map<KeypointExtractorMode, std::string> KeypointExtractorModeNames = { {SPARSE, "sparse"}, {DENSE, "dense"} };

//------------------------------------------------------------------------------
namespace Interpolation
{
// List of poses interpolation models
enum Model
{
  LINEAR    = 0,  // Linear interpolation between 2 poses
  QUADRATIC = 1,  // Quadratic interpolation between 3 poses
  CUBIC     = 2,  // Cubic interpolation between 4 poses
};

static const std::map<Model, unsigned int> ModelRequiredNbData = { {LINEAR,    2},
                                                                   {QUADRATIC, 3},
                                                                   {CUBIC,     4} };
static const std::map<Model, std::string> ModelNames = { {LINEAR,    "Linear"},
                                                         {QUADRATIC, "Quadratic"},
                                                         {CUBIC,     "Cubic"} };

}  // end of Interpolation namespace

//------------------------------------------------------------------------------
//! How to update the map
enum class MappingMode
{
  //! Do not update map, use initial map
  //! Performant in static environment and
  //! more robust to moving objects
  // Forbiding maps update can be useful for example in case
  // of post-SLAM optimization with GPS and then run localization only in fixed
  // optimized map or when performing two SLAM steps (mapping + localization)
  NONE = 0,

  //! Expand the map with new keypoints
  //! The points of the initial maps (if some were loaded) will not be modified
  ADD_KPTS_TO_FIXED_MAP = 1,

  //! Update map with new keypoints
  //! The points of the initial maps can disappear
  UPDATE = 2,
};

//------------------------------------------------------------------------------
//! How to downsample for DSSKE
enum class SamplingModeDSSKE
{
  //! Use 2D downsampling to extract keypoints
  //! The grid is built using the VertexMap (like patches of an image)
  PATCH = 0,

  //! Use 3D downsampling to extract keypoints
  //! The grid is built using the original Scan cloud (like voxel grid in SSKE)
  VOXEL = 1,

  nbSamplingModes = 2
};

//------------------------------------------------------------------------------
//! How to downsample the map
// A voxel grid is used and various downsampling modes
// are possible to select the remaining point in each voxel
enum class SamplingMode
{
  //! Use the first point acquired
  //! Useful for performances issues
  FIRST = 0,

  //! Use the last point acquired
  //! Useful in dynamic environments
  LAST = 1,

  //! Use the point with maximum intensity
  //! The max intensity points can be the most acurate
  MAX_INTENSITY = 2,

  //! Use the closest point to the voxel center
  //! This allows the most uniform sampling but can be biased
  CENTER_POINT = 3,

  //! Use the centroid of the voxel
  //! This smoothes the points (can be useful for planes)
  //! /!\ The sampling process is longer
  CENTROID = 4
};

//------------------------------------------------------------------------------
//! External sensors' references
enum class ExternalSensor
{
  //! Wheel odometer
  WHEEL_ODOM = 0,

  //! IMU
  IMU = 1,

  //! Landmark detector
  LANDMARK_DETECTOR = 2,

  //! GPS
  GPS = 3,

  //! Pose sensor
  POSE = 4,

  //! Camera
  CAMERA = 5,

  //! Manual absolute position anchor
  MANUAL_POSITION = 6,

  nbExternalSensors
};

static const std::map<ExternalSensor, std::string> ExternalSensorNames = { {ExternalSensor::WHEEL_ODOM,        "wheel odometer"},
                                                                           {ExternalSensor::IMU,               "IMU"},
                                                                           {ExternalSensor::LANDMARK_DETECTOR, "landmark detector"},
                                                                           {ExternalSensor::GPS,               "GPS"},
                                                                           {ExternalSensor::POSE,              "external pose"},
                                                                           {ExternalSensor::CAMERA,            "camera"},
                                                                           {ExternalSensor::MANUAL_POSITION,   "manual position"} };

//------------------------------------------------------------------------------
//! Type of pose graph constraints
enum class PGOConstraint
{
  //! Loop closure
  LOOP_CLOSURE = 0,

  //! Landmark
  LANDMARK = 1,

  //! GPS
  GPS = 2,

  //! External pose
  EXT_POSE = 3,

  //! Manual absolute position
  MANUAL_POSITION = 4,

  nbPGOConstraints
};

static const std::map<PGOConstraint, std::string> PGOConstraintNames = { {PGOConstraint::LOOP_CLOSURE, "loop_closure"},
                                                                         {PGOConstraint::LANDMARK,     "landmark"},
                                                                         {PGOConstraint::GPS,          "gps"},
                                                                         {PGOConstraint::EXT_POSE,     "ext_pose"},
                                                                         {PGOConstraint::MANUAL_POSITION, "manual_position"} };

//------------------------------------------------------------------------------
//! Type of loop closure detector
enum class LoopClosureDetector
{
  //! Loop indices are provided from external source
  EXTERNAL = 0,

  //! Automatic detector by teaserpp registration
  TEASERPP = 1,

  //! Automatic detector by nearest pose search
  NEAREST_POSE = 2
};

//------------------------------------------------------------------------------
//! Modes to extract a submap of target keypoints
enum class PreSearchMode
{
  // Extract the bounding box of the current frame
  BOUNDING_BOX = 0,

  // Extract the voxels of the current frame and their neighbors
  PROFILE = 1
};

} // end of LidarSlam namespace