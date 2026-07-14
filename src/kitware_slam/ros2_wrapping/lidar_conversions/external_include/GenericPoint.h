//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Faure Jeanne (Kitware SAS)
// Creation date: 2023-09-29
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

#include <pcl_conversions/pcl_conversions.h>

#ifndef PCL_POINT_TYPES_H
#define PCL_POINT_TYPES_H

#include <pcl/point_types.h>

#define DEFINE_AND_REGISTER_POINT_TYPE(name, fieldType, field) \
  namespace point_conversions                                  \
  {                                                            \
    struct name                                                \
    {                                                          \
      fieldType field;                                         \
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW                          \
    } EIGEN_ALIGN16;                                           \
  }                                                            \
  POINT_CLOUD_REGISTER_POINT_STRUCT(point_conversions::name,   \
                                    (fieldType, field, field))

#define INIT_AND_REGISTER_TYPE(pointName, fieldMsg)                                  \
  DEFINE_AND_REGISTER_POINT_TYPE(Point##pointName##_Int8,   std::int8_t,   fieldMsg) \
  DEFINE_AND_REGISTER_POINT_TYPE(Point##pointName##_Uint8,  std::uint8_t,  fieldMsg) \
  DEFINE_AND_REGISTER_POINT_TYPE(Point##pointName##_Int16,  std::int16_t,  fieldMsg) \
  DEFINE_AND_REGISTER_POINT_TYPE(Point##pointName##_Uint16, std::uint16_t, fieldMsg) \
  DEFINE_AND_REGISTER_POINT_TYPE(Point##pointName##_Int32,  std::int32_t,  fieldMsg) \
  DEFINE_AND_REGISTER_POINT_TYPE(Point##pointName##_Uint32, std::uint32_t, fieldMsg) \
  DEFINE_AND_REGISTER_POINT_TYPE(Point##pointName##_Float,  float,         fieldMsg) \
  DEFINE_AND_REGISTER_POINT_TYPE(Point##pointName##_Double, double,        fieldMsg)

INIT_AND_REGISTER_TYPE(I, intensity)
INIT_AND_REGISTER_TYPE(Ref, reflectivity)
INIT_AND_REGISTER_TYPE(Id, laser_id)
INIT_AND_REGISTER_TYPE(Ring, ring)
INIT_AND_REGISTER_TYPE(Time, time)
INIT_AND_REGISTER_TYPE(T, t)

#endif // PCL_POINT_TYPES_H
