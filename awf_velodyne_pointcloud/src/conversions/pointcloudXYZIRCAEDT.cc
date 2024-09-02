/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <velodyne_pointcloud/pointcloudXYZIRCAEDT.h>

namespace velodyne_pointcloud
{
void PointcloudXYZIRCAEDT::addPoint(
    const float & x, const float & y, const float & z,
    const uint8_t & return_type, const uint16_t & channel, const uint16_t & azimuth, const uint16_t & elevation,
    const float & distance, const float & intensity,    
    const double & time_stamp) 
{
  velodyne_pointcloud::PointXYZIRCAEDT point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.return_type = return_type;
  point.channel = channel;
  point.azimuth = azimuth;
  point.elevation = elevation;
  point.distance = distance;
  point.time_stamp = time_stamp;


  pc->points.push_back(point);
  ++pc->width;
}
}  // namespace velodyne_pointcloud
