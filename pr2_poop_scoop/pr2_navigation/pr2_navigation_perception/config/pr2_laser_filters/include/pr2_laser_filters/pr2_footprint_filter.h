/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef PR2_LASER_SCAN_FOOTPRINT_FILTER_H
#define PR2_LASER_SCAN_FOOTPRINT_FILTER_H
/**
\author Tully Foote
@b ScanFootprintFilter takes input scans and corrects for footprint angle assuming a flat target.  
This is useful for ground plane extraction

**/


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"

namespace pr2_laser_filters
{

class PR2LaserScanFootprintFilterNew : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  PR2LaserScanFootprintFilterNew(){}

  bool configure()
  {
    if(!getParam("inscribed_radius", inscribed_radius_))
    {
      ROS_ERROR("PR2LaserScanFootprintFilter needs inscribed_radius to be set");
      return false;
    }
    return true;
  }

  virtual ~PR2LaserScanFootprintFilterNew()
  { 

  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan ;
    sensor_msgs::PointCloud laser_cloud;

    try{
    projector_.transformLaserScanToPointCloud("base_link", input_scan, laser_cloud, tf_);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Transform unavailable %s", ex.what());
      return false;
    }

    int c_idx = indexChannel(laser_cloud);

    if (c_idx == -1 || laser_cloud.channels[c_idx].values.size () == 0){
      ROS_ERROR("We need an index channel to be able to filter out the footprint");
      return false;
    }
    
    for (unsigned int i=0; i < laser_cloud.points.size(); i++)  
    {
      if (inFootprint(laser_cloud.points[i])){
        int index = laser_cloud.channels[c_idx].values[i];
        filtered_scan.ranges[index] = filtered_scan.range_max + 1.0 ; // If so, then make it a value bigger than the max range
      }
    }
    return true;
  }

  int indexChannel(const sensor_msgs::PointCloud& scan_cloud){
      int c_idx = -1;
      for (unsigned int d = 0; d < scan_cloud.get_channels_size (); d++)
      {
        if (scan_cloud.channels[d].name == "index")
        {
          c_idx = d;
          break;
        }
      }
      return c_idx;
  }

  bool inFootprint(const geometry_msgs::Point32& scan_pt){
    if(scan_pt.x < -1.0 * inscribed_radius_ || scan_pt.x > inscribed_radius_ || scan_pt.y < -1.0 * inscribed_radius_ || scan_pt.y > inscribed_radius_)
      return false;
    return true;
  }

private:
  tf::TransformListener tf_;
  laser_geometry::LaserProjection projector_;
  double inscribed_radius_;
} ;

}

#endif // PR2_LASER_SCAN_FOOTPRINT_FILTER_H
