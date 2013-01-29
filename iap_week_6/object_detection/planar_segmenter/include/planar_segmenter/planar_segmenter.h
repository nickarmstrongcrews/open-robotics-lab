/*
  Copyright 2011, 2012, 2013 Massachusetts Institute of Technology

  This work is sponsored by the Office of the Assistant Secretary of Defense for Research and Engineering under Air Force Contract #FA8721-05-C-0002.
  Opinions, interpretations, conclusions and recommendations are those of the authors and are not necessarily endorsed by the United States Government.
*/

#ifndef PLANAR_SEGMENTER_H
#define PLANAR_SEGMENTER_H

#include <ros/ros.h>

#include <object_detection_msgs/Mask2D.h>

#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#endif // PLANAR_SEGMENTER_H
