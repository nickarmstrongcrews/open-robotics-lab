/*
  Copyright 2011, 2012 Massachusetts Institute of Technology

  This work is sponsored by the Department of the Air Force under Air Force Contract #FA8721-05-C-0002.
  Opinions, interpretations, conclusions and recommendations are those of the authors and are not necessarily endorsed by the United States Government.
*/

#ifndef COLOR_SEGMENTER_H
#define COLOR_SEGMENTER_H

#include <ros/ros.h>

#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

#include <object_detection_msgs/Mask2D.h>

#include <vector>

#endif // COLOR_SEGMENTER_H
