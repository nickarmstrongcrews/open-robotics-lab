/*
  Copyright 2011, 2012 Massachusetts Institute of Technology

  This work is sponsored by the Department of the Air Force under Air Force Contract #FA8721-05-C-0002.
  Opinions, interpretations, conclusions and recommendations are those of the authors and are not necessarily endorsed by the United States Government.
*/

#include <color_segmenter/color_segmenter.h>

// global vars (quick'n'dirty alternative to a class)
int targetR = 255;
int targetG = 0;
int targetB = 0;
int distThresh = 200;
bool normalizeIntensity = true;
ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  sensor_msgs::CvBridge bridge;
  IplImage *img;
  try
  {
    img = bridge.imgMsgToCv(msg, "bgr8");
    cvShowImage("Original", img);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  object_detection_msgs::Mask2D mask;
  mask.height = img->height;
  mask.width = img->width;
  mask.isValid.resize(mask.height*mask.width);
  for (int i = 0; i < img->height; i++)
  {
    for (int j = 0; j < img->width; j++)
    {
      unsigned int r = img->imageData[i * img->widthStep + 3*j + 2];
      unsigned int g = img->imageData[i * img->widthStep + 3*j + 1];
      unsigned int b = img->imageData[i * img->widthStep + 3*j];
      float intensity = r + g + b + 1;                         // add to avoid div by 0
      float targetIntensity = targetR + targetG + targetB + 1; // add to avoid div by 0
      double l1_dist_normalized = fabs(r/intensity-targetR/targetIntensity) + fabs(g/intensity-targetG/targetIntensity) + fabs(b/intensity-targetB/targetIntensity);
      double l1_dist = fabs(r-targetR) + fabs(g-targetG) + fabs(b-targetB);
      if(
	 (normalizeIntensity && l1_dist_normalized < distThresh/(3.0*255)) ||
	 (!normalizeIntensity && l1_dist < distThresh)
        ) {
	img->imageData[i * img->widthStep + 3*j + 2] = targetR;
	img->imageData[i * img->widthStep + 3*j + 1] = targetG;
	img->imageData[i * img->widthStep + 3*j    ] = targetB;
	mask.isValid[i * mask.width + j] = true;
      } else {
	img->imageData[i * img->widthStep + 3*j + 2] = 0;
	img->imageData[i * img->widthStep + 3*j + 1] = 0;
	img->imageData[i * img->widthStep + 3*j    ] = 0;
	mask.isValid[i * mask.width + j] = false;
      }
    }
  }

  pub.publish(mask);

  cvShowImage("Thresholded", img);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "color_tracker");
  ros::NodeHandle nh;

  nh.param("normalize_intensity", normalizeIntensity, true);
  nh.param("init_target_red",   targetR, 255);
  nh.param("init_target_green", targetG, 0);
  nh.param("init_target_blue",  targetB, 0);

  pub = nh.advertise<object_detection_msgs::Mask2D>("color_mask", 1);

  // open the two windows we'll be using
  cvNamedWindow("Original");
  cvNamedWindow("Thresholded");

  // add interactive sliders to change params in real-time
  cvCreateTrackbar("target red  ", "Thresholded", &targetR, 255, NULL);
  cvCreateTrackbar("target green", "Thresholded", &targetG, 255, NULL);
  cvCreateTrackbar("target blue ", "Thresholded", &targetB, 255, NULL);
  cvCreateTrackbar("dist thresh ", "Thresholded", &distThresh, 3*255, NULL);

  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);

  ros::spin();

  // cleanup
  cvDestroyWindow("Original");
  cvDestroyWindow("Thresholded");

  return 0;

}
