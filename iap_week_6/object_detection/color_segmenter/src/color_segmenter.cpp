/*
  Copyright 2011, 2012, 2013 Massachusetts Institute of Technology

  This work is sponsored by the Office of the Assistant Secretary of Defense for Research and Engineering under Air Force Contract #FA8721-05-C-0002.
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

  cv_bridge::CvImagePtr img;
  try {
    img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::imshow("Original", img->image);

  object_detection_msgs::Mask2D mask;
  mask.height = img->image.rows;
  mask.width = img->image.cols;
  mask.isValid.resize(mask.height*mask.width);
  for (int i = 0; i < img->image.rows; i++)
  {
    for (int j = 0; j < img->image.cols; j++)
    {

      cv::Vec3b pixel = img->image.at<cv::Vec3b>(i,j);
      unsigned int b = pixel[0];
      unsigned int g = pixel[1];
      unsigned int r = pixel[2];

      float intensity = r + g + b + 1;                         // add to avoid div by 0
      float targetIntensity = targetR + targetG + targetB + 1; // add to avoid div by 0
      double l1_dist_normalized = fabs(r/intensity-targetR/targetIntensity) + fabs(g/intensity-targetG/targetIntensity) + fabs(b/intensity-targetB/targetIntensity);
      double l1_dist = fabs(r-targetR) + fabs(g-targetG) + fabs(b-targetB);
      if(
	 (normalizeIntensity && l1_dist_normalized < distThresh/255.0) ||
	 (!normalizeIntensity && l1_dist < distThresh)
        ) {
	img->image.at<cv::Vec3b>(i,j)[0] = targetB;
	img->image.at<cv::Vec3b>(i,j)[1] = targetG;
	img->image.at<cv::Vec3b>(i,j)[2] = targetR;
	mask.isValid[i * mask.width + j] = true;
      } else {
	img->image.at<cv::Vec3b>(i,j)[0] = 0;
	img->image.at<cv::Vec3b>(i,j)[1] = 0;
	img->image.at<cv::Vec3b>(i,j)[2] = 0;
	mask.isValid[i * mask.width + j] = false;
      }
    }
  }

  pub.publish(mask);

  cv::imshow("Thresholded", img->image);

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
