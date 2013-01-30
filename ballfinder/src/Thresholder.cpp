#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  static void drawCrossHairs(cv::Mat& Img, cv::Point center, CvScalar color)
  {
    int offset = 10;
    int lineLength = 25;
    int thickness = 5;
    //int total = offset + lineLength + thickness + 1;
    //cout << "rows: " << Img.rows << endl;
    /*    CvSize size = Img.size;
    if (center.x + total > size.width)
      {
	center.x = size.width - total;
      }
    if (center.x - total < 0)
      {
	center.x = total;
      }
    if (center.y + total > size.height)
      {
	center.y = size.height - total;
      }
    if (center.y - total < 0)
      {
	center.y = total;
      }
    */
    cv::line(Img, 
	     cv::Point(center.x + offset, center.y), 
	     cv::Point(center.x + offset + lineLength, center.y), 
	     color, thickness);
    cv::line(Img, 
	     cv::Point(center.x - offset, center.y), 
	     cv::Point(center.x - offset - lineLength, center.y), 
	     color, thickness);
    cv::line(Img, 
	     cv::Point(center.x, center.y + offset), 
	     cv::Point(center.x, center.y + offset + lineLength), 
	     color, thickness);
    cv::line(Img, 
	     cv::Point(center.x, center.y - offset), 
	     cv::Point(center.x, center.y - offset - lineLength), 
	     color, thickness);    
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    IplImage* imgHSV = cvCreateImage(cvSize(cv_ptr->image.cols, cv_ptr->image.rows), 8, 3);
    Mat matImgHSV(imgHSV);
    cv::cvtColor((cv_ptr->image), matImgHSV, CV_BGR2HSV);
    IplImage* imgThreshed = cvCreateImage(cvSize(cv_ptr->image.cols, cv_ptr->image.rows), 8, 1);
    cvInRangeS(imgHSV, cvScalar(0, 210, 100), cvScalar(20, 255, 255), imgThreshed);

    Mat matImgThreshed(imgThreshed);

    //    ImageConverter::drawCrossHairs(cv_ptr->image, cv::Point(0, 420), CV_RGB(200,255,20));


    cout << "Got an image" << endl;
    //    cv::imshow(WINDOW, cv_ptr->image);
    cv::imshow(WINDOW, matImgThreshed);
    cv::waitKey(1);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
