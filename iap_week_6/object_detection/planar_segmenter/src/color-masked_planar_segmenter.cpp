/*
  Copyright 2011, 2012, 2013 Massachusetts Institute of Technology

  This work is sponsored by the Office of the Assistant Secretary of Defense for Research and Engineering under Air Force Contract #FA8721-05-C-0002.
  Opinions, interpretations, conclusions and recommendations are those of the authors and are not necessarily endorsed by the United States Government.
*/

#include <planar_segmenter/planar_segmenter.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// global vars (quick'n'dirty alternative to a class)
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
PointCloud::ConstPtr pendingCloudMsg;
object_detection_msgs::Mask2DConstPtr pendingMaskMsg;

// find indices of points that mask says are valid
void maskCloud(object_detection_msgs::Mask2DConstPtr& mask, PointCloud::ConstPtr& cloud, pcl::PointIndices::Ptr& validIdxs)
{
  if(mask->height == cloud->height && mask->width == cloud->width) {
    for (int i = 0; i < mask->height; i++) {
      for (int j = 0; j < mask->width; j++) {
	if(mask->isValid[i*mask->width+j]==true) validIdxs->indices.push_back(i*mask->width+j);
      }
    }
    ROS_INFO("Masked down to %d from %d points", validIdxs->indices.size(), cloud->height*cloud->width);
  } else {
    ROS_ERROR("Mask not the same height/width as cloud!");
  }
}

void setupSegmentation(pcl::SACSegmentation<PointT>& seg) {
  seg.setOptimizeCoefficients (true); // extra post-hoc local optimization (optional)
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_PROSAC); // PROSAC is a faster, fancy version of RANSAC
  seg.setDistanceThreshold (0.01);  // points within this distance of plane are considered inliers
}

// TODO: make versions with fewer args
void performSegmentation(pcl::SACSegmentation<PointT>& seg, PointCloud::ConstPtr& cloud, PointCloud& cloud_inliers, PointCloud& cloud_remainder, pcl::ModelCoefficients& coeffs) {

  pcl::PointIndices::Ptr inlierIdxs(new pcl::PointIndices());

  seg.segment (*inlierIdxs, coeffs);

  // extract the inliers
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inlierIdxs);
  extract.filter (cloud_inliers);

  // and the remainder
  extract.setNegative (true);
  extract.filter (cloud_remainder);

  ROS_INFO("PointCloud with %d points (organized as %d x %d) segmented into %d planar and %d non-planar points",
           cloud->width*cloud->height,
           cloud->width, cloud->height,
           cloud_inliers.height*cloud_inliers.width,
           cloud_remainder.height*cloud_remainder.width
    );

}

// version without mask
void segmentCloud(PointCloud::ConstPtr& cloud, PointCloud& cloud_inliers, PointCloud& cloud_remainder, pcl::ModelCoefficients& coeffs) {
  pcl::SACSegmentation<PointT> seg;
  setupSegmentation(seg);
  seg.setInputCloud (cloud);
  performSegmentation(seg, cloud, cloud_inliers, cloud_remainder, coeffs);
}

// version with mask
void segmentCloud(PointCloud::ConstPtr& cloud, pcl::PointIndices::Ptr& validIdxs, PointCloud& cloud_inliers, PointCloud& cloud_remainder, pcl::ModelCoefficients& coeffs) {
  pcl::SACSegmentation<PointT> seg;
  setupSegmentation(seg);
  seg.setInputCloud (cloud);
  seg.setIndices(validIdxs);
  performSegmentation(seg, cloud, cloud_inliers, cloud_remainder, coeffs);
}

void processCloud()
{

  if(pendingCloudMsg==NULL) {
    ROS_WARN("no cloud msg yet...");
    return;
  } 

  // vars to hold output
  pcl::ModelCoefficients coeffs;
  PointCloud cloud_inliers;
  PointCloud cloud_remainder;

  if(pendingMaskMsg==NULL) {
    ROS_WARN("no mask msg yet, processing all points");
    segmentCloud(pendingCloudMsg, cloud_inliers, cloud_remainder, coeffs);
  } else {
    ROS_INFO("processing cloud w/ mask");
    // mask cloud (remove points that mask says are invalid)
    pcl::PointIndices::Ptr validIdxs(new pcl::PointIndices);
    maskCloud(pendingMaskMsg, pendingCloudMsg, validIdxs);
    if(validIdxs->indices.size() > 0)
      segmentCloud(pendingCloudMsg, validIdxs, cloud_inliers, cloud_remainder, coeffs);
    else {
    }
      
  }

  pub.publish(coeffs);
  pub2.publish(cloud_inliers);
  pub3.publish(cloud_remainder);

  pendingCloudMsg = PointCloud::ConstPtr(); // reset to NULL

}

/* // commented out because we aren't doing clustering, thus we don't need nearest-neighbor search data structure
// assumes organized point cloud (e.g., from Kinect)
void makeSearchStructure(PointCloud& cloud) {
  pcl::OrganizedNeighborSearch<PointT> orgNeighborIdx;
  orgNeighborIdx.setInputCloud(cloud.makeShared());
}
*/

/* // commented out because we're not using surface normals to detect plane
void estimateNormals(PointCloud& cloud, pcl::PointCloud<pcl::Normal>& normals) {
   pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
   ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE); // or AVERAGE_3D_GRADIENT (6x CPU) or COVARIANCE_MATRIX (9x CPU)
   ne.setMaxDepthChangeFactor(0.1f);
   ne.setNormalSmoothingSize(2.0f);
   ne.setInputCloud(cloud.makeShared());
   ne.compute(normals);
}
*/

void maskCallback (const object_detection_msgs::Mask2DConstPtr& maskMsg)
{
  ROS_INFO("got a mask msg");
  pendingMaskMsg = maskMsg; // stick it in global var, deal with it later
}

void cloudCallback(const PointCloud::ConstPtr& cloudMsg)
{
  ROS_INFO("got a cloud msg");
  pendingCloudMsg = cloudMsg; // stick it in global var, deal with it later
}

int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "planar_segmenter");
  ros::NodeHandle nh;

  // incoming data (queue size 1 => only cache most recent)
  ros::Subscriber sub = nh.subscribe ("input_cloud", 1, cloudCallback);
  ros::Subscriber sub2 = nh.subscribe ("color_mask", 1, maskCallback);

  // Create a ROS publisher for the output point cloud
  //   (if this is just for visualization, then probably publishing the Polygon representing the convex hull of inliers would be better)
  pub = nh.advertise<pcl::ModelCoefficients> ("planar_coeffs", 1);
  pub2 = nh.advertise<PointCloud> ("planar_points", 1);
  pub3 = nh.advertise<PointCloud> ("nonplanar_points", 1);

  // listen for and handle messages until ROS quits
  ros::Rate r(10); // 10 Hz
  while(ros::ok()) {
    ros::spinOnce();
    processCloud();
    r.sleep();
  }

}
