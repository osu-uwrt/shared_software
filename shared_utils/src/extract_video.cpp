/*
extract_video.cpp
Author: Benji Justice

Written with large influence from image_view.cpp
Use to extract video from ROS bag files.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>

boost::mutex imageMutex;

cv_bridge::CvtColorForDisplayOptions options;
cv::VideoWriter _videoWriter;
int count = 0;

void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  boost::mutex::scoped_lock lock(imageMutex);
  cv_bridge::CvImageConstPtr img_ptr;
  img_ptr = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(msg), "", options);
  if (!img_ptr->image.empty())
  {
    const cv::Mat &image = img_ptr->image;
    if (_videoWriter.isOpened())
    {
      _videoWriter.write(image);
      count++;
      ROS_INFO("Wrote frame #%d", count);
    }
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_video", ros::init_options::AnonymousName);
  if (ros::names::remap("image") == "image")
  {
    ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
             "\t$ rosrun image_view image_view image:=<image topic> _image_transport:=<transport>");
  }

  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  options.bg_label = -1;
  options.colormap = -1;
  options.do_dynamic_scaling = 0;
  options.max_image_value = 0.0;
  options.min_image_value = 0.0;

  // Gather parameters and parse arguments
  std::string topic = nh.resolveName("image");
  std::string transport;
  double sec_per_frame;
  local_nh.param("sec_per_frame", sec_per_frame, 0.033333);
  local_nh.param("image_transport", transport, std::string("raw"));
  ros::V_string myargv;
  ros::removeROSArgs(argc, argv, myargv);
  for (size_t i=1; i < myargv.size(); ++i)
  {
    if (myargv[i][0] != '-') {
      transport = myargv[i];
      break;
    }
  }

  ROS_INFO_STREAM("Using transport \"" << transport << "\" on topic \""<<topic<<"\"");

  // Set up image subscriber using image_transport
  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(transport, ros::TransportHints(), local_nh);
  image_transport::Subscriber sub = it.subscribe(topic, 1, imageCB, hints);

  // The size on this line must be equal to the image frame size
  _videoWriter = cv::VideoWriter("extracted.avi", CV_FOURCC('M', 'J', 'P', 'G'),
int(1.0/sec_per_frame), cv::Size(1288, 964));

  ros::spin();

  return 0;
}
