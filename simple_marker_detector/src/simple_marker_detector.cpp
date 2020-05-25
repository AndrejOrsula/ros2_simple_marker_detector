/// Detector of simple markers

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>

// ROS 2 interfaces
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "simple_marker_detector";
/// Size of the queue size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 50;

/// Delay used in the context of cv::waitKey, applicable only if visualisation is enabled
const int CV_WAITKEY_DELAY = 10;

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                  sensor_msgs::msg::Image,
                                                  sensor_msgs::msg::CameraInfo>
    synchronizer_policy;

////////////////////////
/// HELPER FUNCTIONS ///
////////////////////////

//////////////////
/// NODE CLASS ///
//////////////////

/// Class representation of this node
class SimpleMarkerDetector : public rclcpp::Node
{
public:
  /// Constructor
  SimpleMarkerDetector();

private:
  /// Subscriber to color frames
  image_transport::SubscriberFilter sub_color_;
  /// Subscriber to registered (aligned) depth frames
  image_transport::SubscriberFilter sub_depth_;
  /// Subscriber to the camera info
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_;
  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Publisher of the pupil centres
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_marker_centre_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const sensor_msgs::msg::Image::SharedPtr msg_img_color,
                             const sensor_msgs::msg::Image::SharedPtr msg_img_depth,
                             const sensor_msgs::msg::CameraInfo::SharedPtr msg_camera_info);

  /// Create a 3D cloud point at a specific pixel from depth map
  bool create_cloud_point(
      const cv::Mat_<uint16_t> &img_depth,
      cv::Point3_<double> &output,
      const std::array<double, 9> &camera_matrix,
      cv::Point_<float> &pixel,
      const double depth_scale = 0.001);
};

SimpleMarkerDetector::SimpleMarkerDetector() : Node(NODE_NAME),
                                               sub_color_(this, "camera/color/image_raw", "raw"),
                                               sub_depth_(this, "camera/aligned_depth_to_color/image_raw", "raw"),
                                               sub_camera_info_(this, "camera/aligned_depth_to_color/camera_info"),
                                               synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_color_, sub_depth_, sub_camera_info_)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&SimpleMarkerDetector::synchronized_callback, this);

  // Register publisher of the pupil centres
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  pub_marker_centre_ = this->create_publisher<geometry_msgs::msg::PointStamped>("marker_centre", qos);

  // Parameters of the element
  this->declare_parameter<bool>("visualise", true);
  this->declare_parameter<int>("range.hue.min", 40);
  this->declare_parameter<int>("range.hue.max", 80);
  this->declare_parameter<int>("range.saturation.min", 50);
  this->declare_parameter<int>("range.saturation.max", 225);
  this->declare_parameter<int>("range.value.min", 50);
  this->declare_parameter<int>("range.value.max", 225);

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void SimpleMarkerDetector::synchronized_callback(const sensor_msgs::msg::Image::SharedPtr msg_img_color,
                                                 const sensor_msgs::msg::Image::SharedPtr msg_img_depth,
                                                 const sensor_msgs::msg::CameraInfo::SharedPtr msg_camera_info)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");

  // Create output msg
  geometry_msgs::msg::PointStamped marker_centre;
  marker_centre.header = msg_img_color->header;

  // Convert color and depth images to CV
  cv_bridge::CvImagePtr img_color, img_depth;
  try
  {
    img_color = cv_bridge::toCvCopy(msg_img_color, sensor_msgs::image_encodings::BGR8);
    img_depth = cv_bridge::toCvCopy(msg_img_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (const cv_bridge::Exception exception)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid frame. Exception from cv_bridge: %s", exception.what());
    return;
  }

  // Convert to HSV
  cv::Mat img_color_hsv;
  cv::cvtColor(img_color->image, img_color_hsv, cv::COLOR_BGR2HSV);

  // Threshold green hue
  cv::Mat green_mask;
  cv::inRange(img_color_hsv, cv::Scalar(this->get_parameter("range.hue.min").get_value<int>(), this->get_parameter("range.saturation.min").get_value<int>(), this->get_parameter("range.value.min").get_value<int>()), 
  cv::Scalar(this->get_parameter("range.hue.max").get_value<int>(), this->get_parameter("range.saturation.max").get_value<int>(), this->get_parameter("range.value.max").get_value<int>()), green_mask);

  // Apply morphological opening and closing
  cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
  cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

  // Visualise, if enabled
  if (this->get_parameter("visualise").get_value<bool>())
  {
    cv::imshow("original", img_color->image);
    cv::imshow("green", green_mask);
    cv::imshow("green_morphed", green_mask);
    cv::waitKey(CV_WAITKEY_DELAY);
  }

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(green_mask, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

  // Make sure at least one contour was detected
  if (contours.empty())
  {
    return;
  }

  // Determine the largest contour
  std::vector<cv::Point> *largest_contour;
  int longest_contour_length = 0;
  for (auto &contour : contours)
  {
    if (longest_contour_length < contour.size())
    {
      largest_contour = &contour;
    }
  }

  // Find moments of the image and determine centroid
  cv::Moments m = cv::moments(*largest_contour);
  cv::Point_<float> marker_centre_2d(m.m10 / m.m00, m.m01 / m.m00);

  // Determine the 3D position of the marker centre from depth map
  cv::Point3d marker_centre_3d;
  bool ret = create_cloud_point(img_depth->image, marker_centre_3d, msg_camera_info->k, marker_centre_2d);

  if (!ret) {
    return;
  }

  // Publish the pupil centres
  marker_centre.point.x = marker_centre_3d.x;
  marker_centre.point.y = marker_centre_3d.y;
  marker_centre.point.z = marker_centre_3d.z;
  pub_marker_centre_->publish(marker_centre);
}

bool SimpleMarkerDetector::create_cloud_point(
    const cv::Mat_<uint16_t> &img_depth,
    cv::Point3_<double> &output,
    const std::array<double, 9> &camera_matrix,
    cv::Point_<float> &pixel,
    const double depth_scale)
{
  if (pixel.x >= img_depth.cols || pixel.y >= img_depth.rows)
  {
    RCLCPP_ERROR(this->get_logger(), "create_cloud_point() - Pixel out of bounds");
    return false;
  }

  // Get subpixel value of the depth at floating point pixel coordinate
  cv::Mat subpixel_patch;
  cv::remap(img_depth, subpixel_patch, cv::Mat(1, 1, CV_32FC2, &pixel), cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);
  uint16_t z = subpixel_patch.at<uint16_t>(0, 0);

  if (z == 0)
  {
    return false;
  }
  else
  {
    output.z = z * depth_scale;
    output.x = output.z * ((pixel.x - camera_matrix[2]) / camera_matrix[0]);
    output.y = output.z * ((pixel.y - camera_matrix[5]) / camera_matrix[4]);
    return true;
  }
}

////////////
/// MAIN ///
////////////

/// Main function that initiates an object of `SimpleMarkerDetector` class as the core of this node.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleMarkerDetector>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
