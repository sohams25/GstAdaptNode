#ifndef GST_ADAPT_NODE__MEDIA_STREAMER_NODE_HPP_
#define GST_ADAPT_NODE__MEDIA_STREAMER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>

namespace gst_adapt_node
{

class MediaStreamerNode : public rclcpp::Node
{
public:
  explicit MediaStreamerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
  cv::Mat frame_yuv_;
  sensor_msgs::msg::CameraInfo info_msg_;
  std::string video_path_;
  std::string format_;
  double fps_;
  bool loop_;
  uint64_t frame_id_ = 0;

  void on_timer();
  void init_camera_info(int width, int height);
};

}  // namespace gst_adapt_node

#endif  // GST_ADAPT_NODE__MEDIA_STREAMER_NODE_HPP_
