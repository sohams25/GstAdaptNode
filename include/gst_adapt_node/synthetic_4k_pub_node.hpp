#ifndef GST_ADAPT_NODE__SYNTHETIC_4K_PUB_NODE_HPP_
#define GST_ADAPT_NODE__SYNTHETIC_4K_PUB_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>

namespace gst_adapt_node
{

class Synthetic4kPubNode : public rclcpp::Node
{
public:
  explicit Synthetic4kPubNode(const rclcpp::NodeOptions & options);

private:
  static constexpr int WIDTH = 3840;
  static constexpr int HEIGHT = 2160;
  static constexpr double FPS = 30.0;
  static constexpr int RADIUS = 80;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Mat frame_;
  cv::Mat bg_frame_;
  sensor_msgs::msg::CameraInfo info_msg_;

  int cx_, cy_, dx_, dy_;
  uint64_t frame_id_ = 0;

  void on_timer();
  void init_background();
  void init_camera_info();
  void restore_region(cv::Rect roi);
};

}  // namespace gst_adapt_node

#endif  // GST_ADAPT_NODE__SYNTHETIC_4K_PUB_NODE_HPP_
