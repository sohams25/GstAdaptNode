#ifndef GST_ADAPT_NODE__GST_ADAPT_NODE_HPP_
#define GST_ADAPT_NODE__GST_ADAPT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include "gst_adapt_node/hardware_detector.hpp"

namespace gst_adapt_node
{

class ResizeNode : public rclcpp::Node
{
public:
  explicit ResizeNode(const rclcpp::NodeOptions & options);
  ~ResizeNode() override;

private:
  PlatformInfo platform_info_;
  std::string pipeline_string_;
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  GstElement * appsink_ = nullptr;
  rclcpp::TimerBase::SharedPtr bus_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_pub_;

  bool direct_mode_ = false;  // true = OpenCV resize, false = GStreamer pipeline
  int target_w_ = 640;
  int target_h_ = 480;

  void declare_parameters();
  void detect_hardware();
  void build_pipeline();
  void launch_pipeline();
  void launch_direct();
  void poll_bus();
  void shutdown_pipeline();

  void on_image(sensor_msgs::msg::Image::UniquePtr msg);
  void on_image_direct(sensor_msgs::msg::Image::UniquePtr msg);
  static void destroy_ros_image(gpointer user_data);
  static GstFlowReturn on_new_sample(GstAppSink * sink, gpointer user_data);
};

}  // namespace gst_adapt_node

#endif  // GST_ADAPT_NODE__GST_ADAPT_NODE_HPP_
