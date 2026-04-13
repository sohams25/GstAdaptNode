#ifndef GST_ADAPT_NODE__GST_ADAPT_NODE_HPP_
#define GST_ADAPT_NODE__GST_ADAPT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "gst_adapt_node/hardware_detector.hpp"

namespace gst_adapt_node
{

class GstAdaptNode : public rclcpp::Node
{
public:
  explicit GstAdaptNode(const rclcpp::NodeOptions & options);
  ~GstAdaptNode() override;

private:
  PlatformInfo platform_info_;
  std::string pipeline_string_;
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  rclcpp::TimerBase::SharedPtr bus_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  void declare_parameters();
  void detect_hardware();
  void build_pipeline();
  void launch_pipeline();
  void poll_bus();
  void shutdown_pipeline();

  void on_image(sensor_msgs::msg::Image::UniquePtr msg);
  static void destroy_ros_image(gpointer user_data);
};

}  // namespace gst_adapt_node

#endif  // GST_ADAPT_NODE__GST_ADAPT_NODE_HPP_
