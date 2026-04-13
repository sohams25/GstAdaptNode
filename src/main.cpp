#include <rclcpp/rclcpp.hpp>

#include "gst_adapt_node/gst_adapt_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<gst_adapt_node::GstAdaptNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
