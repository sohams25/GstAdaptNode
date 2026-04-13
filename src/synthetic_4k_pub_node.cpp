#include "gst_adapt_node/synthetic_4k_pub_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace gst_adapt_node
{

Synthetic4kPubNode::Synthetic4kPubNode(const rclcpp::NodeOptions & options)
: Node("synthetic_4k_pub",
       rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  cx_(WIDTH / 2), cy_(HEIGHT / 2), dx_(15), dy_(10)
{
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", 10);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);

  init_background();
  init_camera_info();

  auto period = std::chrono::duration<double>(1.0 / FPS);
  timer_ = create_wall_timer(period, std::bind(&Synthetic4kPubNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "Publishing %dx%d @ %.0f Hz on /camera/image_raw (C++ zero-copy)",
    WIDTH, HEIGHT, FPS);
}

void Synthetic4kPubNode::init_background()
{
  bg_frame_ = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
  for (int y = 0; y < HEIGHT; ++y) {
    auto * row = bg_frame_.ptr<cv::Vec3b>(y);
    for (int x = 0; x < WIDTH; ++x) {
      row[x][0] = static_cast<uint8_t>(40 + (x * 40) / WIDTH);
    }
  }
  frame_ = bg_frame_.clone();
}

void Synthetic4kPubNode::init_camera_info()
{
  info_msg_.width = WIDTH;
  info_msg_.height = HEIGHT;
  info_msg_.distortion_model = "plumb_bob";
  info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  double fx = static_cast<double>(WIDTH);
  double fy = fx;
  double cx = fx / 2.0;
  double cy = static_cast<double>(HEIGHT) / 2.0;

  info_msg_.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
  info_msg_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info_msg_.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
}

void Synthetic4kPubNode::restore_region(cv::Rect roi)
{
  roi &= cv::Rect(0, 0, WIDTH, HEIGHT);
  bg_frame_(roi).copyTo(frame_(roi));
}

void Synthetic4kPubNode::on_timer()
{
  // Erase previous circle + text by restoring from clean background
  int margin = RADIUS + 4;
  restore_region(cv::Rect(cx_ - margin, cy_ - margin, margin * 2, margin * 2));
  restore_region(cv::Rect(50, 60, 450, 80));

  // Advance bouncing circle
  cx_ += dx_;
  cy_ += dy_;
  if (cx_ - RADIUS <= 0 || cx_ + RADIUS >= WIDTH) { dx_ = -dx_; }
  if (cy_ - RADIUS <= 0 || cy_ + RADIUS >= HEIGHT) { dy_ = -dy_; }

  // Draw circle + frame counter
  cv::circle(frame_, {cx_, cy_}, RADIUS, {0, 255, 0}, cv::FILLED);
  cv::putText(frame_, std::to_string(frame_id_), {60, 120},
    cv::FONT_HERSHEY_SIMPLEX, 3.0, {255, 255, 255}, 4, cv::LINE_AA);

  // Build Image message via unique_ptr for zero-copy IPC
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  cv_bridge::CvImage cv_img(std_msgs::msg::Header(), "bgr8", frame_);
  cv_img.toImageMsg(*msg);

  auto stamp = now();
  msg->header.stamp = stamp;
  msg->header.frame_id = "synthetic_4k";

  // unique_ptr publish — enables zero-copy when intra-process comms are active
  image_pub_->publish(std::move(msg));

  // CameraInfo with matching stamp
  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(info_msg_);
  info->header.stamp = stamp;
  info->header.frame_id = "synthetic_4k";
  info_pub_->publish(std::move(info));

  ++frame_id_;
}

}  // namespace gst_adapt_node

RCLCPP_COMPONENTS_REGISTER_NODE(gst_adapt_node::Synthetic4kPubNode)
