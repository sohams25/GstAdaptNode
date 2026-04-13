#include "gst_adapt_node/media_streamer_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>

namespace gst_adapt_node
{

MediaStreamerNode::MediaStreamerNode(const rclcpp::NodeOptions & options)
: Node("media_streamer",
       rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  declare_parameter("video_path", "/tmp/test_video.mp4");
  declare_parameter("loop", true);
  declare_parameter("format", "bgr8");
  declare_parameter("image_topic", "/camera/image_raw");
  declare_parameter("info_topic", "/camera/camera_info");

  video_path_ = get_parameter("video_path").as_string();
  loop_ = get_parameter("loop").as_bool();
  format_ = get_parameter("format").as_string();

  cap_.open(video_path_);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Cannot open video: %s", video_path_.c_str());
    return;
  }

  int w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
  int h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
  fps_ = cap_.get(cv::CAP_PROP_FPS);
  if (fps_ <= 0) { fps_ = 30.0; }

  auto image_topic = get_parameter("image_topic").as_string();
  auto info_topic = get_parameter("info_topic").as_string();
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);

  init_camera_info(w, h);

  auto period = std::chrono::duration<double>(1.0 / fps_);
  timer_ = create_wall_timer(period, std::bind(&MediaStreamerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "Streaming %dx%d @ %.1f Hz [%s] from %s (C++ zero-copy)",
    w, h, fps_, format_.c_str(), video_path_.c_str());
}

void MediaStreamerNode::init_camera_info(int width, int height)
{
  info_msg_.width = width;
  info_msg_.height = height;
  info_msg_.distortion_model = "plumb_bob";
  info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  double fx = static_cast<double>(width);
  double fy = fx;
  double cx = fx / 2.0;
  double cy = static_cast<double>(height) / 2.0;

  info_msg_.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
  info_msg_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info_msg_.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
}

void MediaStreamerNode::on_timer()
{
  if (!cap_.isOpened()) { return; }

  if (!cap_.read(frame_)) {
    if (loop_) {
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!cap_.read(frame_)) { return; }
    } else {
      RCLCPP_INFO(get_logger(), "End of video");
      timer_->cancel();
      return;
    }
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();

  if (format_ == "yuv420p") {
    cv::cvtColor(frame_, frame_yuv_, cv::COLOR_BGR2YUV_I420);
    int w = frame_.cols;
    int h = frame_.rows;
    size_t yuv_size = static_cast<size_t>(w) * h * 3 / 2;

    msg->height = h;
    msg->width = w;
    msg->encoding = "yuv420p";
    msg->step = w;
    msg->is_bigendian = false;
    msg->data.assign(frame_yuv_.data, frame_yuv_.data + yuv_size);
  } else {
    cv_bridge::CvImage cv_img(std_msgs::msg::Header(), "bgr8", frame_);
    cv_img.toImageMsg(*msg);
  }

  auto stamp = now();
  msg->header.stamp = stamp;
  msg->header.frame_id = "media_frame";
  image_pub_->publish(std::move(msg));

  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(info_msg_);
  info->header.stamp = stamp;
  info->header.frame_id = "media_frame";
  info_pub_->publish(std::move(info));

  ++frame_id_;
}

}  // namespace gst_adapt_node

RCLCPP_COMPONENTS_REGISTER_NODE(gst_adapt_node::MediaStreamerNode)
