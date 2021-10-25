// MIT License
//
// Copyright (c) 2021 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "camera_info_manager/camera_info_manager.hpp"
#include "h264_msgs/msg/packet.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gst_h264
{

const std::string default_gst_config = "v4l2src device=/dev/video0 ! h264parse";  // NOLINT

// Copy all memory segments in a GstBuffer into dest
void copy_buffer(GstBuffer *buffer, std::vector<unsigned char> & dest)
{
  auto num_segments = static_cast<int>(gst_buffer_n_memory(buffer));
  gsize copied = 0;
  for (int i = 0; i < num_segments; ++i) {
    GstMemory *segment = gst_buffer_get_memory(buffer, i);
    GstMapInfo segment_info;
    gst_memory_map(segment, &segment_info, GST_MAP_READ);

    std::copy(segment_info.data, segment_info.data + segment_info.size,
      dest.begin() + (long) copied);
    copied += segment_info.size;

    gst_memory_unmap(segment, &segment_info);
    gst_memory_unref(segment);
  }
}

class GstH264Node : public rclcpp::Node
{
  GstElement *sink_;
  int seq_{};

  sensor_msgs::msg::CameraInfo camera_info_;

  rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr h264_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Poll GStreamer on a separate thread
  std::thread thread_;
  std::atomic<bool> stop_signal_{};

  // Parameters
  std::string gst_config_;
  bool sync_sink_{};
  std::string camera_info_url_;
  std::string camera_name_;  // Used by CameraInfoManager, will substitute for ${NAME} in URL

  // Return true to continue, false to stop
  bool process_frame()
  {
    // Poll for a frame, briefly block (timeout is in nanoseconds) -- prevents deadlock
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 1000000);

    if (!sample) {
      if (gst_app_sink_is_eos(GST_APP_SINK(sink_))) {
        RCLCPP_INFO(get_logger(), "stream ended");
        return false;
      } else {
        return true;
      }
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);

    if (!buffer) {
      return true;
    }

    camera_info_.header.stamp = now();

    if (++seq_ % 300 == 0) {
      RCLCPP_INFO(get_logger(), "%d h264 frames", seq_);
    }

    auto buffer_size = gst_buffer_get_size(buffer);

    h264_msgs::msg::Packet packet;
    packet.header = camera_info_.header;
    packet.seq = seq_;
    packet.data.resize(buffer_size);

    copy_buffer(buffer, packet.data);

    h264_pub_->publish(packet);

    if (camera_info_pub_) {
      camera_info_pub_->publish(camera_info_);
    }

    gst_sample_unref(sample);

    return true;
  }

public:

  GstH264Node():
    Node("gst_h264_node")
  {
    declare_parameter("gst_config", rclcpp::ParameterValue(default_gst_config));
    declare_parameter("sync_sink", rclcpp::ParameterValue(false));
    declare_parameter("camera_info_url", rclcpp::ParameterValue("file:///tmp/${NAME}.yaml"));
    declare_parameter("camera_name", rclcpp::ParameterValue("forward_camera"));

    get_parameter("gst_config", gst_config_);
    get_parameter("sync_sink", sync_sink_);
    get_parameter("camera_info_url", camera_info_url_);
    get_parameter("camera_name", camera_name_);

    RCLCPP_INFO(get_logger(), "gst_config: %s", gst_config_.c_str());
    RCLCPP_INFO(get_logger(), "sync_sink: %d", sync_sink_);
    RCLCPP_INFO(get_logger(), "camera_info_url: %s", camera_info_url_.c_str());
    RCLCPP_INFO(get_logger(), "camera_name: %s", camera_name_.c_str());

    if (!gst_is_initialized()) {
      gst_init(nullptr, nullptr);
    }

    GError *error = nullptr;
    GstElement *bin;
    if (!(bin = gst_parse_bin_from_description(gst_config_.c_str(), true, &error))) {
      RCLCPP_FATAL(get_logger(), "can't parse gst_config: %s", error->message);
      return;
    }

    GstElement *pipeline = gst_pipeline_new(nullptr);

    sink_ = gst_element_factory_make("appsink", nullptr);

    GstCaps *caps = gst_caps_new_simple("video/x-h264", nullptr, nullptr);
    gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
    gst_caps_unref(caps);

    gst_base_sink_set_sync(GST_BASE_SINK(sink_), sync_sink_);

    gst_bin_add_many(GST_BIN(pipeline), bin, sink_, nullptr);

    if (!gst_element_link(bin, sink_)) {
      RCLCPP_FATAL(get_logger(), "can't link gst_config and appsink, check gst_config");
      return;
    }

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_FATAL(get_logger(), "can't play stream, check gst_config");
      return;
    }

    h264_pub_ = create_publisher<h264_msgs::msg::Packet>("image_raw/h264", 10);

    camera_info_manager::CameraInfoManager camera_info_manager(this, camera_name_,
      camera_info_url_);

    if (camera_info_manager.isCalibrated()) {
      RCLCPP_INFO(get_logger(), "camera info OK");
      camera_info_ = camera_info_manager.getCameraInfo();
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    } else {
      RCLCPP_INFO(get_logger(),
        "failed to load camera info, are you missing the 'file://' prefix?");
    }

    thread_ = std::thread(
      [this]()
      {
        RCLCPP_INFO(get_logger(), "thread running");  // NOLINT

        while (!stop_signal_ && rclcpp::ok() && process_frame());

        RCLCPP_INFO(get_logger(), "thread stopped");  // NOLINT
      });
  }

  ~GstH264Node() override
  {
    if (thread_.joinable()) {
      stop_signal_ = true;
      thread_.join();
    }
  }
};

} // namespace gst_h264

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<gst_h264::GstH264Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
