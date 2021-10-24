## gst_h264

gst_h264 is similar to [gscam](https://github.com/ros-drivers/gscam), but publishes
[h264 messages](https://github.com/clydemcqueen/h264_image_transport/blob/master/h264_msgs/msg/Packet.msg)
instead of [image messages](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html).
It works with GStreamer pipelines that end with h264 buffers rather than decoded image buffers.

### Install and build

gst_h264 has been tested on Ubuntu 20.04 and ROS2 Foxy.
See the [Dockerfile](Dockerfile) for install and build instructions.

### Usage

First, make sure your h264 GStreamer pipeline runs successfully in gst-launch-1.0.
For example, here's a pipeline that works for some H.264-enabled V4L cameras:
~~~
$ gst-launch-1.0 v4l2src device=/dev/video0 ! h264parse ! decodebin ! autovideosink
~~~

Once the pipeline is working in gst-launch-1.0, remove the h264 decoder 
and the sink to get the gst_config parameter:
~~~
$ ros2 run gst_h264 gst_h264_node --ros-args -p gst_config:="v4l2src device=/dev/video0 ! h264parse"
[INFO] gst_h264_node: gst_config: v4l2src device=/dev/video0 ! h264parse
[INFO] gst_h264_node: sync_sink: 0
[INFO] gst_h264_node: camera_info_url: file:///tmp/${NAME}.yaml
[INFO] gst_h264_node: camera_name: forward_camera
[INFO] gst_h264_node: camera calibration URL: file:///tmp/forward_camera.yaml
[INFO] gst_h264_node: camera info OK
[INFO] gst_h264_node: thread running
~~~

Then watch the h264 ROS2 messages flow:
~~~
$ ros2 topic hz /image_raw/h264
average rate: 30.950
	min: 0.004s max: 0.057s std dev: 0.00694s window: 64
...
~~~

The [example launch file](launch/example_launch.py) shows how to use 
[h264_image_transport](https://github.com/clydemcqueen/h264_image_transport) to
decode the h264 messages.

### Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `gst_config` | string | v4l2src device=/dev/video0 ! h264parse | GStreamer pipeline |
| `sync_sink` | bool | False | Enable GstBaseSink synchronization |
| `camera_info_url` | string | file:///tmp/${NAME}.yaml | URL to camera info file |
| `camera_name` | string | forward_camera | Replaces `${NAME}` in the URL |

### Published topics
- `camera_info`
- `image_raw/h264`

### Troubleshooting

GStreamer is very powerful, but pipeline construction can be a bit of a challenge for GStreamer newbies. A few tips:
* you will almost certainly need an `h264parse` element in the pipeline.
* if your source is `videotestsrc` be sure to set `is-live=true` to slow the pipeline down to 30fps.
* if you use `rtph264depay` you may need to add a `capsfilter caps=video/x-h264,stream-format=byte-stream,alignment=au`
  element to generate h264 messages that `h264_image_transport` can decode.
