#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace hikrobot_camera
{

// 一种像素格式：参数里写的名字、SDK 里的数值、ROS 的 encoding、每个像素占几个字节
struct PixelFormatInfo
{
  const char * name;
  unsigned int sdk_value;
  const char * encoding;
  unsigned int bytes_per_pixel;
};

// 在支持的像素格式表里查找（定义在 camera_grab.cpp）
const PixelFormatInfo * findPixelFormatByName(const std::string & name);
const PixelFormatInfo * findPixelFormatByValue(unsigned int sdk_value);
std::string supportedPixelFormatNames();

// 把 SDK 返回值变成 "0x80000203（...）" 这样的文字（定义在 camera_device.cpp）
std::string sdkErrorToString(int ret);

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  // 取一帧的结果：拿到图像 / 这次没有新图像 / 出错
  enum class GrabResult { kFrame, kNoFrame, kError };

  // ---- 阶段 5：选择、打开、关闭相机（camera_device.cpp）----
  bool openCamera();
  void closeCamera();

  // ---- 阶段 6：取图并发布（camera_grab.cpp）----
  bool startGrabbing();
  GrabResult grabOnce(sensor_msgs::msg::Image & msg);
  void grabLoop();

  // ---- 阶段 7：相机参数（camera_params.cpp）----
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);
  std::string applyCameraParameter(const rclcpp::Parameter & param);
  std::string setFloatInRange(const char * key, double value);
  std::string setExposureTime(double value);
  std::string setGain(double value);
  std::string setFrameRate(double value);
  std::string setPixelFormat(const std::string & value);
  bool readCameraParameter(const std::string & name, rclcpp::Parameter & out);
  void applyAllCameraParameters();
  void reportFrameRate();

  // 启动时读一次的参数
  std::string serial_number_;
  std::string ip_address_;
  std::string frame_id_;

  // 相机句柄和取流状态：多个线程都会用到，读写前先锁 camera_mutex_
  std::mutex camera_mutex_;
  void * handle_ = nullptr;
  bool grabbing_ = false;
  bool sdk_initialized_ = false;
  int last_device_count_ = -1;

  // 取图线程
  std::thread grab_thread_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> published_frames_{0};

  // 参数
  std::atomic<bool> syncing_parameters_{false};
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ROS 发布者和定时器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr fps_timer_;
  rclcpp::Time last_fps_time_;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
