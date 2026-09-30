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

struct PixelFormatInfo
{
  const char * name;
  unsigned int sdk_value;
  const char * encoding;
  unsigned int bytes_per_pixel;
};

const PixelFormatInfo * findPixelFormatByName(const std::string & name);
const PixelFormatInfo * findPixelFormatByValue(unsigned int sdk_value);
std::string supportedPixelFormatNames();
std::string sdkErrorToString(int ret);

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  enum class GrabResult { kFrame, kNoFrame, kError };
  bool openCamera();
  void closeCamera();
  static void onException(unsigned int msg_type, void * user);

  bool startGrabbing();
  GrabResult grabOnce(sensor_msgs::msg::Image & msg);
  void grabLoop();
  void sleepWhileRunning(double seconds);
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
  std::string serial_number_;
  std::string ip_address_;
  std::string frame_id_;
  double reconnect_interval_ = 1.0;
  std::mutex camera_mutex_;
  void * handle_ = nullptr;
  bool grabbing_ = false;
  bool sdk_initialized_ = false;
  int last_device_count_ = -1;
  std::thread grab_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> disconnected_{false};
  std::atomic<uint64_t> published_frames_{0};
  std::atomic<bool> syncing_parameters_{false};
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr fps_timer_;
  rclcpp::Time last_fps_time_;
};

}

#endif
