#include "hikrobot_camera/camera_node.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "MvCameraControl.h"

namespace hikrobot_camera
{

namespace
{
// 这 4 个参数需要写到相机上，其他参数（话题名等）不需要
const char * const kCameraParameters[] = {"pixel_format", "exposure_time", "gain", "frame_rate"};

bool isCameraParameter(const std::string & name)
{
  for (const char * camera_param : kCameraParameters) {
    if (name == camera_param) {
      return true;
    }
  }
  return false;
}

std::string toText(double value)
{
  char text[64];
  std::snprintf(text, sizeof(text), "%.3f", value);
  return text;
}
}  // namespace

rcl_interfaces::msg::SetParametersResult CameraNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 节点自己在把"相机实际值"同步回参数，不需要再写一遍相机
  if (syncing_parameters_) {
    return result;
  }

  for (const auto & param : params) {
    if (!isCameraParameter(param.get_name())) {
      continue;
    }
    std::string error;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (handle_ == nullptr) {
        error = "相机未连接，参数没有修改";
      } else {
        error = applyCameraParameter(param);
      }
    }
    if (!error.empty()) {
      // successful = false 时，ROS 不会修改这个参数，ros2 param set 会显示 reason
      result.successful = false;
      result.reason = param.get_name() + ": " + error;
      RCLCPP_WARN(get_logger(), "设置失败 %s", result.reason.c_str());
      return result;
    }
    RCLCPP_INFO(
      get_logger(), "参数 %s 已设置为 %s", param.get_name().c_str(),
      param.value_to_string().c_str());
  }
  return result;
}

// 调用前必须已经锁住 camera_mutex_，并且 handle_ 不是空的
std::string CameraNode::applyCameraParameter(const rclcpp::Parameter & param)
{
  const std::string & name = param.get_name();
  if (name == "exposure_time") {
    return setExposureTime(param.as_double());
  }
  if (name == "gain") {
    return setGain(param.as_double());
  }
  if (name == "frame_rate") {
    return setFrameRate(param.as_double());
  }
  if (name == "pixel_format") {
    return setPixelFormat(param.as_string());
  }
  return "";
}

// 先向相机读出范围，超出范围直接拒绝；在范围内才真正设置。成功返回空字符串
std::string CameraNode::setFloatInRange(const char * key, double value)
{
  MVCC_FLOATVALUE range;
  std::memset(&range, 0, sizeof(range));
  int ret = MV_CC_GetFloatValue(handle_, key, &range);
  if (ret != MV_OK) {
    return std::string("读取 ") + key + " 失败：" + sdkErrorToString(ret);
  }
  if (value < range.fMin || value > range.fMax) {
    return toText(value) + " 超出相机支持的范围 [" + toText(range.fMin) + ", " +
           toText(range.fMax) + "]";
  }
  ret = MV_CC_SetFloatValue(handle_, key, static_cast<float>(value));
  if (ret != MV_OK) {
    return std::string("设置 ") + key + " 失败：" + sdkErrorToString(ret);
  }
  return "";
}

std::string CameraNode::setExposureTime(double value)
{
  // 手动设置曝光前先关掉自动曝光（0 = Off），否则会设置失败或被自动曝光改掉
  const int ret = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
  if (ret != MV_OK) {
    RCLCPP_WARN(get_logger(), "关闭自动曝光失败：%s", sdkErrorToString(ret).c_str());
  }
  return setFloatInRange("ExposureTime", value);
}

std::string CameraNode::setGain(double value)
{
  // 同理，先关掉自动增益
  const int ret = MV_CC_SetEnumValue(handle_, "GainAuto", 0);
  if (ret != MV_OK) {
    RCLCPP_WARN(get_logger(), "关闭自动增益失败：%s", sdkErrorToString(ret).c_str());
  }
  return setFloatInRange("Gain", value);
}

std::string CameraNode::setFrameRate(double value)
{
  // value <= 0：关闭帧率限制，相机按曝光和带宽允许的最快速度出图
  if (value <= 0.0) {
    const int ret = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false);
    if (ret != MV_OK) {
      return "关闭帧率限制失败：" + sdkErrorToString(ret);
    }
    return "";
  }

  // value > 0：先检查范围，再打开帧率限制并设置
  MVCC_FLOATVALUE range;
  std::memset(&range, 0, sizeof(range));
  int ret = MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &range);
  if (ret != MV_OK) {
    return "读取 AcquisitionFrameRate 失败：" + sdkErrorToString(ret);
  }
  if (value < range.fMin || value > range.fMax) {
    return toText(value) + " 超出相机支持的范围 [" + toText(range.fMin) + ", " +
           toText(range.fMax) + "]，或者填 0 表示不限制";
  }
  ret = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
  if (ret != MV_OK) {
    return "打开帧率限制失败：" + sdkErrorToString(ret);
  }
  ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(value));
  if (ret != MV_OK) {
    return "设置 AcquisitionFrameRate 失败：" + sdkErrorToString(ret);
  }
  return "";
}

std::string CameraNode::setPixelFormat(const std::string & value)
{
  // 1. 必须是本节点支持的格式
  const PixelFormatInfo * format = findPixelFormatByName(value);
  if (format == nullptr) {
    return "本节点不支持 '" + value + "'，可选：" + supportedPixelFormatNames();
  }

  // 2. 必须是这台相机支持的格式
  MVCC_ENUMVALUE current;
  std::memset(&current, 0, sizeof(current));
  int ret = MV_CC_GetEnumValue(handle_, "PixelFormat", &current);
  if (ret != MV_OK) {
    return "读取 PixelFormat 失败：" + sdkErrorToString(ret);
  }
  bool camera_supports = false;
  for (unsigned int i = 0; i < current.nSupportedNum && i < MV_MAX_XML_SYMBOLIC_NUM; ++i) {
    if (current.nSupportValue[i] == format->sdk_value) {
      camera_supports = true;
    }
  }
  if (!camera_supports) {
    return "这台相机不支持 " + value;
  }
  if (current.nCurValue == format->sdk_value) {
    return "";  // 已经是这个格式
  }

  // 3. PixelFormat 只能在停止取流时修改：先停，改完再开
  const bool was_grabbing = grabbing_;
  if (was_grabbing) {
    MV_CC_StopGrabbing(handle_);
    grabbing_ = false;
  }
  ret = MV_CC_SetEnumValue(handle_, "PixelFormat", format->sdk_value);
  if (was_grabbing) {
    const int start_ret = MV_CC_StartGrabbing(handle_);
    grabbing_ = (start_ret == MV_OK);
    if (!grabbing_) {
      RCLCPP_ERROR(
        get_logger(), "改完像素格式后重新开始取流失败：%s", sdkErrorToString(start_ret).c_str());
    }
  }
  if (ret != MV_OK) {
    return "设置 PixelFormat 失败：" + sdkErrorToString(ret);
  }
  return "";
}

// 读出相机上这个参数的实际值。调用前必须已经锁住 camera_mutex_
bool CameraNode::readCameraParameter(const std::string & name, rclcpp::Parameter & out)
{
  if (name == "exposure_time" || name == "gain") {
    MVCC_FLOATVALUE value;
    std::memset(&value, 0, sizeof(value));
    const char * key = (name == "gain") ? "Gain" : "ExposureTime";
    if (MV_CC_GetFloatValue(handle_, key, &value) != MV_OK) {
      return false;
    }
    out = rclcpp::Parameter(name, static_cast<double>(value.fCurValue));
    return true;
  }
  if (name == "frame_rate") {
    bool enabled = false;
    if (MV_CC_GetBoolValue(handle_, "AcquisitionFrameRateEnable", &enabled) != MV_OK) {
      return false;
    }
    if (!enabled) {
      out = rclcpp::Parameter(name, 0.0);
      return true;
    }
    MVCC_FLOATVALUE value;
    std::memset(&value, 0, sizeof(value));
    if (MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &value) != MV_OK) {
      return false;
    }
    out = rclcpp::Parameter(name, static_cast<double>(value.fCurValue));
    return true;
  }
  if (name == "pixel_format") {
    MVCC_ENUMVALUE value;
    std::memset(&value, 0, sizeof(value));
    if (MV_CC_GetEnumValue(handle_, "PixelFormat", &value) != MV_OK) {
      return false;
    }
    const PixelFormatInfo * format = findPixelFormatByValue(value.nCurValue);
    if (format == nullptr) {
      return false;
    }
    out = rclcpp::Parameter(name, std::string(format->name));
    return true;
  }
  return false;
}

// 相机刚打开（启动或重连）、还没开始取流时调用：把 ROS 参数的值全部写到相机上
void CameraNode::applyAllCameraParameters()
{
  // 先在锁外读出参数值（ROS 参数有自己的锁，两把锁交叉会死锁）
  std::vector<rclcpp::Parameter> params;
  for (const char * name : kCameraParameters) {
    params.push_back(get_parameter(name));
  }

  std::vector<rclcpp::Parameter> corrections;
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (handle_ == nullptr) {
      return;
    }
    for (const auto & param : params) {
      const std::string error = applyCameraParameter(param);
      if (error.empty()) {
        RCLCPP_INFO(
          get_logger(), "相机参数 %s = %s", param.get_name().c_str(),
          param.value_to_string().c_str());
        continue;
      }
      RCLCPP_WARN(
        get_logger(), "参数 %s = %s 无法用在这台相机上：%s", param.get_name().c_str(),
        param.value_to_string().c_str(), error.c_str());
      rclcpp::Parameter actual;
      if (readCameraParameter(param.get_name(), actual)) {
        corrections.push_back(actual);
      }
    }
  }

  // 设置失败的参数，改成相机的实际值，保证 ros2 param get 看到的就是相机真实状态
  syncing_parameters_ = true;
  for (const auto & actual : corrections) {
    set_parameter(actual);
    RCLCPP_WARN(
      get_logger(), "参数 %s 已改为相机当前值 %s", actual.get_name().c_str(),
      actual.value_to_string().c_str());
  }
  syncing_parameters_ = false;
}

void CameraNode::reportFrameRate()
{
  // 节点发布帧率 = 这段时间发布的帧数 / 时间
  const rclcpp::Time current = now();
  const double seconds = (current - last_fps_time_).seconds();
  last_fps_time_ = current;
  const double published_fps =
    seconds > 0.0 ? static_cast<double>(published_frames_.exchange(0)) / seconds : 0.0;

  // 相机端：设置帧率（AcquisitionFrameRate）和实际帧率（ResultingFrameRate）
  bool limit_enabled = false;
  MVCC_FLOATVALUE set_rate;
  MVCC_FLOATVALUE resulting_rate;
  std::memset(&set_rate, 0, sizeof(set_rate));
  std::memset(&resulting_rate, 0, sizeof(resulting_rate));
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (handle_ != nullptr) {
      ok = MV_CC_GetBoolValue(handle_, "AcquisitionFrameRateEnable", &limit_enabled) == MV_OK &&
        MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &set_rate) == MV_OK &&
        MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &resulting_rate) == MV_OK;
    }
  }
  if (!ok) {
    RCLCPP_INFO(get_logger(), "帧率：节点发布 %.1f fps（相机未连接）", published_fps);
    return;
  }
  const std::string set_text = limit_enabled ? toText(set_rate.fCurValue) + " fps" : "不限制";
  RCLCPP_INFO(
    get_logger(), "帧率：设置 %s | 相机实际 ResultingFrameRate %.1f fps | 节点发布 %.1f fps",
    set_text.c_str(), resulting_rate.fCurValue, published_fps);
}

}  // namespace hikrobot_camera
