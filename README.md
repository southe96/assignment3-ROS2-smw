# hikrobot_camera

基于海康 MVS SDK 的 ROS 2 相机驱动：按序列号或 IP 打开相机，把图像发布到 `/image_raw`（`sensor_msgs/msg/Image`），曝光、增益、帧率、像素格式可以在运行中修改，相机断线后会自动重连并恢复参数。

## 环境要求

| 项目 | 版本 |
|---|---|
| 系统 | Ubuntu 22.04 x86_64 |
| ROS | ROS 2 Humble |
| 相机 SDK | 海康 MVS 5.1.0（SDK V4.8.2.2），安装在 `/opt/MVS` |

## 仓库结构

```text
assignment3-ROS2-smw/                 # 仓库根目录，同时是 colcon 工作空间
├── README.md
├── AGENTS.md
├── docs/                             # 培训提供的 ROS 2 教程和作业要求
└── src/hikrobot_camera/              # 相机功能包
    ├── package.xml                   # ROS 依赖
    ├── CMakeLists.txt                # 查找 MVS SDK、编译、安装
    ├── include/hikrobot_camera/camera_node.hpp
    ├── src/
    │   ├── main.cpp                  # 程序入口
    │   ├── camera_node.cpp           # 节点构造 / 析构
    │   ├── camera_device.cpp         # 枚举、选择、打开 / 关闭相机
    │   ├── camera_grab.cpp           # 取图、发布、断线重连
    │   └── camera_params.cpp         # 相机参数读写、帧率日志
    ├── launch/camera.launch.py       # 启动文件
    └── config/camera.yaml            # 参数默认值
```

## 安装与依赖

**1. MVS SDK**（厂商 SDK，不能用 rosdep 安装）

从 [海康机器人下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0) 下载 MVS 的 Linux x86_64 版本，解压后执行 `sudo ./setup.sh`，默认安装到 `/opt/MVS`。如果装在其他目录，编译前设置 `export MVCAM_SDK_PATH=<安装目录>`。

**2. ROS 依赖**

```bash
source /opt/ros/humble/setup.zsh      # bash 用 setup.bash
sudo rosdep init                       # 只在从没初始化过 rosdep 时执行
rosdep update
cd ~/assignment3-ROS2-smw
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## 编译与运行

下面的命令按 zsh 写，使用 bash 时把 `.zsh` 换成 `.bash`。

### 编译

```bash
cd ~/assignment3-ROS2-smw
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select hikrobot_camera
```

最后显示 `Summary: 1 package finished` 就说明编译成功。修改 `.cpp`、`.hpp` 或 `CMakeLists.txt` 后要重新编译；只改 `camera.yaml` 不用重新编译。

### 运行

**第 1 步：准备相机**

- 关闭 MVS 客户端，否则相机会被占用。
- 用 USB3 线接上相机。
- 只接一台相机时不用改配置。接了多台时，在 `src/hikrobot_camera/config/camera.yaml` 里填写 `serial_number`，序列号要加引号，例如 `"00F26632041"`。

**第 2 步：终端 1，启动节点**

```bash
cd ~/assignment3-ROS2-smw
source install/setup.zsh
ros2 launch hikrobot_camera camera.launch.py
```

正常情况下，终端会打印 SDK 版本和相机打开的信息，之后每 5 秒打印一行：

```text
帧率：设置 不限制 | 相机实际 ResultingFrameRate 88.3 fps | 节点发布 88.2 fps
```

**第 3 步：终端 2，查看图像**

```bash
rviz2
```

在 RViz2 左下角点 **Add** → **By topic** → `/image_raw` → **Image** → **OK**，就能看到实时画面。

**第 4 步：终端 3，修改参数（可选）**

```bash
cd ~/assignment3-ROS2-smw
source install/setup.zsh
ros2 param set /hikrobot_camera exposure_time 20000.0    # 调曝光，画面变亮
ros2 param set /hikrobot_camera pixel_format RGB8Packed  # 换像素格式
ros2 param get /hikrobot_camera exposure_time            # 读当前值
ros2 topic hz /image_raw                                 # 测接收帧率
```

**第 5 步：退出**

在终端 1 按 Ctrl+C。节点会依次停止取流、关闭相机、释放 SDK，最后显示 `process has finished cleanly`。

### 常见问题

| 现象 | 原因和解决方法 |
|---|---|
| `Package 'hikrobot_camera' not found` | 这个终端没有 source，先执行 `source install/setup.zsh` |
| 日志提示相机被占用（`0x80000203`） | MVS 客户端或另一个节点还开着，关掉后节点会自动重试 |
| `Node not found` 或 `ros2 param` 卡住 | 节点没在运行，或者 ros2 daemon 还是旧的：执行 `ros2 daemon stop` 后重试 |
| `ros2 param set` 提示类型不对 | double 参数要写小数点，写 `20000.0`，不要写 `20000` |
| RViz2 画面是黑白的 | 当前是 Mono8 或 Bayer 格式，改成 `YUV422_YUYV_Packed` 或 `RGB8Packed` |

## 相机信息

  本驱动不限定相机型号，适用于 MVS SDK 支持的 USB3 / GigE 相机：参数范围在运行时向相机查询，分辨率取自每一帧，换相机只需修改 `serial_number`（只接一台时留空即可）。下表是测试时使用的相机：

  | 项目 | 值 |
  |---|---|
  | 型号 | MV-CA016-10UC（彩色，USB3） |
  | 序列号 | 00F26632041 |
  | 默认分辨率 | 1440 × 1080 |
  | 曝光范围 | 15 ～ 9999723 μs |
  | 增益范围 | 0 ～ 17.0166 dB |


## 参数

参数写在 `config/camera.yaml` 中。

| 参数 | 类型 | 默认值 | 运行中可改 | 说明 |
|---|---|---|---|---|
| `serial_number` | string | `""` | 否 | 相机序列号 |
| `ip_address` | string | `""` | 否 | 网口相机的 IP，USB 相机留空 |
| `image_topic` | string | `image_raw` | 否 | 图像话题名 |
| `frame_id` | string | `camera_optical_frame` | 否 | 图像的 `header.frame_id` |
| `reconnect_interval` | double | `1.0` | 否 | 打开失败或断线后的重试间隔，单位 s |
| `exposure_time` | double | `10000.0` | 是 | 曝光时间，单位 μs，设置时自动关闭自动曝光 |
| `gain` | double | `0.0` | 是 | 增益，单位 dB，设置时自动关闭自动增益 |
| `frame_rate` | double | `0.0` | 是 | 帧率上限，单位 fps，`0` 表示不限制 |
| `pixel_format` | string | `YUV422_YUYV_Packed` | 是 | 可选 `Mono8` `BayerRG8` `BayerGR8` `BayerGB8` `BayerBG8` `RGB8Packed` `BGR8Packed` `YUV422_YUYV_Packed` `YUV422_Packed`，还要求相机支持 |

- **范围检查**：设置前先向相机查询范围，超出范围、相机不支持或 SDK 报错时拒绝修改，返回原因，参数值和相机状态都不变。
- **参数和相机一致**：启动或重连时，如果 yaml 里的值不能用在当前相机上，节点会打印 WARN，并把参数改成相机的实际值。
- **设置帧率不等于实际帧率**：实际帧率还受曝光和 USB 带宽限制，以日志里的 `ResultingFrameRate` 为准。

选择相机的规则：

| serial_number | ip_address | 行为 |
|---|---|---|
| 空 | 空 | 只有一台相机时直接打开；有多台时报错 |
| 填了 | 空 | 打开序列号匹配的相机，找不到时报错并列出所有相机 |
| 空 | 填了 | 打开 IP 匹配的网口相机 |
| 填了 | 填了 | 两者必须是同一台相机，否则报标识冲突 |

## 图像消息

话题 `/image_raw`，类型 `sensor_msgs/msg/Image`，QoS 为 Reliable、KeepLast(5)。

- `header.stamp`：节点收到这一帧的时间
- `width` / `height`：图像实际宽高
- `step`：一行的字节数，等于 `width × 每像素字节数`
- `data`：长度为 `step × height`

| pixel_format | encoding | 每像素字节数 |
|---|---|---|
| Mono8 | mono8 | 1 |
| BayerRG8 / GR8 / GB8 / BG8 | bayer_rggb8 / bayer_grbg8 / bayer_gbrg8 / bayer_bggr8 | 1 |
| RGB8Packed / BGR8Packed | rgb8 / bgr8 | 3 |
| YUV422_YUYV_Packed | yuv422_yuy2 | 2 |
| YUV422_Packed | yuv422 | 2 |

## 实现要点

- **独立取图线程**：取图线程负责打开相机、取图、发布和重连，主线程负责参数回调和帧率日志，互不阻塞。两个线程用互斥锁保护相机句柄。
- **断线重连**：SDK 的断线回调只设置一个标志，由取图线程关闭旧句柄，再重新打开相机，并把当前参数写回相机。连续 10 次取图失败也按断线处理。
- **资源释放**：退出时按“停止线程 → StopGrabbing → CloseDevice → DestroyHandle → Finalize”的顺序释放。

## 帧率测试

测试条件：1440 × 1080，`frame_rate` 为 0（不限制）。

| pixel_format | 每帧大小 | 相机实际帧率 | 节点发布帧率 |
|---|---|---|---|
| YUV422_YUYV_Packed（默认） | 3.1 MB | 88.3 fps | 88.2 fps |
| RGB8Packed | 4.7 MB | 80.1 fps | 80.0 fps |
| BayerRG8 | 1.6 MB | 165.9 fps | 166.0 fps |

在默认格式下用 `ros2 topic hz /image_raw` 测，结果约 54 fps（不开 RViz2 为 53.4 fps，开着 RViz2 为 54.7 fps）。

- 曝光设成 5000 μs 或 10000 μs 时，帧率都是 88.3 fps，说明这时限制帧率的不是曝光，而是数据传输和相机内部处理，每帧越小，帧率越高。
- 节点发布帧率和相机实际帧率一致，说明节点没有丢帧。`ros2 topic hz` 是 Python 写的，处理 3 MB 的图像时跟不上，所以测出来偏低，这不代表节点的发布能力。

## 已知问题

- 不支持 10 位 / 12 位像素格式（如 Mono10、BayerRG12）。
- Bayer 格式在 RViz2 里需要去马赛克才能显示彩色。
- 相机断开期间不能修改参数，要等重连完成。
