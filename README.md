# RoboMaster assignment3 ROS2
这份仓库提供一个基础工程，供你在 Ubuntu 22.04 / ROS 2 Humble 上，基于海康机器人 MVS SDK 完成相机功能包。

目前只有最小节点和启动配置，连接相机、发布图像、参数设置及断线重连需要你完成。目录划分仅供参考，你可以根据需要调整。

## 开始

1. 点击 GitHub 页面右上角的 **Fork**，将仓库复制到你的账号下。
2. 在你的 Fork 页面点击 **Code**，复制地址并克隆到本地：

   ```bash
   # 将下面的地址替换为你的 Fork 地址
   git clone <你的 Fork 地址>
   cd robomaster-camera-assignment
   ```

3. 阅读 [ROS 2 教程](docs/ROS2Tutorial.md) 和 [作业要求](docs/assignment.md)，按下面的步骤构建并启动工程。
4. 在自己的仓库中完成开发，提交并推送改动，最后提交你的 GitHub 仓库链接。

[AGENTS.md](AGENTS.md) 用于约束 AI 助手的帮助范围：你可以用 AI 理解概念和分析问题，核心实现需要自己完成。

## 仓库结构

```text
robomaster-camera-assignment/          # 同时作为 colcon 工作空间
├── AGENTS.md                         # AI 助教规范
├── README.md
├── docs/ROS2Tutorial.md              # ROS 2 教程
├── docs/assignment.md                # 作业要求
└── src/hikrobot_camera/              # ROS 2 功能包
    ├── package.xml                   # 包信息与依赖
    ├── CMakeLists.txt                # 构建与安装配置
    ├── include/hikrobot_camera/
    │   └── camera_node.hpp          # 节点声明
    ├── src/
    │   ├── main.cpp                 # 程序入口
    │   └── camera_node.cpp          # 在这里开始实现
    ├── launch/camera.launch.py       # 启动文件
    ├── config/camera.yaml           # 参数配置
    ├── cmake/                       # 可按需添加 SDK 查找模块
    └── test/                        # 可按需添加测试
```

## 环境与依赖

先安装 ROS 2 Humble 与开发工具，确保 `ros2`、`colcon` 和 `rosdep` 可用。

工程目前没有接入 MVS SDK。你需要从 [海康机器人下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0) 下载适合系统架构的 SDK，阅读随附文档，并完成构建集成。ROS 和系统依赖可以通过 rosdep 安装，厂商 SDK 需要单独配置。

## 编译

在新终端中进入仓库根目录，运行：

```bash
source /opt/ros/humble/setup.bash
# 仅当系统尚未初始化 rosdep 时执行一次：sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select hikrobot_camera
```

本仓库本身就是工作空间，不需要再放到另一个工作空间的 `src` 中。如果你想使用已有工作空间，也可以只把 `src/hikrobot_camera` 放进去。

## 运行

另开终端，在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hikrobot_camera camera.launch.py
```

如果你使用 Zsh，将环境脚本的 `.bash` 换为 `.zsh`。

初始工程会输出 `Training scaffold only` 并保持运行，按 Ctrl+C 退出。此时尚未实现相机功能，没有图像话题是正常的。

你也可以指定自己的参数文件：

```bash
ros2 launch hikrobot_camera camera.launch.py params_file:=/absolute/path/to/camera.yaml
```

当前 YAML 只配置了 `use_sim_time`。相机相关参数需要你在代码中声明并实现后，再加入配置文件。

## 完成与提交

从 `camera_node.cpp` 的 TODO 开始，按 [作业要求](docs/assignment.md) 完成相机功能。你可以增加源文件或 SDK 封装类，并相应更新构建配置。

完成后：

- 更新 README，说明 SDK 及依赖的安装方式、如何编译启动、有哪些可配置参数。如果有未完成的功能或已知问题，简单注明即可。
- 将源代码、Launch 和参数配置推送到你的 Fork, 然后提交仓库链接到 2719850558@qq.com，格式为：第三次作业-班级-姓名（第三次作业-自动化2305-周湛昊）


---

## 在这里解释你的项目

### 1. 功能概览

`hikrobot_camera` 是海康 MVS 工业相机的 ROS 2 Humble 节点：

- 枚举网口（GigE）和 USB3 相机，按 `serial_number` 或 `ip_address` 选择目标相机；找不到、标识冲突、被占用时打印明确的日志。
- 采集图像，发布 `sensor_msgs/msg/Image`，话题名可配置，默认 `/image_raw`。
- 曝光时间、增益、帧率、像素格式可以用 ROS 参数读取和动态修改。设置前会检查相机报告的范围，也会检查 SDK 返回值；失败时返回原因，参数保持原值。
- 断线后自动重连，重连后把参数重新写回相机；退出时按顺序释放资源。
- 每 5 秒打印一次：设置帧率、相机实际帧率（ResultingFrameRate）、节点发布帧率。

### 2. 环境与依赖

| 项目 | 版本 |
|---|---|
| 系统 | Ubuntu 22.04 x86_64 |
| ROS | ROS 2 Humble |
| 相机 SDK | 海康 MVS 5.1.0（相机 SDK 4.8.2），安装在 `/opt/MVS` |
| 测试相机 | MV-CA016-10UC（USB3），SN 00F26632041|

#### 2.1 安装 MVS SDK（不能用 rosdep 安装）

MVS 是厂商 SDK，没有 rosdep 规则，需要手动安装：

1. 到 [海康机器人下载中心](https://www.hikrobotics.com/cn/machinevision/service/download/?module=0) 下载 “MVS” 的 Linux x86_64 版本（本项目用的是 `MVS-5.1.0_Linux_x86_64`）。
2. 解压后进入目录执行 `sudo ./setup.sh`（也可以安装包里的 `.deb`：`sudo dpkg -i MVS-*.deb`）。默认安装到 `/opt/MVS`。
3. 安装程序会把环境变量写进 `~/.bashrc`。使用 zsh 时，把下面几行加到 `~/.zshrc`：

   ```bash
   export MVCAM_SDK_PATH=/opt/MVS
   export MVCAM_COMMON_RUNENV=/opt/MVS/lib
   export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
   export ALLUSERSPROFILE=/opt/MVS/MVFG
   export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
   ```

4. 检查：`ls /opt/MVS/include/MvCameraControl.h /opt/MVS/lib/64/libMvCameraControl.so` 两个文件都存在。

CMake 会按 `MVCAM_SDK_PATH`（没设置时用 `/opt/MVS`）查找 `include/MvCameraControl.h` 和 `lib/64/libMvCameraControl.so`，找不到就报错停止。SDK 装在别的目录时，设置 `MVCAM_SDK_PATH` 即可，不用改 CMakeLists.txt。安装后的可执行文件带有 RPATH，不设置 `LD_LIBRARY_PATH` 也能找到 `libMvCameraControl.so`。

#### 2.2 ROS 依赖

`package.xml` 里声明了 `rclcpp`、`rcl_interfaces`、`sensor_msgs` 等依赖，用 rosdep 安装：

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

### 3. 编译

```bash
cd ~/assignment3-ROS2-smw
source /opt/ros/humble/setup.bash       # zsh 用 setup.zsh
colcon build --symlink-install --packages-select hikrobot_camera
```

### 4. 运行

```bash
source install/setup.bash               # zsh 用 setup.zsh
ros2 launch hikrobot_camera camera.launch.py
```

- 换相机：修改 `src/hikrobot_camera/config/camera.yaml` 里的 `serial_number`，重新 `colcon build` 后启动（用了 `--symlink-install`，改 yaml 后不重新编译也会生效）。
- 用自己的参数文件：`ros2 launch hikrobot_camera camera.launch.py params_file:=/绝对路径/my_camera.yaml`
- 临时指定序列号：`ros2 run hikrobot_camera camera_node --ros-args -p serial_number:=00D36741054`
- 查看图像：`rviz2`，Add → By topic → `/image_raw` → Image；或者 `ros2 run rqt_image_view rqt_image_view`。
- 启动前要关闭 MVS 客户端，否则相机被占用（错误码 0x80000203）。

选择相机的规则：

| serial_number | ip_address | 行为 |
|---|---|---|
| 空 | 空 | 只有一台相机时直接打开它（打印 WARN）；有多台时报错，要求填写 |
| 填了 | 空 | 打开序列号匹配的相机；找不到时报错，并列出所有相机 |
| 空 | 填了 | 打开 IP 匹配的网口相机（USB 相机没有 IP） |
| 填了 | 填了 | 必须是同一台相机，否则报“标识冲突” |

打开失败时（找不到、被占用、冲突），节点每隔 `reconnect_interval` 秒重试一次，同一类错误日志每 5 秒最多打印一次。

### 5. 参数

| 参数 | 类型 | 单位 | 默认值 | 范围 / 可选值 | 运行中可改 | 说明 |
|---|---|---|---|---|---|---|
| `serial_number` | string | — | `""` | — | 否 | 相机序列号，必须加引号 |
| `ip_address` | string | — | `""` | 点分十进制 IP | 否 | 网口相机 IP |
| `image_topic` | string | — | `image_raw` | — | 否 | 图像话题名 |
| `frame_id` | string | — | `camera_optical_frame` | — | 否 | `header.frame_id` |
| `reconnect_interval` | double | s | `1.0` | > 0 | 否 | 打开失败 / 断线后的重试间隔 |
| `exposure_time` | double | μs | `10000.0` | 以相机报告为准（测试相机：15 ～ 9999723） | 是 | 设置时自动关闭 ExposureAuto |
| `gain` | double | dB | `0.0` | 以相机报告为准（测试相机：0 ～ 17.0166） | 是 | 设置时自动关闭 GainAuto |
| `frame_rate` | double | fps | `0.0` | `0` 或相机报告的范围 | 是 | `0` = 不限制帧率（AcquisitionFrameRateEnable=false）；`> 0` = 打开帧率限制并设置 AcquisitionFrameRate |
| `pixel_format` | string | — | `YUV422_YUYV_Packed` | `Mono8` `BayerRG8` `BayerGR8` `BayerGB8` `BayerBG8` `RGB8Packed` `BGR8Packed` `YUV422_YUYV_Packed` `YUV422_Packed`，且相机支持 | 是 | 修改时节点会先停止取流，改完再开始 |

设置规则和失败时的行为：

- double 类型参数要写小数点，例如 `ros2 param set /hikrobot_camera exposure_time 20000.0`。写成 `20000` 会被 ROS 以类型不匹配拒绝。
- 超出范围、相机不支持、SDK 返回错误、相机未连接时，`ros2 param set` 显示 `Setting parameter failed: <原因>`，参数值不变，相机状态也不变。
- 启动或重连时，如果 yaml 里的值不能用在当前相机上（比如这台相机不支持该像素格式），节点打印 WARN，并把参数改成相机的实际值。这样 `ros2 param get` 读到的始终是相机的真实状态。
- 设置帧率不等于实际帧率：实际帧率还受曝光时间和 USB / 网络带宽限制，看日志里的 `ResultingFrameRate`。

### 6. 话题与消息

- 话题：`/image_raw`（由 `image_topic` 决定），类型 `sensor_msgs/msg/Image`，QoS 为 Reliable、KeepLast(5)。
- `header.stamp`：节点从 SDK 拿到这一帧时的 ROS 时间（`now()`），包含曝光结束到传输完成的延迟，不是相机内部时间戳。
- `width` / `height`：SDK 帧信息里的 `nWidth` / `nHeight`（默认分辨率1440 × 1080）。
- `encoding` 和 `step`（一行的字节数 = width × 每像素字节数）：

| pixel_format | encoding | 每像素字节数 |
|---|---|---|
| Mono8 | mono8 | 1 |
| BayerRG8 / GR8 / GB8 / BG8 | bayer_rggb8 / bayer_grbg8 / bayer_gbrg8 / bayer_bggr8 | 1 |
| RGB8Packed / BGR8Packed | rgb8 / bgr8 | 3 |
| YUV422_YUYV_Packed | yuv422_yuy2 | 2 |
| YUV422_Packed | yuv422（UYVY） | 2 |

- `data`：从 SDK 缓存复制 `step × height` 个字节，复制完立即 `MV_CC_FreeImageBuffer` 归还缓存。

### 7. 实现说明

| 文件 | 内容 |
|---|---|
| `src/camera_node.cpp` | 构造函数：声明参数、创建发布者 / 定时器、初始化 SDK、启动取图线程；析构函数：停线程、释放资源 |
| `src/camera_device.cpp` | 枚举、按 SN / IP 选择、打开 / 关闭相机，断线回调，错误码转文字 |
| `src/camera_grab.cpp` | 像素格式表、开始取流、取一帧并填 Image、取图线程主循环（含重连） |
| `src/camera_params.cpp` | 参数回调、范围检查、自动曝光 / 增益关闭、像素格式停流修改、重连后恢复参数、帧率日志 |

- 线程：取图线程负责打开相机、取图、发布和重连；ROS 执行器线程负责参数回调和帧率定时器。相机句柄由 `camera_mutex_` 保护。
- 断线检测：`MV_CC_RegisterExceptionCallBack` 收到 `MV_EXCEPTION_DEV_DISCONNECT` 时只设置标志，由取图线程关闭旧句柄、重新枚举打开；另外连续 10 次取图出错也按断线处理。
- 释放顺序：停止取图线程 → `MV_CC_StopGrabbing` → `MV_CC_CloseDevice` → `MV_CC_DestroyHandle` → `MV_CC_Finalize`。

### 8. 帧率测试

测试条件：相机MV-CA016-10UC，分辨率1440 × 1080，pixel_format YUV422_YUYV_Packed，exposure_time 10000 μs，frame_rate 0（不限制）。

| 测量方式 | 结果 |
|---|---|
| 相机实际帧率（节点日志 ResultingFrameRate） | 88.3fps |
| 节点发布帧率（节点日志“节点发布”） | 88.2 fps |
| `ros2 topic hz /image_raw`（不开 RViz2） | 53.4 fps |
| `ros2 topic hz /image_raw`（开着 RViz2） | 54.7 fps |

分析：帧率不限制时，相机实际帧率由曝光时间和USB带宽共同决定。曝光10000μs意味着每帧最多100fps。而实测只有88.3 fps，说明此时限制帧率的是 USB 带宽。另外测了其他像素格式：RGB8Packed 每帧 4.7 MB，实测 80.1 fps；BayerRG8 每帧1.6MB；实测165.9 fps；YUV422每帧3.1 MB，实测88.3 fps。每帧字节数越少，帧率越高。节点发布帧率与相机实际帧率基本一致。ros2 topic hz 是 Python 写的订阅者，接收 3 MB 的大图像时跟不上，所以只有约54fps，开不开RViz2差别不大。

### 9. 已知问题与限制

- 按 IP 选择的代码已实现，但只用 USB 相机测试过，没有用网口相机实测。
- 只支持表中 8 位 / 16 位的像素格式；10 / 12 位格式（如 Mono10、BayerRG12）不支持，设置时会被拒绝。
- Bayer 格式的图像需要下游做去马赛克才能显示彩色；想在 RViz2 里直接看彩色画面，用 `YUV422_YUYV_Packed` 或 `RGB8Packed`。
- 相机断开期间设置参数会被拒绝（返回“相机未连接”），需要等重连完成后再设置。
- 图像时间戳是主机接收时间。