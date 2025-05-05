## hebust 2025 AT 电赛校赛视觉上位机

### Depend on

- [`OpenCV`](https://opencv.org/):用于图像处理
- [`yaml-cpp`](https://github.com/jbeder/yaml-cpp):用于解析 YAML 配置
- [`cxxopts`](https://github.com/jarro2783/cxxopts):用于解析命令行参数
- `C++20`：使用现代 C++ 特性
- `CMake`：跨平台构建系统

---

### 🛠️ 环境依赖

### Linux 配置环境

```zsh
sudo apt update
sudo apt install -y \
  libyaml-cpp-dev \
  libopencv-dev \
  cmake \
  build-essential
```

### Build it

```zsh
cmake -B build
cd build
make
```

构建产物在根目录的 bin/ 文件夹下

### how2use

```zsh
Usage:
  vision [OPTION...]

  -s, --source arg     Source file path (default: 
                       /home/sxs/Desktop/raspberry_pi/vision)
      --debug          Enable debug mode
  -l, --log-level arg  Log level (info/warn/error) (default: info)
  -h, --help           Print usage
```

#### USB摄像头

## 📷 USB 摄像头使用与调试指南

适用于 `/dev/video2` 摄像头设备，基于 `v4l2-ctl` 工具。

------

### ✅ 1. 安装必要工具

```sh
sudo apt update
sudo apt install v4l-utils ffmpeg
```

------

### ✅ 2. 识别摄像头挂载在哪个 video 设备

```sh
v4l2-ctl --list-devices
```

示例输出：

```sh
HP True Vision FHD Camera:
    /dev/video0
Integrated_Webcam_HD:
    /dev/video2
```

------

### ✅ 3. 列出支持的视频格式 + 分辨率 + 帧率

```sh
v4l2-ctl -d /dev/video2 --list-formats-ext
```

查找你想要的帧率所对应的分辨率与格式。

------

### ✅ 4. 设置分辨率 + 帧率（例如 640×480 @ 30FPS）

```sh
--set-fmt-video=width=640,height=480,pixelformat=YUYV \
--set-parm=30
```

------

### ✅ 5. 曝光设置（手动控制）

```sh
# 关闭自动曝光
v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1

# 设置具体曝光值（单位通常为 100 微秒）
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_time_absolute=120
```

------

### ✅ 6. 白平衡设置（手动控制）

```sh
# 关闭自动白平衡
v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_automatic=0

# 设置色温（单位 Kelvin）
v4l2-ctl -d /dev/video2 --set-ctrl=white_balance_temperature=5000
```

------

### ✅ 7. 实时测试视频流（可选）

```sh
ffplay /dev/video2
```

> 可实时观察曝光/帧率设置是否生效。

------

### ✅ 8. 查看当前设置状态

```sh
# 分辨率与格式
v4l2-ctl -d /dev/video2 --get-fmt-video

# 当前帧率
v4l2-ctl -d /dev/video2 --get-parm

# 所有控制项值
v4l2-ctl -d /dev/video2 --list-ctrls
```

### 一键配置

```sh
v4l2-ctl -d /dev/video0 \
  --set-ctrl=auto_exposure=1 \
  --set-ctrl=exposure_time_absolute=10 \
  --set-ctrl=gain=1 \
  --set-ctrl=white_balance_automatic=0 \
  --set-ctrl=white_balance_temperature=4500

```

powered by PraySky 2025 4/19 21:00

