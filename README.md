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

