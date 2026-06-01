# AGENTS.md — 21届智能车竞赛 人工智能组完全模型组

## 项目概述

第21届全国大学生智能车竞赛 — 人工智能组完全模型组。使用 C++ 编写，目标部署平台为 Orange Pi 5 开发板。

## 远端仓库

| 项目 | 值 |
|------|-----|
| 平台 | GitHub |
| 地址 | `https://github.com/liuzhengfu102/21smart_car` |
| 当前状态 | 远端已配置，因网络原因暂未推送（待网络恢复后执行 `git push -u origin main`） |

## 关键约束

- **称呼用户为 刘征孚**
- **所有回答使用中文**
- **所有文档使用中文编写**
- **每次修改后自动提交到仓库**
- **每次提交后推送到远端**（网络条件允许时）
- **持久化工程信息**：凡是开发过程中发现的、值得记录的工程信息（如板卡环境配置、依赖库版本、编译参数、目录结构、调试技巧等），必须写入本规则文件，确保后续会话可复用

## 开发工作流

1. **同步为前提**：板卡端 `car.cpp` 为主版本。本工作区镜像仅在修改时与板卡同步。任何修改前，先对比本地与板卡的文件 hash，若不一致则从板卡拉取最新版本
2. **本地修改**：代码修改在本地完成，保持频繁的 git commit 以便回退
3. **版本管理**：每次有意义的修改使用 `git commit` 记录，避免丢失历史版本
4. **推送需确认**：本地修改完成后，必须经过用户同意，再推送至板卡并尝试编译运行。严禁未经同意推送
5. **自动审查**：每次修改代码后，分析修改内容是否引入逻辑缺陷、越界风险、未初始化变量、内存泄漏等问题，确保不引入新 bug

```bash
# 同步检查：对比本地与板卡文件 MD5
ssh root@192.168.3.57 "md5sum /home/orangepi/Desktop/smartcar/car.cpp"
md5sum car.cpp
# 若不一致，拉取最新：
scp root@192.168.3.57:/home/orangepi/Desktop/smartcar/car.cpp .
```

## 板卡环境

### SSH 连接

| 项目 | 值 |
|------|-----|
| 板卡型号 | Orange Pi 5 (Rockchip RK3588) |
| 操作系统 | Orange Pi 1.2.2 Jammy (Ubuntu 22.04.5 LTS) |
| 内核 | 6.1.99-rockchip-rk3588 |
| 架构 | aarch64 / ARM64 |
| IP 地址 | `192.168.3.57` |
| SSH 用户 | `root` |
| SSH 密码 | `orangepi` |
| 内存 | ~8GB |
| 磁盘 | 29GB (SD卡 mmcblk1p1)，已用 ~65% |

```bash
# SSH 连接（非交互式可使用 paramiko/Python 脚本）
ssh root@192.168.3.57
# 密码: orangepi
```

> **注意**：SSH 用户为 `root`，不是 `orangepi5`。当前本机 SSH 公钥已添加至板卡 `~/.ssh/authorized_keys`，可直接免密连接。

### 板载工具链

| 工具 | 版本 |
|------|------|
| g++ / gcc | 11.4.0 (Ubuntu) |
| CMake | 3.22.1 |
| GNU Make | 4.3 |
| Python | 3.10.12 |
| OpenSSH | 8.9p1 |

### 已安装关键库

| 库 | 版本/路径 | 用途 |
|------|-----------|------|
| OpenCV (含 DNN) | 4.5.4 (`/usr/lib/aarch64-linux-gnu/cmake/opencv4`) | 视觉处理 |
| RKNN Runtime | `librknnrt.so`, `librknn_api.so` (位于 `/lib/`) | NPU 推理 |
| RGA | `librga.so` (位于 `/lib/aarch64-linux-gnu/`) | 硬件图像加速 |
| ROS2 Humble | cv-bridge 已安装 | ROS2 视觉桥接 |
| GStreamer | opencv 插件已安装 | 视频流处理 |

### OpenCV 编译参数（CMakeLists.txt 中使用）

```cmake
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(car ${OpenCV_LIBS} rt)
```

## 现有项目结构（板卡端）

板卡项目路径：`/home/orangepi/Desktop/smartcar/`

```
smartcar/
├── car.cpp              # 主控程序（C++）
├── CMakeLists.txt        # CMake 构建配置
├── shm_writer.py         # 共享内存测试写入脚本（Python）
├── vserial.txt           # 虚拟串口路径 (/dev/pts/8)
├── src/                  # 预留源码目录（当前为空）
├── .venv/                # Python 虚拟环境
└── build/
    ├── car               # 编译产物（可执行文件）
    ├── config.txt        # 配置文件（内容: 3）
    ├── zhong.txt         # 中线参数文件（内容: 39）
    ├── CMakeCache.txt    # CMake 缓存
    └── Makefile          # 自动生成
```

本地仓库已同步源代码文件：`car.cpp`、`CMakeLists.txt`、`shm_writer.py`、`build/config.txt`、`build/zhong.txt`。

## 构建与部署

### 在板卡上编译

```bash
ssh root@192.168.3.57 "cd /home/orangepi/Desktop/smartcar/build && cmake .. && make"
```

### 传输文件到板卡

```bash
scp <本地文件> root@192.168.3.57:/home/orangepi/Desktop/smartcar/
```

### 一次性编译+传输+部署

```bash
scp car.cpp CMakeLists.txt root@192.168.3.57:/home/orangepi/Desktop/smartcar/ && \
ssh root@192.168.3.57 "cd /home/orangepi/Desktop/smartcar/build && cmake .. && make"
```

## 系统架构

### 数据流

```
[AI 视觉推理] → 共享内存 (/shm_road_coords) → [car.cpp 控制程序] → 串口 (/dev/ttyS3) → [底层驱动]
```

- AI 推理模块（Python，待确认）将**赛道中线点**和**检测目标**写入共享内存
- C++ 控制程序读取共享内存，计算转向角度和速度，通过串口发送指令

### 共享内存协议（`/shm_road_coords`，4096 字节）

| 偏移 | 类型 | 字段 | 说明 |
|------|------|------|------|
| 0 | uint32 | road_num | 赛道中线点数 |
| 4 | float ×2 ×N | road_points | 每个点 (x, y) 坐标 |
| ... | uint32 | obj_num | 检测目标数量 |
| ... | uint32 + float×4 | objects | 每个目标 (c, x1, y1, x2, y2) |

目标类别 c：0=金币, 1=车, 2=行人

### 串口通信协议（8 字节帧，115200 bps）

| 字节 | 含义 |
|------|------|
| 0 | 帧头 0xAA |
| 1 | 帧头 0x55 |
| 2 | 目标速度 |
| 3 | 转向中位值 高8位 |
| 4 | 转向中位值 低8位 |
| 5 | 目标中位值 高8位 |
| 6 | 目标中位值 低8位 |
| 7 | 帧尾 0xFF |

串口设备：`/dev/ttyS3`（测试时可用虚拟串口 `/dev/pts/8`）

### 控制逻辑要点（car.cpp）

1. **赛道中线**：取所有道路点 x 坐标平均值作为基准 `base_road_x`
2. **障碍物识别**：检测到车(1)或人(2)时，根据目标在左/右侧调整转向偏移 ±160
3. **金币追踪**：检测到金币(0)时，计算偏离量并用 EMA 低通滤波平滑追踪
4. **防闪烁**：障碍物消失后维持状态 0.5 秒防止误判
5. **信号处理**：Ctrl+C 触发 `SIGINT`，发送停车指令并关闭串口
6. **滤波参数**：金币追踪 α=0.15，最终输出 α=0.3

## 开发注意事项

- 板卡为 ARM64 架构，本地 x86 编译的二进制无法在板卡上运行
- 板卡 IP 可能变化，连接失败时需确认 IP 地址（当前 `192.168.3.57`）
- 板卡仅有 SD 卡存储，注意磁盘空间（已用 65%）
- 当前 USB 设备未连接摄像头（lsusb 仅显示 root hub）——开发时需确认摄像头连接状态
- **禁止在代码或提交中硬编码密码等敏感信息**——以上密码仅用于 SSH/SCP 运维操作
- 本机通过 paramiko (Python) 可实现非交互式 SSH 操作，免密密钥已部署
- **`cv::` 命名空间陷阱**：`getTickCount()` 和 `getTickFrequency()` 必须在 `cv::` 命名空间下调用。重构时如移除 `using namespace cv;`，需为这两个函数添加 `cv::` 前缀，否则板卡端 g++ 11.4.0 编译失败
