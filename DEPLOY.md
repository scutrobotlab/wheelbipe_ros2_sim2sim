# 部署说明

本文覆盖 WheelBipe V14 normal-only 的 Sim2Sim / Sim2Real 部署：依赖、构建、MuJoCo、单串口真机、Keyboard/Xbox/DT7、模型覆盖和故障处理。接口与参数定义统一以 [PARAMETERS.md](PARAMETERS.md) 为准。

## 1. 环境要求

- Ubuntu 22.04 x86_64
- ROS 2 Humble Desktop 或等价的 Humble 开发环境
- `colcon`、CMake、C++17 编译器、`rosdep`、`libglfw3-dev`
- MuJoCo 3.5.0
- ONNX Runtime 1.20.0 CPU（Linux x86_64）

先安装 ROS 2 Humble，并初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

如果系统已经初始化过 rosdep，可忽略 `sudo rosdep init` 的重复初始化提示。

## 2. 准备依赖

### 在线安装

在仓库根目录执行：

```bash
./scripts/bootstrap.sh
```

脚本会校验并准备：

- `.deps/mujoco-3.5.0`
- `.deps/onnxruntime-linux-x64-1.20.0`
- ROS 与系统依赖

下载使用官方 release，并校验固定 SHA-256。

### 使用本地已解压目录

```bash
./scripts/bootstrap.sh \
  --mujoco-source /path/to/mujoco-3.5.0 \
  --onnxruntime-source /path/to/onnxruntime-linux-x64-1.20.0
```

### 离线安装

离线缓存目录需要包含以下官方压缩包：

```text
mujoco-3.5.0-linux-x86_64.tar.gz
onnxruntime-linux-x64-1.20.0.tgz
```

执行：

```bash
./scripts/bootstrap.sh --offline-cache /path/to/archive-cache
```

如果 ROS 依赖已经预装且离线环境不能访问 apt 源：

```bash
./scripts/bootstrap.sh --offline-cache /path/to/archive-cache --skip-rosdep
```

随时可以单独检查环境：

```bash
./scripts/check_env.sh
```

## 3. 构建

```bash
./scripts/build.sh
```

脚本固定传入 `BUILD_TESTING=OFF`。清理 CMake 缓存后重建：

```bash
./scripts/build.sh --clean-cache
```

手动执行 ROS 命令前加载工作空间和固定 ROS domain：

```bash
source setup_ros_domain.bash
```

## 4. 运行

### MuJoCo GUI

```bash
./scripts/demo.sh
```

脚本默认自动进入 RL，使用仓库内置 35D normal-only 模型。

### MuJoCo Headless

```bash
./scripts/demo.sh --headless
```

设置自动退出时间，单位为仿真秒：

```bash
./scripts/demo.sh --headless --duration 10
```

### Keyboard

终端 A 启动仿真：

```bash
./scripts/demo.sh
```

终端 B 启动键盘节点；该节点需要直接访问终端输入：

```bash
source setup_ros_domain.bash
ros2 run keyboard_teleop keyboard_teleop_node --ros-args \
  --params-file src/tools/keyboard_teleop/config/keyboard_teleop_params.yaml
```

按键：

| 按键 | 功能 |
| --- | --- |
| `0/1/2/3` | INIT / IDLE / PREPARE / RL |
| `w/s` | 增加 / 减少前进速度 |
| `a/d` | 增加 / 减少偏航角速度 |
| `t/g` | 增加 / 减少高度 |
| `Space` | 速度归零 |
| `r` | 高度恢复 0.22 m |
| `x` | 退出键盘节点 |

### Xbox

确认 Linux evdev 能识别手柄：

```bash
./scripts/check_xbox.sh
```

然后运行：

```bash
./scripts/demo.sh --xbox
```

Xbox 模式不会自动进入 RL：`START` 进入 RL，`SHARE/RECORD` 回到 IDLE；左摇杆 Y 控制前进速度，右摇杆 X 控制偏航，LT/RT 降低/升高目标高度。

如果自动发现失败，在 `src/tools/xbox_teleop/config/xbox_teleop_params.yaml` 中把 `device_path` 设为稳定的 `/dev/input/by-id/...-event-joystick`。出现权限错误时，将当前用户加入 `input` 组后重新登录：

```bash
sudo usermod -aG input "$USER"
```

### 单串口真机

公开版 RealBridge 只支持一个 H7 串口和普通 DT7 包。推荐安装稳定设备名并授予 `dialout` 组权限：

```bash
sudo install -m 0644 config/99-wheelbipe-serial.rules /etc/udev/rules.d/99-wheelbipe-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

重新插拔串口并重新登录，然后执行：

```bash
./scripts/check_real.sh
./scripts/demo.sh --real
```

`--real` 是一键后端切换：它启动同一控制器和 ONNX，但将 MuJoCo plugin 替换为 RealBridge。真机不会自动进入 RL，默认从 DT7 读取 `cmd_state/cmd_vel_x/cmd_omega_z/cmd_height`；状态只接受 `0–3`，没有 jump 字段或 jump 行为。

未安装 udev 规则时可临时指定设备：

```bash
./scripts/demo.sh --real --serial-port /dev/ttyUSB0
```

改用 Keyboard 或 Xbox：

```bash
./scripts/demo.sh --real --no-dt7
./scripts/demo.sh --real --xbox
```

启动真机前必须架空车轮、确保急停可触达并确认下位机自身看门狗和限位有效。RealBridge 在收到首个 CRC 正确的状态包前不发送控制包，状态超时后也禁止继续发送，但这些软件检查不能替代硬件急停、电流/温度/电压保护或机械限位。

## 5. 覆盖 ONNX 模型

默认模型：

```text
src/controllers/template_ros2_controller/policy/parallel/V14-35-flat-and-rotation-13k.onnx
```

临时覆盖：

```bash
export WHEELBIPE_RL_MODEL_PATH=/absolute/path/to/policy.onnx
./scripts/demo.sh --headless
```

覆盖模型仍必须严格满足以下合同：

- 单输入，名称 `obs`，类型 `float32`，shape `[1,35]`
- 单输出，名称 `actions`，类型 `float32`，shape `[1,6]`

名称、类型、数量或 shape 不匹配时，控制器拒绝进入推理并执行安全退出，不提供旧策略维度兼容。

## 6. 常见故障

| 现象 | 处理 |
| --- | --- |
| `Workspace is not built` | 执行 `./scripts/build.sh` |
| 找不到 MuJoCo / ONNX Runtime | 执行 `./scripts/bootstrap.sh` 或检查 `.deps` 目录 |
| ONNX `shape/type/name mismatch` | 使用严格的 `obs [1,35] -> actions [1,6]` float32 模型 |
| GUI 无法创建窗口 | 检查图形环境，或使用 `--headless` |
| 控制器未进入 RL | 查看 `current_state`；键盘按 `3`，Xbox 按 `START` |
| 指令很快归零 | 默认命令超时为 0.5 s，输入节点需要持续发布 |
| Xbox 找不到设备 | 运行 `./scripts/check_xbox.sh`，检查 `device_path` 和 input 组权限 |
| ROS 节点互不可见 | 每个终端都执行 `source setup_ros_domain.bash` |
| `/dev/wheelbipe_h7` 不存在 | 安装 udev 规则并重新插拔，或用 `--serial-port` 指定实际设备 |
| 串口 `Permission denied` | 将用户加入 `dialout` 组并重新登录 |
| `H7 state packet timed out` | 检查单串口、2,000,000 baud、143 字节状态包和 CRC；输出会被禁止 |
| DT7 不生效 | 真机默认启用；若使用了 `--no-dt7`/`--xbox`，输入源已切为 ROS topic |

调试网络输入和输出：

```bash
source setup_ros_domain.bash
ros2 topic echo /wheelbipe_V14/template_ros2_controller/rl_network_input
ros2 topic echo /wheelbipe_V14/template_ros2_controller/rl_network_output
```

## 7. Sim2Sim / Sim2Real 一键切换

本项目把策略控制器与硬件后端分开：ONNX、35D 观测、6D 动作、FSM、输入设备和 ROS topic 位于不变的上层，Sim2Sim / Sim2Real 的切换点只有 `ros2_control` hardware plugin。

| 层级 | Sim2Sim | Sim2Real | 是否需要改策略控制器 |
| --- | --- | --- | --- |
| 策略与 FSM | 本仓库 normal-only 控制器 | 同一控制器 | 否 |
| 指令输入 | Keyboard / Xbox / topic | 普通 DT7，或 Keyboard / Xbox / topic | 否 |
| 状态与命令合同 | 八关节 + IMU | 八关节 + IMU + DT7 状态 | 否 |
| hardware plugin | `mujoco_ros2_control/MujocoSystem` | `template_real_ros2_ctrl::RealBridge` | 仅切换此层 |

两个后端已经收敛为同一个入口：`./scripts/demo.sh` 与 `./scripts/demo.sh --real`。切换不需要重新训练、重新导出 ONNX 或复制控制器代码。

真机公开范围固定为：单串口、2,000,000 baud、6 个通信关节、下位机 IMU、普通 DT7 四字段，以及 158 字节发送包 / 143 字节接收包。双串口、HI12M0、DT7 jump、录制和实验探针不属于本仓库接口。

适配其他硬件时只替换 `hardware_interface::SystemInterface`，并保持八关节顺序、接口单位/符号、IMU 语义、500 Hz 控制周期和 35D/6D 策略合同。新硬件仍必须独立完成急停、下位机通信看门狗、关节限位、热/电流/电压保护、传感器标定和失联降级。
