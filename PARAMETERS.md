# 接口与参数

本文是公开部署版唯一的接口与参数权威说明。默认值来自以下文件：

- 控制器：`src/controllers/template_ros2_controller/config/template_ros2_controller_parameters.yaml`
- bringup：`src/middlewares/template_middleware/launch/template_bring_up.launch.py`
- ros2_control：`src/middlewares/template_middleware/config/wheelbipe_V14.yaml`
- Keyboard：`src/tools/keyboard_teleop/config/keyboard_teleop_params.yaml`
- Xbox：`src/tools/xbox_teleop/config/xbox_teleop_params.yaml`
- MuJoCo 硬件模型：`src/resources/robot_descriptions/wheelbipe_V14/xacro/ros2control.xacro`
- 真机通信：`src/interfaces/real_bridge/include/real_bridge/real_msg.hpp`

## 1. 固定策略合同

| 项目 | 值 |
| --- | --- |
| 模型 | `V14-35-flat-and-rotation-13k.onnx` |
| 输入 | 单 tensor：`obs`，`float32[1,35]` |
| 输出 | 单 tensor：`actions`，`float32[1,6]` |
| 控制器频率 | 500 Hz |
| 推理频率 | 50 Hz |
| 默认输出模式 | `hardware_pd_vel` |

ONNX Runtime 启动时严格检查单输入输出、名称、类型和静态 shape。覆盖模型也必须遵守同一合同。

## 2. 35D 观测索引

0–27 沿用基础观测的顺序与缩放；28–34 是固定 normal-only 控制块，不由 topic、参数或 FSM 改写。

| 索引 | 含义 | 缩放 / 固定值 |
| ---: | --- | --- |
| 0 | 前进速度指令 `linear.x` | × 1.0 |
| 1 | 保留的横向速度指令 | 固定 0，× 1.0 |
| 2 | 偏航角速度指令 `angular.z` | × 1.0 |
| 3 | 目标高度 | × 5.0 |
| 4–6 | 机体角速度 x/y/z | 各 × 0.5 |
| 7–9 | 机体系投影重力 x/y/z | 各 × 1.0 |
| 10–13 | 四个腿关节位置相对默认位置 | 各 × 1.0 |
| 14–15 | 两个轮关节位置槽 | 固定 0 |
| 16–19 | 四个腿关节速度 | 各 × 0.1 |
| 20–21 | 两个轮关节速度 | 各 × 0.1 |
| 22–27 | 上一周期 6 维策略动作 | 各 × 1.0 |
| 28 | `normal` | 固定 1 |
| 29 | `stair` | 固定 0 |
| 30 | `slope` | 固定 0 |
| 31 | `recover` | 固定 0 |
| 32 | `jump` | 固定 0 |
| 33 | `height_target` 模式量 | 固定 0 |
| 34 | `state_time` 模式量 | 固定 0 |

因此每次推理的尾部恒为：

```text
[normal, stair, slope, recover, jump, height_target, state_time]
[  1.0,   0.0,   0.0,     0.0,  0.0,          0.0,        0.0]
```

基础观测默认 clamp 均为 `[-100, 100]`。观测噪声和延迟默认关闭；开启后仅作用于传感器观测，不改变 normal-only 尾部。

## 3. 6D 动作与关节映射

八关节固定顺序：

```text
0 left_front1_joint     4 left_wheel_joint
1 left_rear1_joint      5 right_wheel_joint
2 right_front1_joint    6 left_spring2_joint
3 right_rear1_joint     7 right_spring2_joint
```

| 动作 | 目标 | `joint_action_scale` | 默认 Kp | 默认 Kd | 输出限幅 |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0–3 | 四个腿关节位置目标 | 0.5 | 60 | 2 | ±50.9 Nm |
| 4–5 | 左/右轮速度目标 | 10.0 | 0 | 0.2 | ±9.99 Nm |

`default_dof_pos` 和 `joint_bias` 默认全 0。两个弹簧关节不由策略动作直接驱动；它们仍保留在八关节 `ros2_control` 合同中。PREPARE 的四腿目标为 0 rad，`Kp=80`、`Kd=2`，最大插值速度 1 rad/s。

## 4. 控制器参数

### 模型与推理

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `rl_model_path` | `policy/parallel/V14-35-flat-and-rotation-13k.onnx` | 绝对路径或包 share 相对路径 |
| `rl_inference_frequency` | 50 | 推理频率，范围 1–500 Hz |
| `rl_publish_network_io` | `true` | 发布 35D 输入与 6D 输出调试 topic |
| `rl_print_inference_time` | `false` | 打印每次 CPU 推理耗时 |

### 指令与安全范围

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `motion_command_timeout_sec` | 0.5 s | 速度和高度指令超时 |
| `motion_linear_x_min/max` | -2.5 / 2.5 m/s | 前进速度范围 |
| `motion_angular_z_min/max` | -3.0 / 3.0 rad/s | 偏航角速度范围 |
| `default_command_height` | 0.22 m | 启动与空闲超时高度 |
| `command_height_min/max` | 0.20 / 0.40 m | 公开高度范围 |
| `auto_enter_rl` | `false` | INIT 后自动进入 RL；demo 默认覆盖为 true |
| `use_dt7` | `false` | 从 RealBridge 读取普通 DT7 四字段；真机 demo 默认覆盖为 true |

只消费 `Twist.linear.x` 和 `Twist.angular.z`；其他 Twist 分量不进入策略。

### 输出与滤波

| 参数 | 默认值 | 可选值 / 说明 |
| --- | --- | --- |
| `lowlevel_output_mode` | `hardware_pd_vel` | `torque`、`hardware_pd`、`hardware_pd_vel` |
| `rl_action_filter_type` | `none` | `none`、`moving_avg`、`lowpass` |
| `rl_action_filter_window` | 3 | 移动平均窗口 |
| `rl_action_filter_alpha` | 0.8 | 低通系数 `[0,1]` |

### 观测扰动

| 参数 | 默认值 |
| --- | --- |
| `enable_noise` | `false` |
| `imu_gyro_noise_stddev` | `[0.01, 0.01, 0.01]` |
| `imu_accel_noise_stddev` | `[0.01, 0.01, 0.01]` |
| `joint_position_noise_stddev` | `0.01` |
| `joint_velocity_noise_stddev` | `0.5` |
| `enable_delay` | `false` |
| `observation_delay_steps` | `0`，范围 0–20 个 500 Hz 控制周期 |

## 5. ROS topic 与状态机

默认 namespace 是 `/wheelbipe_V14`。

| 相对 topic | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `motion_command` | `geometry_msgs/msg/Twist` | 输入 | `linear.x`、`angular.z` |
| `height_command` | `std_msgs/msg/Float64` | 输入 | 目标高度，clamp 到 0.20–0.40 m |
| `state_command` | `std_msgs/msg/Int32` | 输入 | FSM 状态切换 |
| `current_state` | `std_msgs/msg/Int32` | 输出 | 当前 FSM 状态 |
| `joint_commands` | `sensor_msgs/msg/JointState` | 输出 | 关节目标调试信息 |
| `joint_final_torque` | `std_msgs/msg/Float64MultiArray` | 输出 | 最终八关节力矩 |
| `template_ros2_controller/rl_network_input` | `std_msgs/msg/Float64MultiArray` | 输出 | 35D 策略输入 |
| `template_ros2_controller/rl_network_output` | `std_msgs/msg/Float64MultiArray` | 输出 | 6D 策略输出 |

状态 ID：`0=INIT`、`1=IDLE`、`2=PREPARE`、`3=RL`。normal-only 是策略观测合同，不增加新的状态 ID 或切换 topic。

真机 `use_dt7=true` 时，控制器还申请四个只读状态接口：`dt7/cmd_state`、`dt7/cmd_vel_x`、`dt7/cmd_omega_z`、`dt7/cmd_height`。它们分别映射 FSM、前进速度、偏航角速度和高度；速度与高度仍应用上表限幅。`cmd_state` 只接受 `0–3`，通信包不含 jump 字段。

## 6. Bringup 参数

`template_bring_up.launch.py`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `backend` | `sim` | `sim` 使用 MuJoCo；`real` 使用单串口 RealBridge |
| `prefix` | `wheelbipe_V14` | ROS namespace |
| `controller_params` | 空 | 控制器参数定义 YAML；空值使用包默认 |
| `auto_enter_rl` | `false` | INIT 后自动进入 RL |
| `xbox` | `false` | 启动 Xbox 输入节点 |
| `use_dt7` | `false` | 申请并使用 RealBridge 的普通 DT7 状态接口；仅允许 `backend=real` |
| `xbox_config` | 空 | Xbox YAML；空值使用包默认 |
| `render` | `true` | 创建 MuJoCo viewer |
| `run_duration` | `0.0` | 自动退出的仿真秒数；0 表示持续运行 |
| `real_serial_port` | `/dev/wheelbipe_h7` | 唯一 H7 串口 |
| `real_baudrate` | `2000000` | 串口波特率 |
| `real_serial_reconnect_interval_ms` | `1000` | 串口重连间隔 |
| `real_state_timeout_ms` | `100` | 状态超时后禁止发送控制包 |

`scripts/demo.sh` 提供 `--headless`、`--xbox`、`--duration SECONDS`、`--real`、`--serial-port DEVICE` 和 `--no-dt7`。仿真非 Xbox 模式自动进入 RL；真机始终关闭自动进入 RL，并默认启用 DT7。`--no-dt7` 或 `--xbox` 将真机输入切回 ROS topic。

## 7. Keyboard 参数

| 参数 | 默认值 |
| --- | --- |
| `control_mode` | 0（步进） |
| `lin_vel_x_step` / `ang_vel_z_step` | 0.5 m/s / 1.0 rad/s |
| `height_step` | 0.01 m |
| `max_lin_vel_x` / `max_ang_vel_z` | 2.5 m/s / 3.0 rad/s |
| `min_height` / `default_height` / `max_height` | 0.20 / 0.22 / 0.40 m |
| `use_rate_in_continuous_mode` | `false` |
| `lin_vel_x_rate` / `ang_vel_z_rate` | 2.0 / 2.0 每秒 |
| `prefix` | `wheelbipe_V14` |

Keyboard 以 20 Hz 发布指令；连续渐变模式内部以 50 Hz 更新。

## 8. Xbox 参数

| 参数 | 默认值 |
| --- | --- |
| `device_path` | 空，按 `device_name_hint` 自动发现 |
| `device_name_hint` | `Xbox` |
| `grab_device` | `false` |
| `publish_rate_hz` | 50 |
| `stick_deadzone` | 3000 |
| `max_linear_x` / `max_angular_z` | 2.5 m/s / 3.0 rad/s |
| `min_height` / `default_height` / `max_height` | 0.20 / 0.22 / 0.40 m |
| `height_down_rate` / `height_up_rate` | 0.2 / 0.2 m/s |
| `idle_state_id` / `rl_state_id` | 1 / 3 |

默认映射：左摇杆 Y 为前进速度，右摇杆 X 为偏航，LT/RT 为降低/升高高度，`START` 进入 RL，`SHARE/RECORD` 回到 IDLE。

## 9. 单串口与普通 DT7 合同

RealBridge 只实现一个串口，不存在 `serial_mode`、第二端口或通道切换参数。

| 项目 | 固定值 / 默认值 |
| --- | --- |
| 设备 | `/dev/wheelbipe_h7`，可由 launch/CLI 覆盖 |
| 波特率 | 2,000,000 |
| 帧头 / 帧尾 | `A8 E6` / `C3 F7` |
| CRC | CRC-16，多项式 `0x8005`，初值 `0xFFFF` |
| PC → H7 | 158 字节：时间戳、6 个关节命令和兼容保留区 |
| H7 → PC | 143 字节：时间戳、6 个关节状态、IMU、13 字节 DT7 |
| DT7 | `uint8 cmd_state` + 3 个 float：`cmd_vel_x/cmd_omega_z/cmd_height` |
| 状态超时 | 100 ms |

PC → H7 的 6 个关节固定为四腿驱动和两轮；两个弹簧关节只为上层八关节合同保留，不进入串口帧。兼容保留区在公开版恒发 0，不暴露弹簧补偿或 speed-error ROS 接口。收到首个帧头、帧尾和 CRC 均正确的状态包前不发送控制包；状态超时后停止发送。双串口、HI12M0、DT7 jump、数据录制和运行探针不属于此合同。

## 10. MuJoCo 执行器模型

四个腿驱动关节启用同一执行器动态模型：

| 参数 | 默认值 |
| --- | ---: |
| 时间常数 | 0.014 s |
| 传输延迟 | 0 s |
| DC gain | 1.0 |
| 自然频率 | 50 Hz |
| 阻尼比 | 0.10 |
| 力矩变化率上限 | 1500 Nm/s |
| 执行器输出上限 | 54 Nm |
| 速度衰减起点 | 4 rad/s |
| 速度衰减斜率 | 0.055 / (rad/s) |
| 最小速度增益 | 0.55 |

气弹簧模型启用，位置范围 `[-0.005, 0.07] m`，对应力 `650 -> 450 N`；手动粘性阻尼为 `500 N/(m/s)`。这些值属于当前 MuJoCo 模型，不因 35D normal-only 合同而改变。

## 11. Sim2Sim / Sim2Real 后端合同

后端切换不改变以下公共合同：

| 合同 | 固定要求 |
| --- | --- |
| 控制器 | `robot_locomotion/TemplateRos2Controller` |
| 策略 | `obs float32[1,35] -> actions float32[1,6]` |
| 控制周期 | 500 Hz；策略推理 50 Hz |
| 关节顺序 | 第 3 节定义的八关节顺序 |
| 状态接口 | 每关节 position / velocity / effort，外加 IMU；真机额外提供普通 DT7 四字段 |
| 命令接口 | position / velocity / effort / kp / kd |
| ROS 接口 | 第 5 节定义的 topic、类型和状态 ID |

Sim2Sim 使用 `mujoco_ros2_control/MujocoSystem`，Sim2Real 使用 `template_real_ros2_ctrl::RealBridge`。`backend:=sim|real` 或 `scripts/demo.sh [--real]` 只切换 Xacro/launch 中的 hardware plugin；35D 参数、ONNX、控制器和 ROS topic 保持不变。
