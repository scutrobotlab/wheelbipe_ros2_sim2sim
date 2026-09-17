# 200 mm 越障复现（2026-09-16）

已删除原场景的 3 个矮障碍；台阶、平台顶面为 0.200 m，平台后为 17° 下坡。
推荐启动使用下面的独立入口。它选择 `source_v14` 仿真配置、2.7 m 直线助跑距离、
默认高度命令 0.40 m，仍由原生 ROS2 controller + MuJoCo bridge 执行。

## 启动

```bash
cd /home/gx/wheelbipe_ros2_sim2sim
source /home/gx/miniconda3/etc/profile.d/conda.sh
conda activate wheelbipe_humble
./scripts/demo_step200.sh
```

等机器人进入 RL（约 3–5 秒），点击仿真窗口，按 ↑ 将窗口速度档调至 2.5 m/s，再按住 W
直线接近台阶，默认高度为 0.40 m。松键后速度命令清零；W/S 前后，A/D 转向，
T/G 调高度，F 切换跟随，空格暂停，Backspace 重置。鼠标左拖旋转、右拖平移、滚轮缩放。
历史键盘阶梯验收由独立脚本发送每 0.2 秒增加 0.5 m/s 的命令。
如需旧的外部键盘节点，先使用 `./scripts/demo_step200.sh --no-viewer-keyboard`
关闭窗口命令发布，避免两个输入源互相覆盖。

## 权重与验收条件

源模型：
`src/controllers/template_ros2_controller/policy/parallel/Source-V14-35-rough-dash-3500.onnx`

对应服务器 `/home/gjuser/wheeled-legged_RL/pretrained/26_infantry/rough_dash/2026-07-19_19-15-07/model_3500.pt`。
本机与服务器 checkpoint/ONNX SHA 相同。ONNX SHA-256：
`aa8a339e1dac43b7e7fca3769e65ed18300e1d0c61eb91a5efedfe0ed266cdff`。
128 个随机 35D 输入的 checkpoint/ONNX 最大绝对差为 5.7220459e-6。

- 7 次恒定 2.5 m/s、0.40 m 高度命令全部通过，其中 3 次在最终 task-keyframe 启动实现下复测。
- 另 1 次键盘速度阶梯命令通过。
- 台阶前缘 x=1.20 m，起点 x=-1.50 m；平台末端 x=2.50 m；台阶宽 1.80 m。
- 判定要求两轮 x>1.72 m、轮心 z>0.245 m、两轮 |y|<0.75 m 且保持直立，持续至少 0.20 s；
  之后两轮 x>2.60 m 且仍在通道内。运动开始至试验结束 upright 必须一直 >0.5。
- `final_dash3500_v25_h40_rep*.json/csv` 与 `final_dash3500_keyboard_ramp.json/csv` 是最终配置记录。
  `final_dash3500_v25_h40_rep1.mp4` 是原生 ROS2 状态轨迹离线渲染，不是另一个简化动力学 rollout。

## 修复与边界

1. 七月 `rough_rotation_stair` 小台阶验收不能代表 `rough_dash` 的 200 mm 能力；九月 19999 步权重也不能仅凭迭代数认为更强。
2. 源 USD instance proxies 中的 16 个导向轮圆柱、4 个前连杆 box、云台 yaw 圆柱此前丢失；UniLab 静态 MJCF 已恢复共 33 个碰撞形状及对应 touch volumes。
3. `source_v14` 显式使用源质量/惯量/armature、40 Nm 腿力矩、5 Nm 轮力矩、无部署电机响应滤波、50 N s/m 弹簧阻尼。
   弹簧力为 `400 + (200 / 0.07) * max(0.06076 - q, 0)`，已在 1001 个行程点与桥接公式核对。
4. MuJoCo 单摩擦通道取源动态摩擦范围中点：base 0.055、wheel 0.7、guide 0.4。它不是 PhysX 静/动摩擦逐轨迹等价。
5. ROS2 仍使用 normal-only 35D/6D 控制；没有把源 Airborne/StepUp 的地形传感器状态伪装为已部署，也不声称所有速度/高度/方向均能通过。
6. 原 `deployment` 配置与用户原机器人 XML 保留；普通 `demo.sh` 保持原配置选择。新增 `demo_step200.sh` 负责选择此验收配置。

`sourcephysics_*`、`sourcefriction_*` 是调试阶段的中间配置（含曾写错并已修正的弹簧映射），不能引用为最终 profile 的验证结果。
源九月权重、七月小台阶 UniLab 权重以及默认学习率的短程 fine-tune 有失败案例，保留 JSON，不作为推荐模型。

UniLab 完整 gate：2357 passed、78 skipped、279 deselected、1 xfailed；类型检查与 benchmark smoke 通过。
训练实验及进一步模型验证记录位于 `/home/gx/UniLab/logs/wheelbipe_step200_20260916/`。

补充：fixed 1e-5 的 200 轮 UniLab warm start 同样没有通过本次原生 200 mm 验收，
因此也没有替换启动默认源模型。源策略可用与继续训练的能力保持是两个独立结论。
