# Third-party and asset notices

The repository license in [LICENSE](LICENSE) applies to original project work and
the SCUTRobotLab assets explicitly identified below. It does not replace licenses
and copyright notices that apply to third-party components.

Pinned binary dependency versions, official archive URLs and archive SHA-256 values are recorded in [third_party/dependencies.lock](third_party/dependencies.lock).

## MuJoCo 3.5.0

- Project: https://github.com/google-deepmind/mujoco
- Version used by the bootstrap script: 3.5.0
- License: Apache License 2.0
- Local runtime directory: `.deps/mujoco-3.5.0` (not tracked by Git)

## ONNX Runtime 1.20.0

- Project: https://github.com/microsoft/onnxruntime
- Version used by the bootstrap script: 1.20.0 CPU, Linux x64
- License: MIT License
- Local runtime directory: `.deps/onnxruntime-linux-x64-1.20.0` (not tracked by Git)

## mujoco_ros2_control components

`src/interfaces/mujoco_bridge` is based on the following exact upstream tree:

- Source: https://github.com/sangteak601/mujoco_ros2_control
- Source revision: [`8ddb0b624e6f2546d368990ea178f66feb1bc2ff`](https://github.com/sangteak601/mujoco_ros2_control/tree/8ddb0b624e6f2546d368990ea178f66feb1bc2ff)
- License: MIT License

Attribution was verified by SHA-256 against that revision. The package metadata, plugin XML, rendering/control-node implementations and most public headers were identified from that source revision. The current package contains project-specific changes to its build, MuJoCo system, node and rendering code, together with WheelBipe ros2_control integration work.

Applicable source files under `src/interfaces/mujoco_bridge` retain their MIT License notice and Sangtaek Lee copyright attribution. Those notices must remain with copies and substantial portions of the software. The source repository is itself presented as a fork in the `moveit/mujoco_ros2_control` lineage.

## MIT Biomimetics Robotics Lab utilities

The following headers contain an MIT License notice and copyright © 2019 MIT Biomimetics Robotics Lab:

- `src/controllers/template_ros2_controller/include/utils/MathUtilities.h`
- `src/controllers/template_ros2_controller/include/utils/orientation_tools.h`

The license notice in each source file applies to that file.

`src/controllers/template_ros2_controller/include/utils/cppTypes.h` is a
reduced copy of the fixed-size Eigen type aliases from Cheetah-Software. It
contains an MIT License notice and copyright © 2019 MIT Biomimetic Robotics
Lab, matching the upstream license text. Its notice applies to that file.

## SCUTRobotLab model, policy and mesh assets

The WheelBipe model uses one canonical set of 21 STL files under:

```text
src/resources/robot_descriptions/wheelbipeV14_2/meshes/
```

Both `wheelbipe_V14/xacro/` and `wheelbipeV14_2/mjcf/` reference that set. The repository also contains:

```text
src/controllers/template_ros2_controller/policy/parallel/
└── V14-35-flat-and-rotation-13k.onnx
```

SCUTRobotLab authorizes these 21 STL files and the listed ONNX policy artifact for
release and redistribution under the MIT License:

- Licensor: [SCUTRobotLab](https://www.scutbot.cn/)
- License: [MIT License](LICENSE)
- Copyright: © 2025-2026 SCUTRobotLab

The directory-level notices are stored beside the assets in
[`policy/ASSET_LICENSE.md`](src/controllers/template_ros2_controller/policy/ASSET_LICENSE.md)
and
[`meshes/ASSET_LICENSE.md`](src/resources/robot_descriptions/wheelbipeV14_2/meshes/ASSET_LICENSE.md).
The ONNX policy comes from the public
[`SCUTRobotLab/wheeled-legged_RL`](https://github.com/SCUTRobotLab/wheeled-legged_RL)
training repository at commit
`9797db9ae12ce7bb04e4a1fbf37dbd8af2c989a3`. Its SHA-256 is
`a1244761f7ede02f8c80d076d4315a25f014df43df3f7f0d20c2ca5bcd518719`.
This deployment repository keeps one normal-mode example to make the public
sim2sim path concise; the training framework itself is also open source.
