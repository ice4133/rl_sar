# rl_sar

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![macOS](https://img.shields.io/badge/macOS-Experimental-orange.svg?logo=apple)](https://www.apple.com/macos/)
[![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)
[![ROS2 Foxy/Humble](https://img.shields.io/badge/ros2-foxy/humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-lightgrey.svg?logo=gazebo)](http://gazebosim.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2.7-orange.svg?logo=mujoco)](https://mujoco.org/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

[English document](README.md)

本仓库提供了机器人强化学习算法的仿真验证与实物部署框架，适配四足机器人、轮足机器人、人形机器人。"sar"代表"simulation and real"

> 支持**IsaacGym**和**IsaacSim**
>
> 支持**ROS-Noetic**和**ROS2-Foxy/Humble**
>
> 支持**libtorch**和**onnxruntime**
>
> 支持**Linux**和**macOS**(只支持Mujoco仿真)
>
> 支持**Gazebo**和**Mujoco**(部分支持)
>
> 支持**Locomotion**和**Dance**

支持列表：

|Robot Name (rname:=)|Pre-Trained Policy|Gazebo|Mujoco|Real|
|-|-|-|-|-|
|Unitree-A1 (a1)|legged_gym (IsaacGym)|✅|❌|✅|
|Unitree-Go2 (go2)|himloco (IsaacGym)</br>robot_lab (IsaacSim)|✅|✅|✅</br>✅|
|Unitree-Go2W (go2w)|robot_lab (IsaacSim)|✅|✅|✅|
|Unitree-B2 (b2)|robot_lab (IsaacSim)|✅|✅|⚪|
|Unitree-B2W (b2w)|robot_lab (IsaacSim)|✅|✅|⚪|
|Unitree-G1 (g1)|robomimic/locomotion (IsaacGym)</br>robomimic/charleston (IsaacGym)</br>whole_body_tracking/dance_102 (IsaacSim)</br>whole_body_tracking/gangnam_style (IsaacSim)|✅|✅|✅|
|FFTAI-GR1T1 (gr1t1)</br>(Only available on Ubuntu20.04)|legged_gym (IsaacGym)|✅|❌|⚪|
|FFTAI-GR1T2 (gr1t2)</br>(Only available on Ubuntu20.04)|legged_gym (IsaacGym)|✅|❌|⚪|
|zhinao-L4W4 (l4w4)|legged_gym (IsaacGym)|✅|❌|✅|
|Deeprobotics-Lite3 (lite3)|himloco (IsaacGym)|✅|❌|✅|
|Agibot-D1 (d1)|robot_lab (IsaacSim)|✅|✅|✅|
|DDTRobot-Tita (tita)|robot_lab (IsaacSim)|✅|❌|⚪|

> [!IMPORTANT]
> Python版本暂时停止维护，如有需要请使用[v2.3](https://github.com/fan-ziqi/rl_sar/releases/tag/v2.3)版本，后续可能会重新上线。

> [!NOTE]
> 如果你想使用IsaacLab（IsaacSim）训练策略，请使用 [robot_lab](https://github.com/fan-ziqi/robot_lab) 项目。
>
> robot_lab配置文件中的关节顺序 `joint_names` 与本项目代码中 `xxx/robot_lab/config.yaml` 中定义的相同。
>
> 在 [Github Discussion](https://github.com/fan-ziqi/rl_sar/discussions) 或 [Discord](https://www.robotsfan.com/dc_rl_sar) 中讨论

> [!CAUTION]
> **免责声明：使用者确认使用本代码产生的所有风险及后果均由使用者自行承担，作者不承担任何直接或间接责任，操作前必须确保已采取充分安全防护措施。**

## 准备

拉取仓库

```bash
git clone --recursive --depth 1 https://github.com/fan-ziqi/rl_sar.git
```

如需更新

```bash
git pull
git submodule update --init --recursive --recommend-shallow --progress
```

## 依赖

安装必要的依赖：

```bash
# Ubuntu
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# macOS
brew install boost lcm yaml-cpp tbb libomp pkg-config glfw
```

如果需要使用ROS，安装下列依赖包：

```bash
# ros-noetic (Ubuntu20.04)
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager

# ros2-foxy (Ubuntu20.04) / ros2-humble (Ubuntu22.04)
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

## 编译

在项目根目录中执行下面的脚本编译整个项目

```bash
./build.sh
```

`src` 下的 `package.xml` 由脚本按 ROS 版本创建软链接。Humble 使用
`package.ros2.xml`；如果只需补齐 XML 而暂不编译，执行：

```bash
source /opt/ros/humble/setup.bash
./build.sh --configure
```

若想单独编译某几个包，可以在后面加上包名

```bash
./build.sh package1 package2
```

若想删除构建，可以使用下列命令，此命令会删除所有编译产物和创建的软链接

```bash
./build.sh -c  # or ./build.sh --clean
```

如果不需要仿真，只在机器人上运行，可以使用CMake进行编译，同时禁用ROS（编译生成的可执行文件在`cmake_build/bin`中，库在`cmake_build/lib`中）

```bash
./build.sh -m  # or ./build.sh --cmake
```

若想使用Mujoco仿真器

```bash
./build.sh -mj  # or ./build.sh --mujoco
```

此模式只编译 MuJoCo 仿真程序。默认下载 LibTorch 和 ONNX Runtime；如果只运行 TorchScript `.pt` 策略，可以只下载 LibTorch：

```bash
INFERENCE_RUNTIME=libtorch ./build.sh -mj
```

详细的使用说明可以通过`./build.sh -h`查看

```bash
Usage: ./build.sh [OPTIONS] [PACKAGE_NAMES...]

Options:
  -c, --clean    Clean workspace (remove symlinks and build artifacts)
  -m, --cmake    Build using CMake (for hardware deployment only)
  -mj,--mujoco   Build with MuJoCo simulator support (CMake only)"
  -h, --help     Show this help message

Examples:
  ./build.sh                    # Build all ROS packages
  ./build.sh package1 package2  # Build specific ROS packages
  ./build.sh -c                 # Clean all symlinks and build artifacts
  ./build.sh --clean package1   # Clean specific package and build artifacts
  ./build.sh -m                 # Build with CMake for hardware deployment
  ./build.sh -mj                # Build with CMake and MuJoCo simulator support
```

> [!TIP]
> 如果 catkin build 报错: `Unable to find either executable 'empy' or Python module 'em'`, 在`catkin build` 之前执行 `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3`

## 运行

下文中使用 **\<ROBOT\>/\<CONFIG\>** 代替表示不同的环境，如 `go2/himloco` 、 `go2w/robot_lab`。

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>`中，并配置`<ROBOT>/<CONFIG>/config.yaml`和`<ROBOT>/base.yaml`中的参数。

### 仿真

#### Gazebo

打开一个终端，启动gazebo仿真环境

```bash
# ROS1
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=<ROBOT>

# ROS2
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>
```

打开一个新终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_sim

# ROS2
source install/setup.bash
ros2 run rl_sar rl_sim
```

> [!TIP]
> **重要提示：** 启动Gazebo后，必须在另一个终端中启动`rl_sim`来控制机器人。如果不启动`rl_sim`，机器人将不会被控制，可能会摔倒。
>
> Ubuntu22.04中若启动Gazebo后看不到机器人，则是机器人初始化到了视野范围外，启动rl_sim后会自动重置机器人位置。若机器人在站立过程中翻倒，请使用键盘`R`或手柄`RB+Y`重置机器人环境。

如果第一次启动Gazebo无法打开则需要下载模型包

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

#### Mujoco

实时调整摩擦、等效指令延迟、电机参数、观测噪声、质量和重心，可使用新增的 [C++ / Qt 统一调参面板](src/rl_sar/library/thirdparty/sim_tuner/README.md)。编译 `sim_tuner_panel` 后，给仿真追加 `--tuner-port 9870`，另一个终端运行 `./cmake_build/bin/sim_tuner_panel --connect`。`--tuner-port` 同时自动启用策略并开始仿真，无需按起立或启动按键。面板代码与接口实现在 `src/rl_sar/library/thirdparty/sim_tuner/`。

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example: ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof
```

ICE Robocon 使用品牌/型号名称 `ice/robocon`。配置和导出的策略随代码提供；模型资源目录不由本仓库跟踪，首次使用时需生成下文所述的 MJCF：

```bash
./scripts/run_ice_robocon.sh
# 等价命令：./cmake_build/bin/rl_sim_mujoco ice/robocon scene
# 从 home 姿态直接加载最新导出的策略
./scripts/run_ice_robocon.sh scene --auto-policy
```

默认以暂停状态等待手动操作。**点击 MuJoCo 窗口，使其获得键盘焦点**，按 `0` 起立，再按 `Enter` 开始仿真，等待约两秒后按 `1` 启用策略。按住 `W/S` 前后、`A/D` 横移、`Q/E` 转向，以该方向速度上限为目标平滑加速；松键或按住空格后，目标归零并按减速度限制停车，支持组合按键，相反方向相互抵消。键盘没有运动键按住时采用手柄，手柄仍被推动时松键会交回手柄。切换窗口会清除键盘按住状态。`0`/`9` 在策略状态下先减速、等待轮速和角速度稳定，再切换到站立/趴下；`1` 可取消尚未完成的普通停车请求。`P`/手柄 `LB+X` 直接切入阻尼，不等待普通停车。`R` 复位会重新进入当前状态并清除斜坡和旧策略目标；`Enter` 暂停/继续，暂停期间过渡进度保持不变。`--auto-policy` 从初始姿态直接进行平滑策略接管，速度仍由键盘/手柄控制。

键盘和手柄摇杆共用 `policy/ice/robocon/base.yaml` 的 `command_limits_lower/upper`（依次为 vx、vy、wz）。当前范围来自保存的 Smooth 微调配置：前后 ±2.0 m/s、横移 ±0.3 m/s、转向 ±0.8 rad/s；按住 `W` 与左摇杆向前推到底都会以 vx=2.0 m/s 为目标，摇杆半幅约为半速，回中后平滑减速。此阶段尚未扩速到最终的 ±2.5 m/s。`command_acceleration_limits` 和 `command_deceleration_limits` 分别控制三个方向的指令加速和减速，单位为 m/s²、m/s²、rad/s²；换向时先减到零，再反向加速。`getup_duration/getdown_duration` 控制起立和趴下时长，`policy_transition_duration` 控制策略接管时长，`stop_settle_duration` 控制普通退出前的稳定等待时间。修改后重启生效。这里限制的是给策略的指令，实际机身速度和停车距离仍由策略及动力学决定。其他机器人的 FSM 和输入处理保持原有行为。

策略配置在 `policy/ice/robocon/base.yaml` 和 `policy/ice/robocon/robot_lab/config.yaml`；`source.yaml` 记录训练配置来源和 checkpoint 哈希。模型资源位于 `src/rl_sar_zoo/ice/robocon_description/`，其中包含原始 URDF、网格和部署用 MJCF。该资源目录属于单独下载的 `rl_sar_zoo` 仓库，不由本仓库跟踪。

手柄操作：`A` 起立，`RB+X` 暂停/继续，起立完成后 `RB+方向键上` 启用策略；左摇杆控制前后和横移，右摇杆左右控制转向。起立状态只维持姿态，摇杆运动指令需要在策略状态下使用。MuJoCo 适配器从 Linux joydev 读取实际轴/按键映射，兼容 Xbox USB 与蓝牙编号差异；启动日志显示手柄名称及 LX/LY/RX、LB/RB 的实际编号，便于排查映射。

Robocon 当前使用 `mujoco_joint_drive: implicit`，对应训练中的隐式执行器：物理积分器在每个物理步计算 PD 反馈，并对反馈与前馈合力限幅。`explicit` 保留原先按控制周期计算力矩的方式，其他机器人缺省仍使用该方式。导入脚本根据保存的执行器类型生成此项；Python 回放默认读取此项，使用 `--drive explicit` 或 `--drive implicit` 可做对照。驱动反馈形式对齐不表示 Newton 和原生 MuJoCo 的所有动力学差异已经消除。

更新 Robocon 的训练结果时，使用保存的 `params/env.yaml`、`params/agent.yaml` 和 checkpoint 重新导入。脚本复用 `scripts/convert_urdf.py`（来自本地 `unitree_rl_mjlab`），并补齐浮动基座、执行器、传感器、初始姿态和场景；输入 URDF 和训练结果保持原样。

当前部署为 `ice_robocon_smooth_from4999/2026-11-27_20-15-53_target_smooth_airspin_s42/model_7998.pt`，从原始4999微调，20项奖励含原有横移腾空项及新增目标平滑、悬空轮超速约束。原生策略及FSM测试通过；目前跟踪最完整，但横移/混合目标变化和步态风格尚未全面达标。重新导入会保留已有的部署加减速度和状态过渡时长；策略参数仍从该次训练记录生成。

```bash
# Ubuntu 本地独立环境：MuJoCo 3.2.7 / CPU PyTorch 2.3.0 / NumPy 1.26.4
# 依赖解压到 library/，无需 sudo；不修改 robot_lab 的训练环境。
bash scripts/setup_ice_robocon.sh

.venv-deploy/bin/python scripts/import_ice_robocon.py \
  --run-dir /home/jasper/robot_lab/logs/rsl_rl/ice_robocon_smooth_from4999/2026-11-27_20-15-53_target_smooth_airspin_s42 \
  --asset-root /home/jasper/robot_lab/source/robot_lab/robot_lab/assets/data/Robots/ice/robocon \
  --checkpoint model_7998.pt --force

./cmake_build/bin/test_ice_robocon

# 驱动实现：旧显式公式兼容、隐式反馈后限幅、轻载轮阻尼稳定性
cmake --build cmake_build --target test_mujoco_joint_drive -j2
./cmake_build/bin/test_mujoco_joint_drive

# ICE 状态机闭环：起立、接管、启停、反向、暂停、复位和正常/直接退出
cmake --build cmake_build --target test_ice_fsm -j2
./cmake_build/bin/test_ice_fsm

# 单工况诊断：实际 C++ FSM 起立、平滑接管后，保持横移指令 14 秒
# 输出 logs/robocon_diagnostics/fsm/rollout_lateral02.csv，含可回放的 qpos
./cmake_build/bin/test_ice_fsm --rollout lateral02 14 0 0.2 0

# 固定仿真步长回放：站立、前进 0.5、横移 0.2、转向 0.6，共 22 秒
.venv-deploy/bin/python scripts/eval_ice_robocon.py --ramp-commands --video
# 各方向运动后返回零指令，共 34 秒；也可用 --policy-root 指向备份配置做同条件对比
.venv-deploy/bin/python scripts/eval_ice_robocon.py --ramp-commands --profile stop-start --video \
  --output logs/robocon_diagnostics/stop_start
# 原 robot_lab / Newton 环境运行同一策略、同一段指令，关闭随机化和观测噪声
/home/jasper/robot_lab/.venv/bin/python scripts/compare_ice_robocon_robot_lab.py
```

当前 Smooth `model_7998` 的记录在 `logs/robocon_diagnostics/smooth_from4999_7998/report.txt`。28组实际C++定点仿真均未触发保护，原生策略与完整FSM验收通过。前后±2实测+1.947/-1.929，横移±0.2实测+0.190/-0.204，转向±0.6实测+0.581/-0.589；混合(0.5,0.2,0.4)约为(0.472,0.181,0.403)。相比原始4999，横移及混合跟踪改善，但低速仍欠跟踪，部分轮速目标变化和离地事件增加，不能宣称所有动作均丝滑。倒退时接触力可能短暂卸载，按原生姿态重算轮底几何末6秒最大离地仅0.58mm，主要仍为滚动。`native_video/`有实际C++姿态回放；`backup/`保存原始4999。

旧 native_coordination_v2 `model_7999` 的评估记录在 `logs/robocon_diagnostics/native_coordination_v2_7999/report.txt`。实际 C++ rl_sar/LibTorch/FSM 跑了26组固定指令，正横移0.3触发倾斜限制、负横移-0.3触发高度限制；其余工况完成运行不代表跟踪合格。前后±1.5实测约+1.449/-1.450，横移±0.2几乎为零，转向±0.6约+0.305/-0.059；混合指令横移和转向分量失效，并有低趴、悬空轮接近30rad/s空转现象。完整启停FSM测试通过，但横移跟踪测试失败。该版比v1明显退化，不宜扩速。`native_grid/`保存C++日志与CSV，`native_video/`保存其姿态轨迹回放，`backup/`保存v1策略和模型。辅助Python回放与原生FSM启动流程不同，其失败计数单独记录。

上一版 native_coordination `model_7999` 的评估记录在 `logs/robocon_diagnostics/native_coordination_7999/report.txt`，含新旧三版相同隐式驱动下的对照、原训练环境对照、录像和有效奖励核查。20 组固定指令未倒地，完整 FSM 测试通过；前后 ±1.5 m/s 实测约 +1.49/-1.45 m/s。但横移 0.2 m/s 实测约 0.116 m/s，低速横移和混合指令欠跟踪，横移时仍有轮速振荡，`test_ice_robocon` 因横移误差超过 0.05 m/s 失败。未放宽判据。原 robot_lab/Newton 横移也约 0.117 m/s，此阶段尚不宜直接扩速。

旧 balanced_commands_resume `model_7999` 的本机评估记录在 `logs/robocon_diagnostics/balanced_7999/`：中速前进、横移和转向跟踪改善，但低速横移、长期静止漂移和正常退出仍有问题。`test_ice_fsm` 在等待正常退出时失败：轮速不能连续 0.3 秒满足停车判据。

旧 `model_9998` 的评估记录在 `logs/robocon_diagnostics/smooth_9998/`。前后 ±2.5 m/s 滚动改善，但横移欠跟踪、部分转向和混合指令倒地；当时原生策略测试与完整 FSM 测试均未通过。`eval_ice_robocon.py` 会保存 `diagnostics.npz`，包含关节目标、实际关节运动和四轮地面法向力；通过 `--ramp-commands` 使用部署加减速斜坡，并通过 `--policy-root` 指定备份策略。

上述 `model_9998` 历史数据基于旧显式 PD。切换隐式驱动后，状态机流程不再复现该次转向倒地，但横移跟踪仍不合格；`test_ice_robocon` 已新增横移误差判据，不能再仅凭转向和不倒地判断通过。驱动改动的记录在 `logs/robocon_diagnostics/native_baseline_setup/`。新的训练基线默认关闭所有自定义奖励，先训练低速协调任务，再使用独立 Speed 任务扩到 ±2.5 m/s；训练命令与单项腾空奖励对照见 `robot_lab/README.md`。

导入脚本仅支持当前 Robocon 的无历史帧、无观测归一化 ELU MLP 策略。它按保存的训练配置展开关节参数和左右轮速度符号，并在动作缩放后裁剪目标；原始动作仍用于下一帧观测。改变策略结构或观测定义时，需要同步适配导入和部署代码。

如果现有 `policy.pt` 和 YAML 已经就绪，只缺少 `robocon.xml`、`scene.xml`
及网格，使用 `--model-only`。此模式读取保存的 `params/env.yaml` 和现有策略配置，
生成模型时保留策略、配置和来源记录，不需要原 checkpoint。下面是本机已有资源的示例：

```bash
.venv-deploy/bin/python scripts/import_ice_robocon.py --model-only \
  --run-dir "$HOME/robot_lab/logs/rsl_rl/ice_robocon_flat/2026-11-24_16-06-21_robocon_motor_update_validation" \
  --asset-root "$HOME/robot_lab/source/robot_lab/robot_lab/assets/data/Robots/ice/robocon"

# 资源已存在时，重新生成需追加 --force
./scripts/run_ice_robocon.sh scene --auto-policy
```

Robocon 默认启用 `physics_sync_control`：物理步长 0.0025 s，每两个物理步更新一次 PD，每四次 PD 更新运行一次策略。运行中由物理线程读取一致状态并立即消费动作，暂停时仍处理窗口输入；调整 MuJoCo 播放速度不会改变控制的仿真周期。其他机器人保留原来的线程调度。

XML 从训练配置读取 Newton 的接触材料参数。此训练记录的机器人摩擦系数为 1.0，地面为 0.6，实际接触取 1.0；`ke=160000`、`kd=1100` 对应 `solref="0.00181818 1.375"`。原脚本将机器人摩擦写为 0.6，并使用默认 `solref="0.02 1"`，会改变轮地滑移和支撑。原训练使用 Newton 碰撞检测，部署使用 MuJoCo 碰撞检测；参数对齐后，横移等动作仍可能存在差异。

回放的 CSV、数值报告和视频位于 `logs/robocon_diagnostics/latest/`。原生界面可额外记录机体坐标系速度、姿态和指令：

```bash
mkdir -p logs/robocon_diagnostics
RL_SAR_TELEMETRY=logs/robocon_diagnostics/interactive.csv \
  ./scripts/run_ice_robocon.sh scene --auto-policy
```

本地 `unitree_rl_mjlab` 的 Bodydemo03 部署模板使用另一套策略约定（含 gait phase、不同关节顺序和 400/12 的腿部 PD）。本任务的 robot_lab 策略是 57 维观测、16 维动作、60/2 的腿部 PD，不能直接套用该模板；这里只复用其 URDF 转换思路。

低速测试还需注意训练命令生成器：`UniformThresholdVelocityCommand._resample_command` 会将平移指令模长不超过 0.2 m/s 的指令清零，策略很少接触这些非零小指令。手柄小幅输入仍可能进入这个区间。原训练环境直接输入 vy=0.1 的对照也几乎没有横移，不能仅凭低速不响应判断模型移植失败。支持这一低速范围需要调整训练命令分布及相应奖励后验证。

#### Docker

Docker支持仿真和部署。详见 [docker/README.md](docker/README.md)。

```bash
# 启动容器
xhost +local:docker  # 在主机上启用X11
cd docker && docker compose up -d
docker compose exec rl_sar bash

# 容器内 - MuJoCo仿真
./cmake_build/bin/rl_sim_mujoco g1 scene_29dof

# 容器内 - Gazebo仿真
ros2 launch rl_sar gazebo.launch.py rname:=go2
# (新终端) ros2 run rl_sar rl_sim

# 容器内 - 真实机器人
./cmake_build/bin/rl_real_go2 <NETWORK_INTERFACE>
```

### 使用手机网页控制 (实验性)

安装依赖

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-web-video-server

# 如果您使用的ROS2版本不是Humble、Jazz或Rolling，需要从源码编译 `web_video_server`
cd <your_ros2_workspace>/src
git clone https://github.com/RobotWebTools/web_video_server.git
cd <your_ros2_workspace>
colcon build --packages-select web_video_server
```

在机器人上运行 rosbridge 和 web_video_server

```bash
# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun web_video_server web_video_server

# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run web_video_server web_video_server
```

访问 [http://robot.robotsfan.com/](http://robot.robotsfan.com/)， 填写IP地址和端口，检查右上角的设置界面，然后连接机器人。进入控制页面后，将屏幕水平放置，点击左上角的全屏按钮，即可使用手机浏览器控制机器人！

### 使用手柄或键盘控制

MuJoCo 使用窗口内的按住/松开控制，速度上限见上面的 Robocon 说明。下面的方向键增减描述适用于原有 Gazebo/真机终端控制。

|手柄控制|键盘控制|功能描述|
|---|---|---|
|**基础**|||
|A|Num0|让机器人从程序开始运行时的姿态以位控插值运动到`base.yaml`中定义的`default_dof_pos`|
|B|Num9|让机器人从当前位置以位控插值运动到程序开始运行时的姿态|
|X|N|切换导航模式 (导航模式屏蔽速度命令，接收`cmd_vel`话题)|
|Y|N/A|N/A|
|**仿真**|||
|RB+Y|R|重置Gazebo环境 (让摔倒的机器人站起来)|
|RB+X|Enter|切换Gazebo运行/停止 (默认为运行状态)|
|**电机**|||
|LB+A|M|N/A (推荐设置为电机使能)|
|LB+B|K|N/A (推荐设置为电机失能)|
|LB+X|P|电机Passive模式 (`kp=0, kd=8`)|
|LB+RB|N/A|N/A (推荐设置为急停保护)|
|**技能**|||
|RB+DPadUp|Num1|基础Locomotion|
|RB+DPadDown|Num2|技能2|
|RB+DPadLeft|Num3|技能3|
|RB+DPadRight|Num4|技能4|
|LB+DPadUp|Num5|技能5|
|LB+DPadDown|Num6|技能6|
|LB+DPadLeft|Num7|技能7|
|LB+DPadRight|Num8|技能8|
|**移动**|||
|LY轴|W/S|前后移动 (X轴)|
|LX轴|A/D|左右移动 (Y轴)|
|RX轴|Q/E|偏航旋转 (Yaw)|
|N/A(松开摇杆)|Space|将所有控制指令设置为零|

### 真实机器人

<details>

<summary>Unitree A1（点击展开）</summary>

与Unitree A1连接可以使用无线与有线两种方式

- 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
- 有线：用网线连接计算机和机器人的任意网口，配置计算机地址为192.168.123.162，子网掩码255.255.255.0

新建终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_a1

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_a1

# CMake
./cmake_build/bin/rl_real_a1
```

</details>

<details>

<summary>Unitree Go2/Go2W/G1(29dofs)（点击展开）</summary>

#### 网线连接

用网线的一端连接Go2/Go2W/G1(29dofs)机器人，另一端连接你的电脑，并开启电脑的 USB Ethernet 后进行配置。机器狗机载电脑的 IP 地地址为 `192.168.123.161`，故需将电脑 USB Ethernet 地址设置为与机器狗同一网段，如在 Address 中输入 `192.168.123.222` (`222`可以改成其他)。

通过`ifconfig`命令查看123网段的网卡名字，如`enxf8e43b808e06`，下文用 \<YOUR_NETWORK_INTERFACE\> 代替

Go2:

新建终端，启动控制程序。如果控制Go2W，需要在命令后加`wheel`，否则留空。

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# CMake
./cmake_build/bin/rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]
```

G1(29dofs):

开机后将机器人吊起来，按L2+R2进入调试模式，然后新建终端，启动控制程序。

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# CMake
./cmake_build/bin/rl_real_g1 <YOUR_NETWORK_INTERFACE>
```

#### 在机载Jetson中部署

使用网线连接电脑和机器人，登陆Jetson主机，密码123：

```bash
ssh unitree@192.168.123.18
```

将手机连接机器人的USB，开启手机的USB网络共享，拉取代码并使用 `./build.sh -m` 编译，编译成功后运行：

```bash
# Go2:
./cmake_build/bin/rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# G1(29dofs):
./cmake_build/bin/rl_real_g1 <YOUR_NETWORK_INTERFACE>
```

然后即可拔掉手机和网线，使用遥控器控制。

#### 开机自启动

如需设置开机自启动，可以参考以下流程：

创建一个服务文件

```bash
sudo touch /etc/systemd/system/rl_sar.service
```

写入以下内容，假设rl_sar工程在 `~/rl_sar` 目录下

```
[Unit]
Description=RL SAR Service
After=network.target

[Service]
Type=simple
User=unitree
WorkingDirectory=/home/unitree/rl_sar
ExecStart=/home/unitree/rl_sar/cmake_build/bin/rl_real_go2 eth0 wheel
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

重新加载 systemd 配置：

```bash
sudo systemctl daemon-reload
```

设置开机自动启动：

```bash
sudo systemctl enable rl_sar.service
```

禁用开机自动启动：

```bash
sudo systemctl disable rl_sar.service
```

启动服务：

```bash
sudo systemctl enable rl_sar.service
```

停止服务：

```bash
sudo systemctl stop rl_sar.service
```

重启服务：

```bash
sudo systemctl restart rl_sar.service
```

查看服务日志：

```bash
sudo journalctl -u rl_sar.service -f
```

重启后机机器人会先运行内置站立程序，rl_sar服务启动后则会自动阻尼趴下，随后可以使用遥控器正常控制。

</details>

<details>

<summary>云深处科技 Lite3 (Click to expand)</summary>

Lite3通过无线网络进行连接。
(由于一些型号的Lite3没有开放网线接口，需要额外安装，所以有线连接方式暂时没有进行测试)

- 连接Lite3的Wifi，并测试通信状况。我们强烈建议在运行本项目之前，先通过 [Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)进行测试和检查，在确认一切正常后再运行。
 **(注意：无线连接可能会出现丢包断联甚至失控，请注意安全)**

- 确认所使用Lite3的IP地址和本地端口与目标端口号码，并设置 **在 rl_sar/src/rl_real_lite3.cpp的行46-48**中.
- 在Lite3的运动主机中设置 **jy_exe/conf/network.toml**，使其IP地址指向与Lite3同一网段的本机，建立基于UDP的双向通信.

> [!CAUTION]
> **检查关节映射参数<br>检查确认 rl_sar/policy/himloco/config.yaml中的joint mappng参数。在Sim2Sim中使用的默认joint mapping参数与实机部署时的joint mapping是不同的，如果使用错误可能造成机器人错误的行为，带来潜在的硬件损坏和安全风险。**

Lite3也支持使用云深处Retroid手柄控制，详情参见[Deeprobotics Gamepad](https://github.com/DeepRoboticsLab/gamepad)

新建终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_lite3

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_lite3

# CMake
./cmake_build/bin/rl_real_lite3
```

</details>

<details>

<summary>智元 Agibot D1（点击展开）</summary>

D1通过无线网络进行连接。

- 连接D1的WiFi（默认SSID和密码在机器人侧面标签上）
- 机器人默认IP: `192.168.234.1` (WiFi) 或 `192.168.168.168` (有线)
- 需要在机器人端配置 `/opt/export/config/sdk_config.yaml`，将 `target_ip` 设置为你的电脑IP

**网络配置步骤：**

1. SSH登录机器人：
```bash
ssh firefly@192.168.234.1  # 密码: firefly
```

2. 修改SDK配置文件：
```bash
vim /opt/export/config/sdk_config.yaml
```
将 `target_ip` 修改为你的电脑IP地址（如 `192.168.234.2`）

3. 重启机器人使配置生效

**运行控制程序：**

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_d1 [本地IP] [机器人IP]

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_d1 [本地IP] [机器人IP]

# CMake
./cmake_build/bin/rl_real_d1 [本地IP] [机器人IP]

# 示例（使用默认IP）
./cmake_build/bin/rl_real_d1 192.168.234.2 192.168.234.1
```

</details>

### 训练执行器网络

下面拿A1举例

1. 取消注释`rl_real_a1.hpp`中最上面的`#define CSV_LOGGER`，你也可以在仿真程序中修改对应部分采集仿真数据用来测试训练过程。
2. 运行控制程序，程序会记录所有数据到`src/rl_sar/policy/<ROBOT>/motor.csv`。
3. 停止控制程序，开始训练执行器网络。注意，下面的路径前均省略了`rl_sar/src/rl_sar/policy/`。
    ```bash
    rosrun rl_sar actuator_net.py --mode train --data a1/motor.csv --output a1/motor.pt
    ```
4. 验证已经训练好的训练执行器网络。
    ```bash
    rosrun rl_sar actuator_net.py --mode play --data a1/motor.csv --output a1/motor.pt
    ```

## 添加你的机器人

下面使用 **\<ROBOT\>/\<CONFIG\>** 代替表示你的机器人环境。你只需要创建或修改下述文件，命名必须跟下面一样。（你可以参考go2w对应的文件）

```yaml
# 你的机器人description
rl_sar/src/rl_sar_zoo/<ROBOT>_description/CMakeLists.txt
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros1.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros2.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/robot.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/gazebo.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control.yaml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control_ros2.yaml

# 你训练的policy
policy/<ROBOT>/base.yaml  # 此文件中必须遵守实物机器人的关节顺序
policy/<ROBOT>/<CONFIG>/config.yaml
policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # libtorch使用，注意导出jit
policy/<ROBOT>/<CONFIG>/<POLICY>.onnx  # onnxruntime使用

# 机器人的fsm
src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp
src/rl_sar/fsm_robot/fsm_all.hpp

# 你实物机器人的代码
rl_sar/src/rl_sar/src/rl_real_<ROBOT>.cpp  # 可以按需自定义forward()函数以适配您的policy
```

## 贡献

衷心欢迎社区的贡献，以使这个框架更加成熟和对所有人有用。贡献可以是bug报告、功能请求或代码贡献。

[贡献者名单](CONTRIBUTORS.md)

## 引用

如果您使用此代码或其部分内容，请引用以下内容：

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

## 致谢

本项目使用了以下开源代码库中的部分代码：

- [unitreerobotics/unitree_sdk2-2.0.0](https://github.com/unitreerobotics/unitree_sdk2/tree/2.0.0)
- [unitreerobotics/unitree_legged_sdk-v3.2](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.2)
- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [google-deepmind/mujoco-3.2.7](https://github.com/google-deepmind/mujoco)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)
- [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)
- [Deeprobotics/Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
- [chengyangkj/ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)
