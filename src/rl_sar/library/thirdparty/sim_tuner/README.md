# MuJoCo 统一调参面板（C++ / Qt）

独立桌面面板，通过本机 TCP 修改 `rl_sim_mujoco` 的运行参数。面板使用 Qt 5 Widgets / Network；仿真端不依赖 Qt。参数定义共享于 `parameters.hpp`，接口和实现位于本目录。

## 构建与启动

在工程根目录执行（需要 Qt 5.15 开发库；Ubuntu 对应 `qtbase5-dev`）：

```bash
cmake -S src/rl_sar -B cmake_build -DUSE_CMAKE=ON -DUSE_MUJOCO=ON
cmake --build cmake_build --target sim_tuner_panel rl_sim_mujoco -j2

# 终端 1：启动带调参接口的仿真；不传 --tuner-port 则使用原有控制链路。
./cmake_build/bin/rl_sim_mujoco ice/robocon scene --tuner-port 9870
# 也可使用 ./scripts/run_ice_robocon.sh scene --tuner-port 9870

# 终端 2：启动调参面板。
./cmake_build/bin/sim_tuner_panel --port 9870 --connect
```

`--tuner-port` 自动包含 `--auto-policy` 行为：从初始姿态直接平滑接管策略并开始仿真，无需按起立、开始或切换策略按键，也无需等待面板连接。没有启用调参时，单独追加 `--auto-policy` 也可自动启动；不传这两个选项的 Robocon 保留手动启动方式。暂停期间仍能调参。多个仿真使用不同端口；每个仿真同时允许一个面板连接。接口只绑定 `127.0.0.1`。

面板也可独立构建，无需 MuJoCo、LibTorch 或 ROS：

```bash
cmake -S src/rl_sar/library/thirdparty/sim_tuner -B /tmp/sim_tuner_build
cmake --build /tmp/sim_tuner_build -j2
/tmp/sim_tuner_build/sim_tuner_panel
```

主工程未找到 Qt 时跳过面板目标，仍可构建仿真端；也可显式指定 `-DBUILD_SIM_TUNER_PANEL=OFF`。

## 面板操作

- **接触**：开启“覆盖滑动摩擦”，再调整系数；同时更新场景碰撞 geom 和显式 contact pair，避免轮地摩擦取较大值导致低摩擦设置无效。关闭覆盖恢复各自的 XML 原值。接触时间常数倍率只修改正值格式的 `solref[0]`，不更改直接刚度格式。
- **驱动**：等效指令延迟、电机强度、力矩上限、Kp/Kd 倍率、附加关节阻尼与摩擦。全机器人统一调整，不更改策略文件。
- **观测**：关节位置/速度、陀螺仪的独立高斯噪声，以及陀螺仪固定偏置。只修改控制器读到的状态，不修改 `mjData` 的物理状态。随机种子可设，复位重启随机序列。
- **模型**：机器人子树质量、额外惯量、基座局部重心偏移、关节转子惯量，以及重力倍率。带 `*` 的参数通过“应用全部并复位”生效；其他参数可实时应用。质量倍率同时缩放质量和惯量，额外惯量倍率在此基础上再缩放。
- “设置值”与“已生效”分别展示草稿和仿真确认值；黄色边框表示两者不同。普通参数默认在停止拖动 150 ms 后自动发送。模型参数仅保存为草稿，直到手动点击“应用全部并复位”。
- 保存/加载 JSON 配置。加载仅填入面板，不自动应用；重新连接会读取仿真当前值，因此离线准备的配置可在连接后加载。恢复基准会复位姿态并恢复启动时的模型数值。
- 右侧显示仿真时间、参数版本与生效时间、质量、滑动摩擦范围、在途指令数。可选择关节查看位置、速度和力矩曲线。黄色曲线是延迟后已采用的目标，绿色是物理状态；10 Hz 曲线用于观察趋势，不适合辨识高频电机动态。

所有倍率相对启动时的原始值计算，重复修改不会累乘。关闭面板或断开连接保留仿真当前参数；重新启动仿真回到 XML/YAML 基准。不会写回模型或策略配置。

## 等效延迟的定义

`delay_ms` 延迟一整条电机指令（q、dq、Kp、Kd、前馈力矩、原始力矩限幅）。在 t 发出的指令，于第一个满足 `sim_time >= t + delay_ms / 1000` 的物理步采用；采用后保持到下一条指令到达。隐式 PD 仍使用物理引擎的当前本地反馈，不把上位机观测噪声注入电机内部反馈。

- 时间基准为 MuJoCo 仿真时间；暂停、慢放不会改变延迟的仿真时长，也没有 `sleep()` 阻塞物理线程。
- 延迟精度受物理步长限制。例如 2.5 ms 步长下，3 ms 延迟会在后续第 2 个物理步（5 ms）生效。
- 首条延迟指令到达前，电机输出为零。运行中修改延迟会清除在途指令，保持最后已经采用的目标，后续新指令使用新延迟。
- 仿真复位、历史回退会清空旧指令；暂停时同一仿真时刻的指令只保留最新一条。
- 延迟作用于 FSM 发出的全部电机指令，包括起立、阻尼等状态。此工具用于仿真，不改变实机控制程序。

`motor_strength` 在 PD 反馈和前馈上同时乘倍率，随后按 `torque_limit_scale` 限幅。模型中既有的关节 `actuatorfrcrange` 和其他硬限制仍可能使实际输出低于面板限幅。被动阻尼、摩擦和转子惯量只修改机器人子树的 hinge 关节，不作用于浮动基座。

## 接口与线程

协议为一行一个 JSON 对象，版本为 1。接收线程只收发数据；仿真线程持有 `sim->mtx` 时处理请求、校验整批参数并应用，暂停时由原控制循环处理。单条请求限 64 KiB，队列和每次处理数量有上限。模型参数重算使用独立 `mjData` 调用 `mj_setConst`，随后通过现有复位入口重置仿真。运行中更换整个模型需要重启控制器。

```json
{"protocol":1,"id":1,"op":"status"}
{"protocol":1,"id":2,"op":"set","params":{"friction_override":1,"sliding_friction":0.3,"delay_ms":10}}
{"protocol":1,"id":3,"op":"set","reset":true,"params":{"mass_scale":1.1}}
{"protocol":1,"id":4,"op":"defaults","reset":true}
```

成功响应包含 `ok`、请求 `id`、完整 `params`、`version`、`applied_at`、仿真时间和关节数据；失败响应包含 `ok:false` 和 `error`，校验失败时整批参数不生效。状态回读是已应用的模型/参数数据；摩擦范围表示碰撞 geom 的参数范围，单个实际接触仍受 MuJoCo 的接触配置影响。

## 验证

```bash
cmake --build cmake_build --target test_sim_tuner -j2
./cmake_build/bin/test_sim_tuner
QT_QPA_PLATFORM=offscreen ./cmake_build/bin/sim_tuner_panel --smoke-test
```

测试涵盖批量校验、摩擦及显式 pair、模型复位与派生常量、指令延迟边界、暂停队列、强度与限幅、时间回退、噪声复现和基准恢复。`test_sim_tuner --serve <port>` 可启动无 GUI 的小型物理测试服务，约 60 秒后退出，供协议和面板联调。
