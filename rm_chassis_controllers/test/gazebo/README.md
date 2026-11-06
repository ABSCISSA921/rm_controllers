# Gazebo 原链路集成工具

<a id="scope"></a>
## 范围与实际模型

本目录仅提供测试工具。`session.py` 默认只准备路径、单份车型配置视图和展开 URDF；只有显式 `--run` 才启动隔离进程。普通生产启动链、公共硬件框架、Manual、virtualDbus 和机械源文件均不修改。

当前 `series_legged2/chassis.urdf.xacro` 实际包含 `series_legged2/leg.urdf.xacro`；后者 knee 的 parent 是 thigh，是串联链。MuJoCo 的 `series_legged` 是带被动杆的五连杆。这两个 plant 的 A/B/C 结果不能合并作同机械性能比较。现有 C 多体模型要求五连杆闭链，不能只替换模型路径后用于该 Gazebo plant；后续若实现串联适配，还须同步编码器到 FK、观测、参考和零闭链约束维数。

先按原动作真实尝试 A/B 起立并评分。若具体失败已说明机械几何接口不适配，保留失败证据、收敛所需兼容改动，不机械重复九次试验。当前工具完成不代表原动作或 GUI 集成已通过。

<a id="prepare"></a>
## 准备与执行

从工作区根目录执行：

```bash
source base_link_control_plan/devel/setup.bash
cmake -S src/rm_controllers/rm_chassis_controllers/test/gazebo \
  -B base_link_control_plan/gazebo_tools/build -DCMAKE_BUILD_TYPE=Release
cmake --build base_link_control_plan/gazebo_tools/build -j2
python3 src/rm_controllers/rm_chassis_controllers/test/gazebo/session.py \
  --artifact base_link_control_plan/artifacts/A \
  --output base_link_control_plan/gazebo_tools/new_preflight_A
```

冻结包需包含 `lib/librm_chassis_controllers.so`、`controller.yaml`、`manual.yaml`。每个运行目录只在原车型文件名下选择该版本的一份 YAML；这是用于 A/B 版本对照的局部测试包视图，不是生产算法覆盖文件或第二套算法配置。默认 Gazebo 描述始终来自当前工作区原 `rm_description`，其 dirty 文件保留。

获得执行安排后，对新的输出目录追加 `--run`。会依序载入原 `rm_gazebo/empty_world.launch` 和原 `rm_config/manual.launch`。测试器只在 roslaunch 的内存 node 配置中向 gzserver 添加 `-s libbase_link_gazebo_observer.so`，原 launch 文件不变。ROS 与 Gazebo master 均使用随机 localhost 端口，Gazebo 进程实载控制器 `.so` 的 `/proc/PID/maps` 与冻结包 SHA256 复核。

`rqt_driver.py` 由真实 `rqt_gui.Main` 加载原 C++ `rqt_virtual_dbus` 插件。QtTest 对插件的 `SlipButton`、`KeyboardButton` 发鼠标/按键事件，原拨杆 widget 改值经过其 `valueChanged` 信号。工具自身没有 ROS publisher；DBUS 由原插件发布，命令由原 Manual 产生。

默认 `QT_QPA_PLATFORM=offscreen`，移除该进程的 `DISPLAY`/`WAYLAND_DISPLAY`，不触碰用户桌面。原 `GlobalMouseInput` 构造仅连接 timer，鼠标未勾选时不启动；工具保持鼠标捕获关闭、wheel 居中。Qt 离屏鼠标事件与 QCursor API 已独立验证。按集成执行安排，在内存 launch 配置关闭 gzclient，保留 gzserver 物理引擎及原机械/算法；这不修改原 launch 文件。若实际传感器渲染仍需 X 显示，记录错误后可局部准备 Xvfb，不能使用用户桌面或安装全局软件。

首次用 [stand_attempt.json](stand_attempt.json) 实际起立。成立后才用以下命令生成三个场景：

```bash
python3 src/rm_controllers/rm_chassis_controllers/test/gazebo/scenarios.py \
  --output base_link_control_plan/gazebo_tools/scenarios
```

S1 为 W/S 启停反向，S2 为 Shift 静止及移动升降，S3 为 C 切换 RAW 并加入 W。事件按仿真时间触发，实际 Manual 命令另行记录，不把 GUI 指令幅值当实际底盘命令。

<a id="evidence"></a>
## 数据与时间语义

`truth/physics.csv` 在每个 `WorldUpdateEnd` 同一回调中读取 base 原点、左右轮 link 原点的位姿/世界速度、六编码器 q/dq 和 Gazebo `Joint::GetForce`。base 速度使用 `WorldLinearVel()` 的 link 原点定义，未用 CoM 速度替代。采样器没有 set-pose、force 写入、接触改写或控制器状态修改。

`truth/contacts.csv` 保留每个碰撞点、法向、深度、两侧世界系接触力和接触自身时间。普通 Gazebo transport 订阅请求接触报告，采样直接读物理管理器，未把异步 topic 最新值混入物理行。`Joint::GetForce` 单独标记为 API 诊断值；未经与后端施力语义核验，不称作实际执行电机力矩。

`received_topics.jsonl` 保留 DBUS、`/cmd_vel`、`/cmd_chassis`、`/leg_cmd`、Mode 和兼容状态的原消息与接收时间。无 header 的命令只有接收时间，不能据此声称逐拍和物理真值同源。GUI 事件在 `gui_events.jsonl`，插件截图在 `virtual_dbus_*.png`。截图或 Mode=Normal 都不是物理站立成功证据。

`manifest.json` 记录选定版本、模型/配置/库哈希、机械拓扑、隔离地址、实际加载库和运行状态。`attempt_finished` 只表示工具链完成；物理起立、稳态、任务跟踪和失败原因需基于真值另行评分。

<a id="links"></a>
## 计划入口

[主计划](../../../../../BASE_LINK_CONTROL_PLAN.md) · [测试规程](../../../../../base_link_control_plan/TEST_PROTOCOL.md) · [模型契约](../../../../../base_link_control_plan/MODEL_CONTRACT.md)
