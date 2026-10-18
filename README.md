# rm_controllers
## Status
### Continuous Integration
[![CI](https://github.com/rm-controls/rm_controllers/actions/workflows/industrial_ci.yml/badge.svg)](https://github.com/rm-controls/rm_controllers/actions/workflows/industrial_ci.yml)
[![Format](https://github.com/rm-controls/rm_controllers/actions/workflows/format.yml/badge.svg)](https://github.com/rm-controls/rm_controllers/actions/workflows/format.yml)
[![Doxygen-docs](https://github.com/rm-controls/rm_controllers/actions/workflows/doxygen.yml/badge.svg)](https://github.com/rm-controls/rm_controllers/actions/workflows/doxygen.yml)

### Licenses
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2Frm-controls%2Frm_controllers.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2Frm-controls%2Frm_controllers?ref=badge_large)
## Documentation
[rm-controls Documentation](https://rm-control-docs.netlify.app/)

[rm-control Doxygen Documentation](https://rm-controller.netlify.app/)

## 轮腿控制器与外部 TinyMPC 的构建

### 当前范围与默认行为

`RM_BUILD_BIPEDAL` 默认 `OFF`。普通机器人直接执行 `catkin build`，不编译轮腿源码，也不查找、构建或下载 TinyMPC。轮腿使用者显式开启一次后，选择保存在该包的 `CMakeCache.txt` 中，后续仍直接执行 `catkin build`，不需要 profile。

轮腿 `Normal` 的地面四维反馈已接入 TinyMPC。`Normal` 在当前控制周期直接调用后端，使用当前观测和参考轨迹同步求解。求解失败时请求进入 PROTECT，并将本周期关节命令清零。配置和执行边界见本文末尾。

轮腿构建只使用 `RM_BUILD_BIPEDAL` 一个开关：

| RM_BUILD_BIPEDAL | 行为 |
| --- | --- |
| OFF（默认） | 仅构建公共底盘库，不查找 TinyMPC |
| ON | 必须找到并通过兼容性检查的外部 TinyMPC SDK，再构建并私有链接独立轮腿库；依赖缺失时配置失败 |

旧的 `RM_BIPEDAL_WITH_TINYMPC` 已移除，重新配置时会自动清除其缓存项。无论旧值是什么，轮腿是否构建、是否要求 TinyMPC 都只由 `RM_BUILD_BIPEDAL` 决定，不再提供无 TinyMPC 依赖的轮腿构建模式。

库的依赖方向如下：

```text
librm_chassis_controllers.so                 始终构建
  公共 ChassisBase、Omni、Sentry、Swerve、Balance

librm_bipedal_controllers.so                 显式开启才构建
  BipedalController、VMCController、轮腿模型/估计/VMC/状态机
    -> librm_chassis_controllers.so
    -> 外部 librm_tinympc.a                   开启轮腿时必须提供
```

公共底盘库不依赖轮腿库或 TinyMPC。`catkin_package(LIBRARIES ...)` 只导出公共库，避免下游 `${catkin_LIBRARIES}` 被动带入轮腿依赖。普通两轮平衡的 `BalanceController` 始终保留；轮腿 `BipedalController` 和调试用 `VMCController` 一起受开关控制。

插件类名保持 `rm_chassis_controllers/BipedalController`、`rm_chassis_controllers/VMCController`，已有 YAML 无需改名。插件 XML 使用静态 `<class_libraries>` 描述两个库：关闭轮腿后仍可能枚举到这些名称，但仅请求加载轮腿插件时才会尝试打开轮腿库。静态声明本身不会加载或编译轮腿库。

### Eigen 兼容性实测结论（2026-10-18）

| 项目 | 固定验证条件 |
| --- | --- |
| TinyMPC 来源 | [TinyMPC/TinyMPC](https://github.com/TinyMPC/TinyMPC) |
| 上游 commit | `023f36bd5b27267e1a96ed47a774d120b5707f13` |
| 工作空间 Eigen | `/usr/include/eigen3`，3.3.7；`Eigen3_DIR=/usr/lib/cmake/eigen3` |
| 上游内附 Eigen | 版本宏为 3.4.90，仅用于对照构建 |
| 工具链 | GCC 9.4.0、CMake 3.16.3 |
| 语言标准 | 外部 TinyMPC 实现 C++17，消费其头文件和静态库的测试程序 C++14，与控制器一致 |

**未经修改的该版本 TinyMPC 不能直接使用 Eigen 3.3.7。** 编译失败点为：

1. `admm.cpp` 使用 `s(Eigen::placeholders::last)`；3.3.7 没有该索引 API。
2. `codegen.cpp` 使用 `mat.reshaped<RowMajor>()[i]`；3.3.7 没有 `reshaped()`。

仓库提供的 [外部构建驱动](tools/tinympc_external/CMakeLists.txt) 只在**外部 build 目录中的源码副本**上进行两处等价替换：

```cpp
s(s.size() - 1)
mat(i / mat.cols(), i % mat.cols())
```

前者读取向量最后一个元素，后者按行优先顺序读取矩阵元素。上游 checkout 保持不变；仓库不保存 TinyMPC 实现或它的 Eigen，也不使用 FetchContent 或 submodule。`Eigen.h` 仅用三行适配头转发到系统 `<Eigen/Core>`、`<Eigen/LU>`，不是版本桥接层。

驱动还会在构建副本中移除每次求解成功时的 `Solver converged in ... iterations` 标准输出，避免控制循环持续打印；此修改不受 `RM_TINYMPC_EIGEN33_COMPAT` 开关影响。已有 SDK 需重新构建、安装，并重新链接轮腿库后生效。

驱动固定检查上述 commit 和未修改的上游受跟踪文件，使用 `find_package(Eigen3 3.3.7 EXACT CONFIG REQUIRED)`；遇到版本变化应重新验证，不能直接放宽版本条件。

本次实测：

- 全部四个上游实现文件均能使用系统 Eigen 3.3.7 编译，编译参数不含上游内附 Eigen 目录。
- [数值检查](tools/tinympc_external/smoke.cpp) 覆盖十状态、四输入、16 个预测节点、宽输入边界和激活饱和边界，各运行 40 个闭环步；检查收敛状态、有限值、输入约束和预测动力学误差。
- 检查二阶锥投影的三个分支，并执行代码生成，覆盖两处兼容修改。
- 与未修改上游、内附 Eigen 的构建对照：两组第一步控制量、最终状态范数和最大输入的最大绝对差均为 `0`；每组生成文件中的 2821 个数值最大绝对差也为 `0`。
- 这是所列版本、工具链和测试场景的兼容证据，不是轮腿实机稳定性或求解实时性的证明；自适应 rho 在测试中关闭。

因此当前采用统一 Eigen 3.3.7，暂不引入两套 Eigen 的桥接接口，也不升级工作空间或系统 Eigen。

### 第一步：普通机器人构建

在已经准备好 ROS 和本工作空间依赖的终端中执行：

```bash
cd /home/nesc/rm_ws
catkin build
```

全新的包构建目录默认得到 `RM_BUILD_BIPEDAL:BOOL=OFF`。只有以前显式开启过的构建目录会继续保持 ON，符合轮腿机器人的使用方式：

```bash
rg '^RM_BUILD_BIPEDAL:' \
  build/rm_chassis_controllers/CMakeCache.txt
```

### 第二步：在工作空间外准备 TinyMPC

下面使用用户目录作为外部源码、构建和安装位置，不修改全局 `Eigen3_DIR`、`CMAKE_PREFIX_PATH` 或 shell 启动文件。各命令成功后再继续下一步；首次 clone 的目标目录应不存在。

```bash
RM_WS=/home/nesc/rm_ws
RM_TINY_ROOT="$HOME/third_party/rm-tinympc"
RM_TINY_PREFIX="$HOME/.local/opt/rm-tinympc/eigen-3.3.7"

mkdir -p "$RM_TINY_ROOT"
git clone https://github.com/TinyMPC/TinyMPC.git "$RM_TINY_ROOT/TinyMPC"
git -C "$RM_TINY_ROOT/TinyMPC" checkout --detach \
  023f36bd5b27267e1a96ed47a774d120b5707f13
```

先确认控制器当前使用的 Eigen 路径和版本与本文相同。如果更换到其他版本的工作空间，应同步调整并重新验证整个外部 SDK，不能混用现成静态库。

```bash
rg '^Eigen3_DIR:' "$RM_WS/build/rm_chassis_controllers/CMakeCache.txt"
rg '^#define EIGEN_(WORLD|MAJOR|MINOR)_VERSION ' \
  /usr/include/eigen3/Eigen/src/Core/util/Macros.h
```

### 第三步：构建、测试并安装外部 SDK

本仓库的驱动提供安装导出，目标名为 `RmTinyMPC::tinympc`。这是本项目自定义的包，不能假设上游自身已经支持 `find_package(TinyMPC)`。

```bash
cmake -S "$RM_WS/src/rm_controllers/tools/tinympc_external" \
  -B "$RM_TINY_ROOT/build-system-eigen" \
  -DTINYMPC_SOURCE_DIR="$RM_TINY_ROOT/TinyMPC" \
  -DEigen3_DIR=/usr/lib/cmake/eigen3 \
  -DRM_TINYMPC_EIGEN33_COMPAT=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DCMAKE_INSTALL_PREFIX="$RM_TINY_PREFIX"

cmake --build "$RM_TINY_ROOT/build-system-eigen" --parallel 2
(cd "$RM_TINY_ROOT/build-system-eigen" && ctest --output-on-failure)
cmake --install "$RM_TINY_ROOT/build-system-eigen"
```

安装目录应包含：

```text
lib/librm_tinympc.a
lib/cmake/RmTinyMPC/RmTinyMPCConfig.cmake
include/rm_tinympc/Eigen.h
include/rm_tinympc/tinympc/*.hpp
share/RmTinyMPC/LICENSE
```

安装内容不含另一套 Eigen。SDK 实现与所有消费它的代码必须使用相同 Eigen 和一致的对齐等 ABI 编译选项；不要给其中一方单独添加 Eigen 对齐宏或特殊向量化选项。

### 第四步：显式开启轮腿

完成外部 SDK 的构建与安装后，显式开启轮腿并指定 SDK 配置目录：

```bash
cd "$RM_WS"
catkin build rm_chassis_controllers --cmake-args \
  -DRM_BUILD_BIPEDAL=ON \
  -DRmTinyMPC_DIR="$RM_TINY_PREFIX/lib/cmake/RmTinyMPC"
```

此时配置阶段会查找 SDK，并使用 C++14 编译、链接包含 TinyMPC 头文件的检查程序，核对实际 Eigen 版本。失败会明确报错，不自动下载依赖，也不自动关闭用户选择的功能。

后续不需要重复填写这些参数：

```bash
catkin build
```

应生成 `devel/lib/librm_bipedal_controllers.so`。`BipedalController` 初始化时一次性建立 MPC 后端；Normal 地面反馈在当前周期同步求解，失败时请求进入 PROTECT，并将本周期关节命令清零。

如果 SDK 不存在，普通 OFF 构建仍然成功；`RM_BUILD_BIPEDAL=ON` 的构建应失败。`RmTinyMPC_DIR` 是包级提示路径，不需要全局增加安装前缀。

### 第五步：验证编译与加载隔离

默认 OFF 时，`build/rm_chassis_controllers/compile_commands.json` 中不应有 `/src/bipedal_wheel_controller/` 编译单元；公共库在 OFF、ON 两种模式下都不应定义轮腿控制器符号，也不应依赖轮腿或 TinyMPC：

```bash
nm -D --defined-only devel/lib/librm_chassis_controllers.so \
  | c++filt | rg 'BipedalController|VMCController'
readelf -d devel/lib/librm_chassis_controllers.so \
  | rg -i 'bipedal|tinympc'
```

以上两项预期没有匹配，`rg` 返回 1 在这里表示未发现相关条目。

新增的插件加载检查无需硬件，rostest 会启动测试用 ROS master，验证四个公共插件能创建且不加载轮腿插件；ON 时额外验证两个轮腿插件可以从新库创建：

```bash
catkin build rm_chassis_controllers --make-args chassis_plugin_load_test
source devel/setup.bash
rostest rm_chassis_controllers chassis_plugin_load.test
```

轮腿开启后，还应检查 `ldd -r devel/lib/librm_bipedal_controllers.so` 没有未解析依赖或符号。

本次构建隔离验收结果：

| 验收项 | 结果 |
| --- | --- |
| 默认 OFF | 构建成功，编译清单中轮腿源码数为 0 |
| ON | SDK 的 C++14 编译/链接检查及独立轮腿库构建成功 |
| 插件加载 | OFF 模式 1 个检查通过，ON 模式 2 个检查通过 |
| 公共库依赖 | 不定义轮腿控制器符号，不依赖轮腿库或 TinyMPC；公共源码编译参数不含 SDK 头文件目录 |
| 缺失 SDK | 开启依赖时报错，关闭时配置成功且不使用 SDK 路径 |
| 安装 | ON 安装轮腿库和头文件；OFF 向新前缀安装时均不包含这些内容 |
| 缓存 | 开启后不再传开关的构建成功，缓存仍为 ON |

**已有测试限制：** 修改前的 `bipedal_observation_test.cpp` 已因不存在的 `lqr10` 命名空间、`Config::position_release_tau`、`Config::hold_capture_speed` 等引用无法编译。本次保留该用例并使其随轮腿开关构建，没有删除或伪造通过结果。因此 ON 模式下完整 `catkin run_tests rm_chassis_controllers` 仍需先修复这个既有问题；新增插件检查和外部 SDK 数值检查应单独执行。

从旧版单库切换到本分支时，建议停用正在运行的控制进程，并一次性执行 `catkin clean rm_chassis_controllers` 后重建，以清除旧库。清理会清掉此包的开关缓存，轮腿机器需重新显式开启。日常构建不用清理。若主动从 ON 切回 OFF，旧轮腿产物或旧 install 前缀可能残留；做关闭验收时也应清理该包，并避免终端继续叠加旧工作空间。不要仅根据磁盘上存在旧 `.so` 判断本次编译开关。

### 复现原版失败和上游对照

为了确认没有通过遗漏 `codegen.cpp` 等文件绕过不兼容代码，可在另一个外部 build 目录关闭兼容替换：

```bash
cmake -S "$RM_WS/src/rm_controllers/tools/tinympc_external" \
  -B "$RM_TINY_ROOT/build-unmodified-system-eigen" \
  -DTINYMPC_SOURCE_DIR="$RM_TINY_ROOT/TinyMPC" \
  -DEigen3_DIR=/usr/lib/cmake/eigen3 \
  -DRM_TINYMPC_EIGEN33_COMPAT=OFF
cmake --build "$RM_TINY_ROOT/build-unmodified-system-eigen" -- -k -j2
```

最后一条命令预期编译失败，报告上述两个新版 API。不要安装这套产物。

上游内附 Eigen 的对照构建完全留在外部目录：

```bash
cmake -S "$RM_TINY_ROOT/TinyMPC" -B "$RM_TINY_ROOT/build-bundled" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$RM_TINY_ROOT/build-bundled" --target tinympcstatic --parallel 2
c++ -std=c++14 -O3 \
  -I"$RM_TINY_ROOT/TinyMPC/src" \
  -I"$RM_TINY_ROOT/TinyMPC/include/Eigen" \
  "$RM_WS/src/rm_controllers/tools/tinympc_external/smoke.cpp" \
  "$RM_TINY_ROOT/build-bundled/src/tinympc/libtinympcstatic.a" \
  -o "$RM_TINY_ROOT/build-bundled/tinympc_eigen_smoke"
(cd "$RM_TINY_ROOT/build-bundled" && ./tinympc_eigen_smoke > smoke.log)
(cd "$RM_TINY_ROOT/build-system-eigen" && ./tinympc_eigen_smoke > smoke.log)
```

两份日志中的 `EIGEN` 行应分别为 3.4.90、3.3.7；对照 `RESULT wide`、`RESULT bounded` 行的数值。代码生成文件位于各 build 目录的 `generated_wide/src/tiny_data.cpp`、`generated_bounded/src/tiny_data.cpp`，比较其数值部分时忽略生成时间注释。

### Normal 的 MPC 接入

只有 Normal 的地面 U4 反馈进入 MPC，输入顺序为 `[T_L, T_R, Tp_L, Tp_R]`，单位 N·m。左右轮总力矩默认各限幅 **±3.58 N·m**，虚拟腿力矩按当前要求不设限。后端用 `-infinity/+infinity` 表示未设置的单侧边界；拒绝 NaN、反向无穷大和倒置边界。

现有状态估计、腿长 PID、重力/横滚/弹簧补偿、VMC、跳跃及机械模式继续使用原路径。MPC 输出已经包含静态前馈，Normal 不再重复叠加。位移观测保留原逻辑：非 RAW、速度绝对值不超过 0.5 m/s 且速度指令绝对值不超过 0.01 m/s 时积分，其余情况清零。当前位移参考保持为零，保留原停车位置保持语义；只有 MPC 预测窗口内的位置参考按目标速度推进，不跨控制周期累计。航向参考仍按偏航角速度持续积分，并在预测窗口内继续推进。离开 Normal、重入和开始跳跃会清理后端热启动和旧结果。

参数放在 `chassis_controller` 命名空间下，以下即默认值，无需新增 YAML 即可采用这些 MPC 设置：

```yaml
mpc:
  horizon: 16
  wheel_torque_limit: 3.58
  rho: 1.0
  max_iter: 100
  abs_pri_tol: 0.0001
  abs_dua_tol: 0.0001
  max_solve_time: 0.008
```

继续使用已有十状态 `q`、四输入 `r` 和 `lqr_model` 物理参数；六状态旧配置仍需先迁移到当前控制器要求的十状态配置。MPC 在控制器初始化时固定采用当时的 Q/R、物理模型和 `default_leg_length`，运行期间不重建求解器或缓存。动态 Q/R 接口仍保留，但运行时更新不会应用到当前 MPC 后端；修改 MPC 使用的 Q/R、模型或其他 MPC 参数后需重新加载控制器。

Normal 直接调用 `mpc_backend::solve()`，使用本周期观测生成 `10×N` 的预测参考，并立即采用本次求解的首步总输入。预测采样周期固定为 **0.001 s（1 kHz）**。腿长、Q/R 和实测控制周期变化均不会触发重建；重新进入 Normal 也只清理热启动数据，继续使用初始化时的模型与权重。状态估计、位移及航向参考的跨周期积分仍使用实测 `chassis.dt`，静态姿态参考仍按当前腿长计算。控制器初始化时建立后端失败会使控制器加载失败；运行时发现后端缺失或未初始化则请求进入 PROTECT，不尝试重建。

已移除后台线程、请求/结果队列、代次标记及异步结果时效检查。旧的 `mpc/sample_period`、`length_tolerance`、`max_result_age`、`velocity_tolerance` 和 `yaw_rate_tolerance` 参数不再使用。

Normal 只接受本周期成功的 MPC 输出，不再计算 LQR 增益或回退输入。后端缺失或未初始化、求解未收敛、结果超时，或观测/参考/静态平衡点无效时，`computeFeedback()` 返回失败，由 `execute()` 请求进入 PROTECT，并清零本周期全部关节命令。反馈标志失效后不会将旧 MPC 输出作为当前命令使用；`NormalFeedback::active` 统一表示本周期 MPC 求解成功，仅在成功时置位，求解开始、失败或退出 Normal 时清零。现有兼容状态话题继续发布实际使用的 U4/F。

模型与缓存仅在控制器初始化时建立，控制线程中只同步求解，求解仍包含动态内存分配。`max_solve_time` 只检查 `solve()` 内部耗时，不能抢占求解；其默认 8 ms 也不是 1 ms 控制周期的实时保证。固定模型不随实际腿长变化更新，本实现尚未证明硬实时性或实机稳定性。虚拟腿不限幅不提供 VMC 后髋/膝关节的力矩保证。

接入回归可单独执行（不依赖已知过时的 `bipedal_observation_test`）：

```bash
catkin build rm_chassis_controllers --no-deps --make-args \
  bipedal_normal_mpc_test bipedal_mpc_backend_test
source devel/setup.bash
./devel/lib/rm_chassis_controllers/bipedal_mpc_backend_test
rostest rm_chassis_controllers bipedal_normal_mpc.test
```

回归覆盖真实 Normal 的同步后端输出与单次前馈、预测窗口内移动参考、行驶位移清零和停车漂移保持、腿长/权重/控制周期变化时继续使用初始模型与 1 ms 预测步长、未初始化后端保护、未收敛/超时保护、模式重入、轮力矩限幅及非法观测保护；后端测试同时覆盖轮受限而虚拟腿不受限的求解。
