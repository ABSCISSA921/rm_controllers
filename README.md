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

本次实现的是**编译隔离、外部依赖构建和 Eigen 兼容性验证**。`normal.cpp` 目前仍使用现有十状态 LQR；开启 TinyMPC 依赖不会自动把控制算法换成 MPC。算法接管的后续方案见本文末尾。

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

应生成 `devel/lib/librm_bipedal_controllers.so`。**此阶段只是准备并验证依赖，现有 Normal LQR 没有替换成 MPC。** 静态库中未被实际引用的对象可能不会进入最终 `.so`，不能把链接命令出现 TinyMPC 当成控制算法已经使用它的证据。

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

### 后续控制算法修改方案

按以下顺序推进，保持后续 TinyMPC 代码和编译要求仅作用于轮腿目标：

1. 在轮腿实现目录增加 MPC 后端；TinyMPC 头文件留在私有实现中，不放进公共底盘头文件。正常 OFF 构建必须始终不触及该代码和依赖。
2. 以现有 `linearModel()` 的连续模型为起点，非实时阶段生成固定腿长、固定采样周期的离散模型和缓存；统一十状态、四输入的顺序、正负号和单位。
3. 保留静态平衡点与前馈。若求解偏差状态和增量输入，明确 `z=x-x_eq`、`v=u-u_ff`，输出只加一次前馈，约束也转换到同一坐标系。
4. 初期仅替换 `Normal::computeFeedback()` 的四维地面反馈；保留状态估计、腿长 PID、重力/横滚补偿、VMC、跳跃和保护流程。先影子运行，再在仿真中接管输出。
5. 求解设置迭代上限、检查状态/有限值/时效/模型版本。模型、权重、腿长和采样周期变化时同步更新缓存，不能只换 A/B。模式切换作废旧解。
6. 分别测量求解分配、日志输出、最大时延和失败行为。上游默认实现不是已经证明适合当前实时线程的版本；确定执行线程、延迟补偿及经验证的 LQR 回退策略后再进入实机阶段。
7. 最终执行器约束仍需在 VMC 后检查，虚拟腿力矩受限不等于髋/膝力矩一定受限；完成闭环和时序验证后，才更新目标轮腿机器人的配置。其他机器人的 YAML 保持原样。
