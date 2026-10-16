# XMC18 双臂视觉抓取项目完整交接

整理日期：2026-10-08，Asia/Shanghai。

本文用于新会话接续。它是历史和当前代码的交接记录，不是执行机械臂的授权文件。
依据包括本会话历史、本地源码、SDK CHANGELOG、标定验证记录和持久化运行报告。
对没有完整原始日志支撑的早期操作，明确按历史记录描述，不宣称重新验证过。

## 0. 接手先读：当前状态与关键风险

1. **用户最后明确告知：机械臂已断开连接。** 此后没有再尝试连接或发送运动指令。
   当前任务是整理交接，不是启动运动。新会话先离线阅读；不要自动重连、回位或续跑历史轨迹。
2. 主项目是 `/home/arm/xCoreSDK-v0.5.1`，不是默认工作目录 `/home/arm/xCoreSDK-v0.7.2`。
3. 现在要做的是：**左臂识别假人左肘，夹爪正对目标，先关节运动到预抓取位，再保持姿态直线靠近，使两指之间的夹持中心到达目标。**
   已不是跟随棋盘格，也不是始终保持 40 cm 距离；40 cm 是第一段终点的退让距离。
4. 一开始拍摄一次静止观测（内部 10 帧），规划完整两段。第二段从第一段终点开始，不重新识别近景中的关节。
5. 2026-10-07 发生过真实接触：长距离单段 MoveAbsJ 靠近时夹爪碰到假人其他部位，用户急停。
   事后零间距执行被禁止。不能用“低速”“逆解通过”“现场说安全”混同于已完成全部验证。
6. **目前完整两段规划、局部模型连续逆解、执行状态机模拟均通过；完整两段实机执行尚未完成。**
   SDK 执行后端已编译，但在事故后的入口保护之后，不能通过现有命令直接运行。
7. 用户报告负载、重心已配置，碰撞检测已开启；但断开前 SDK 当前工具组返回：质量 0 kg、重心和惯量全零。
   SDK 工具组与 RL 工程工具不是一回事；不能据此断言示教器没配置，也不能猜负载覆盖它。
8. **整理时新发现 P0：返回观察位程序的速度有三套互相矛盾的表述。**
   `example/Test/return_observation.cpp` 实际 `command.jointSpeed=0.5`，即 **50%**；
   它的日志和 `return_to_grasp_start.md` 写 1%；`return_to_grasp_start.sh` 帮助写 10%。
   **这次只记录，没有更改参数。返回脚本不可按“1% 低速”直接运行。先核对并统一源码、日志、帮助。**
9. 没有真实夹爪 IO 闭合流程，不能宣称完成了闭合或成功抓取。
10. 右臂历史手眼验证没有通过有效放行；左右安装相似不代表标定可共用。当前视觉运动只考虑左臂。

## 1. 用户目标与工作方式

### 1.1 长期目标

深度相机识别抓取点和目标放置点，将坐标转换到机械臂坐标系；后续让大语言模型参考
任务代码库合成整体操作程序，完成定位、靠近、夹取、搬运或放置。

### 1.2 任务演化

- 起初比较 SDK 0.4.1b、0.7.2，后引入 0.5.1，最终当前开发使用 0.5.1。
- 相机驱动/OpenCV 显示、双臂连接、拖拽手眼标定、实时运动测试。
- 棋盘格视觉伺服、轴向辨识、软限位和奇异附近的运动问题。
- 提出棋盘格 demo：固定起始位，收到信号后识别，提前定位并靠近，夹爪闭合占位，抬升 20 cm、停 3 s、返回。
- 之后明确改为假人人体关节点识别，不再跟随标定板。
- 从膝盖、肩部的初期探索转到左肘；最终要求夹爪正对关节，两指中间到达目标，供下一步夹取。
- 单段关节运动发生接触后，改为“关节运动到预抓取位 + 固定姿态直线靠近”。

### 1.3 用户明确偏好

- 优先成功率，不要求从观察位到目标全程直线。
- 但靠近目标的最后一段希望路径可控，已认可先到预抓取位再直线靠近的方案。
- 不要每执行一段就重新识别，更不能把旧图伪装成新识别。
- 机械臂/假人位置发生变化后，需要重新获取配对观测和坐标转换。
- 单纯移动到世界坐标“上方”不是最终效果；末端必须正对抓取方向。
- 不要反复问已经回答的基础参数；需要什么信息先查本地，缺少物理数据再针对性询问。
- 添加 cpp 后希望自动发现、自动编译，可执行文件尽量与 cpp 同名。
- 要能区分官方示例、功能参数配置、任务参考代码和试验程序。

## 2. SDK 版本、控制器与选择依据

| SDK | 本地目录 | 本地 CHANGELOG 的兼容性与关键点 |
| --- | --- | --- |
| 0.4.1b | `/home/arm/xCoreSDK-v0.4.1.b/xCoreSDK-v0.4.1.b` | 0.4.1 基础要求 xCore >=2.2.1，部分特性 >=2.2.2；b 修复协作模式状态问题、增加写寄存器数组 |
| 0.5.1 | `/home/arm/xCoreSDK-v0.5.1` | xCore >=3.0.2；speed/zone 改 double，修复跟随限位与控制器不一致、监听恢复、工具工件查询等；当前项目使用版本 |
| 0.7.2 | `/home/arm/xCoreSDK-v0.7.2` | xCore >=3.2.2；注意 0.6.0 起的变更写明 xMateModel 不再支持 XMC18/XMC20 |

重要纠正：0.7.2 确实存在
`/home/arm/xCoreSDK-v0.7.2/lib/Linux/x86_64/libxMateModel.a`。
**库文件存在不等于其中仍支持 XMC18。** 不要再以“0.7.2 没有这个库”解释版本选择。
0.5.1 的 `include/rokae/model.h` 明列支持 XMC18，当前连续逆解和 Jacobian 检查依赖它。

用户最初控制器为 2.2.2，之后已更新；历史实机识别为 XMC18-R925-W4X3B1、控制器 3.0.4。
断开状态下没有重新读取控制器版本，重连时仍需核对。
不能把“头文件相似、程序可编译”当成对 0.4.1b 所有自定义功能的无条件替代认证。

手册：

- `xCore_SDK_C++使用手册V2.1_A.pdf`：早期参考，用户指出与旧版本对应。
- `xCore_SDK_C++使用手册V3.2_A.pdf`：后加入的新指南，曾讨论第 13、14、15 页实时内核内容。
- 研发反馈实时内核版本只是参考；用户明确**不要求必须 PREEMPT_RT**。

## 3. 硬件、网络、视觉运行环境

### 3.1 固定配置

来源：`example/function_config/robot_vision_config.hpp`。

| 项目 | 值 |
| --- | --- |
| 左臂 IP | `192.168.2.161` |
| 右臂 IP | `192.168.2.160` |
| 本机机械臂网口地址 | `192.168.2.6` |
| 当前左相机序列号 | `236422072262` |
| 当前右相机序列号 | `236422070786` |
| 当前历史实际双相机型号 | RealSense D435；重连后仍按序列号核对设备型号 |
| 棋盘格内角点 | `8 x 6` |
| 单格尺寸 | `0.0286 m`，即 28.6 mm，用户确认 |

早期用户单独接过 D455，序列号 `108222250630`；不要把它与后来左右臂的 D435 配置混为一谈。

### 3.2 相机和 OpenCV 的历史问题

- 早期能枚举 D455，但 `/tmp/realsense_d455_test` 等待帧时出现 `Frame didn't arrive within 5000`。
- 另一次 `cam_test` 能连接，但 OpenCV highgui 报 `The function is not implemented`，属于缺少 GUI 后端。
- 用户安装 `libgtk-3-dev pkg-config` 后继续处理 OpenCV 构建，之后明确反馈图像已可显示。
- 当前主要依赖放在 `.local`，不是早期 `/tmp` 目录。不要认为临时目录在重启后仍存在。
- RGB-D 正式采集使用 C++ `mannequin_snapshot`；历史 Python RealSense 直接采集不够稳定。

### 3.3 当前程序环境

```text
LD_LIBRARY_PATH=/home/arm/.local/xcore-vision/lib
PYTHONPATH=/home/arm/.local/share/xmc-pose-python
Python=/home/arm/论文/大语言模型/qwen_env/bin/python
YOLO weights=/home/arm/.local/share/xmc-pose-models/yolo11m-pose.pt
OpenCV C++=4.11.x
librealsense=2.55.x
```

Python 与 C++ 的 OpenCV 安装不是同一套；不要混用包路径。
YOLO 使用 GPU，当前本机查询为 RTX 3080 Laptop GPU，驱动 595.71.05。

### 3.4 实时内核历史与这次整理时的实际读数

曾尝试安装 `5.15.0-1032-realtime`，NVIDIA 595.71.05 DKMS 构建报错；用户后要求撤销/重新选择兼顾 GPU 的方案。
不能把曾下载过 realtime 内核当成现在正在运行 PREEMPT_RT。

本次整理实际读数：

```text
uname -r: 6.8.0-134-lowlatency
当前工具终端 ulimit -r: 0
当前工具终端 ulimit -l: 4080488 KB
cpu0 scaling_governor: powersave
nvidia-smi: NVIDIA GeForce RTX 3080 Laptop GPU, 595.71.05
```

这是本次会话/终端的读数，不代表其他登录会话的 PAM 权限完全相同。
历史执行过 `cpupower frequency-set -g performance` 也不能保证重启后仍是 performance。
当前“假人两段靠近”是 NrtCommand 的 MoveAbsJ + MoveL，不是上位机 1 ms 视觉闭环。
未来重新做 1 kHz 实时控制，需要另测调度权限、延迟、丢包以及 GPU 负载共存；没有硬实时达标证据。

## 4. 哪些是原本的，哪些是后加的

### 4.1 来源分类原则

不要把“在旧 SDK 目录内”理解成“官方示例”。0.4.1b 目录已经混入大量用户自己的代码和后续修改。
当前没有可用的干净 Git 基线来证明每一行的作者和修改时间；以下是依据目录职责、历史和文件内容的来源分类，
不是对所有官方文件逐字未修改的保证。

### 4.2 SDK 原始主体与官方示例

SDK 的 `include/rokae/`、`lib/`、`external/`、官方 CHANGELOG、发行 README、手册属于 SDK 来源。
根 `CMakeLists.txt` 仍是 SDK 工程骨架：C++17、lib/include/external/example/doc 等。

`example/original_cases/` 是迁移归类后的官方案例目录。**目录分组是项目后加的，里面这些 cpp 的用途是保留官方示例。**

```text
force_control_commands.cpp
get_keypad_state.cpp
modbusRTU_endtool_control.cpp
move_example.cpp
path_record.cpp
read_robot_state.cpp
rl_project.cpp
sdk_example.cpp
xmatemodel_er3_er7p.cpp
rt/cartesian_impedance_control.cpp
rt/cartesian_s_line.cpp
rt/follow_joint_position.cpp
rt/joint_impedance_control.cpp
rt/joint_position_control.cpp
rt/joint_s_line.cpp
rt/move_commands.cpp
rt/rt_industrial.cpp
rt/torque_control.cpp
```

`original_cases/AI_PROMPT.md` 是后加的 AI 使用提示，不是厂商说明书。
`function_config/print_helper.hpp` 是 SDK 风格打印辅助头文件的重用/归位，不是新抓取算法。
官方示例可能直接上电运动；“官方”不意味着可以不审读就运行。

### 4.3 0.4.1b 中原项目已有/历史自定义代码

旧目录 `example/` 中包括：

- `hand_eye_calibration.cpp`、`tuozhuai-biaoding.cpp`、`shouyanbiaoding-11-4.cpp`：不同版本的手眼标定程序。
- `catch_info.cpp`、`cam_test.cpp`：历史数据/相机相关程序。
- `back-home.cpp`、`move.cpp`、`4-16.cpp`、`test.cpp`：历史运动/试验程序。
- `rt_fellow.cpp`、`rt_fellow-new.cpp`：历史实时跟随代码。
- `guided_pick_test.cpp`、`hand_eye_diagnostics.hpp` 和距离控制等说明：旧项目内后续功能代码/说明，不能认作官方发行样例。

**标定程序名称相似，但参数不一致，不能互换运行：**

| 旧文件 | 当前文件内参数 | 备注 |
| --- | --- | --- |
| `hand_eye_calibration.cpp` | 7x5、20 mm | 不匹配现在 8x6、28.6 mm 的板 |
| `shouyanbiaoding-11-4.cpp` | 9x6、20 mm | 同样不匹配当前板 |
| `tuozhuai-biaoding.cpp` | 8x6、28.6 mm | 此文件已在后续按用户测量修改；输出 `LeftArm/RightArm_interactive_calib.yml`，路径取决于运行工作目录 |

用户要求参考旧程序并保持机械臂 IP；不是要求把全部旧代码照搬为官方内容。

### 4.4 0.5.1 的功能工具/配置目录：后加和迁移内容

`example/function_config/`：

| 文件 | 来源/用途 |
| --- | --- |
| `robot_vision_config.hpp` | 后加统一 IP、序列号、8x6/28.6 mm 板参数 |
| `drag_hand_eye_calibration.cpp` | 参考旧标定流程适配 0.5.1 的拖拽标定工具；会改变拖拽/机器人模式，不是纯读程序 |
| `tcp_four_point_calibration.cpp` | 后加 TCP 四点标定工具；存在不等于当前 TCP 已完成四点标定 |
| `visual_servo_config.hpp` | 后加视觉伺服、工作区、限位/奇异、工具长度等参数 |
| `board_pick_demo_config.hpp` | 后加棋盘格抓取 demo 配置 |
| `LeftArm_interactive_calib.yml`、`RightArm_interactive_calib.yml` | 用户实测标定结果，不是官方默认参数；有效性左右不同 |
| `left_gripper_geometry.json` | 本轮最新添加，记录用户的 300/150/20/100 mm 几何数据及未验证项 |
| `tool_load_readback.cpp` | 本轮最新添加，只查询 SDK 工具组与当前加载工程的工具列表；已编译，但因用户断开，尚未实机运行此新工具 |
| `AI_PROMPT.md` | 后加目录使用提示；其中部分临时库路径和阶段性说明已过时，按当前源码/本文核对 |
| `calibration_runs/` | 后加标定验证报告、配对 RGB-D/姿态、校准快照 |
| `recorded_poses/` | 后加观察位和手动成功案例姿态 |
| `calibration_backup_*`、`recalibration_*`、`invalid_20mm_*` | 历史备份/采集/失效结果归档，不能当活跃标定直接恢复 |

0.5.1 的拖拽工具当前输出名是 `<ArmName>_drag_hand_eye_051.yml`，通常写入运行工作目录；
不要误以为会自动覆盖当前 `*_interactive_calib.yml`。替换活跃标定前先备份、核对相机、板参数和验证结果。

### 4.5 Test：实际任务与试验程序，均非官方示例

| 文件/组 | 作用与状态 |
| --- | --- |
| `back.cpp` | 历史双臂 HOME 返回，设置默认速度 100；不是当前左臂观察位返回，不要混用 |
| `record_point.cpp` | 当前是空文件，被自动扫描跳过，不是可用点位记录器 |
| `rt_motion_test.cpp` | 后加实时小幅正弦运动测试，可选择左右/依次双臂；默认参数就可能运动，不能当只读测试 |
| `visual_servo_follow.cpp` | 后加棋盘格视觉伺服、轴向辨识、候选位姿扫描/准备、工具组查询等；不是当前假人两段抓取主入口 |
| `board_pick_demo.cpp` | 后加信号触发棋盘格抓取 demo；夹爪 IO 仍为占位，不能宣称实夹成功 |
| `mannequin_snapshot.cpp` | 后加 C++ RGB-D + 同时段机器人姿态采集；静止前后检查，不是硬件同步采样 |
| `mannequin_joint_preview.py`、`mannequin_joint_stability.py` | 后加姿态关节点预览与稳定性评估 |
| `mannequin_joint_base_preview.cpp/.py` | 后加相机点到基坐标预览 |
| `mannequin_joint_pipeline.py` | 早期假人识别/转换入口之一；先读再用，非当前全流程入口 |
| `generate_joint_marker.py`、`mannequin_marker_preview.py`、`mannequin_knee_marker_A4.*` | 初期建议膝盖贴标记的生成/预览资源；用户不便打印，当前主线仍是 YOLO 关节 |
| `mannequin_micro_approach.cpp/.py` | 早期微步非接触靠近工具；不是最终两段执行器 |
| `mannequin_pregrasp_geometry.py` | 表面法向估计等预抓取几何辅助 |
| `left_shoulder_micro_approach.py` | 历史名字仍带 shoulder，实际支持 `--joint left_elbow`；当前视觉采集/识别/规划分发主入口 |
| `mannequin_approach_plan.cpp` | 当前主规划器；含旧非接触执行、历史诊断、新两段规划/回放以及仍受保护的 SDK 两段后端 |
| `mannequin_pose_diagnosis.cpp` | 位姿/几何诊断；旧工具对 normal 元数据有要求，不能任意拿 demonstrated 几何喂入 |
| `return_observation.cpp`、`return_to_grasp_start.sh/.md` | 固定左臂观察位返回；当前存在 50%/1%/10% 速度矛盾，优先核对 |
| `plan_left_elbow_full.sh` | 当前一键完整两段规划入口，仅规划/模拟/一致性验证，没有运动权限 |
| `validate_full_plan.py` | JSON/CSV 离线一致性检查，不访问机器人或相机 |
| `test_mannequin_approach_args.py` | 命令行组合、只读和执行禁止回归测试 |
| `test_mannequin_pregrasp_geometry.py` | 合成深度数据的法向几何测试 |
| `test_validate_full_plan.py` | 规划清单、交接姿态、目标/TCP、历史诊断标签等离线测试 |
| `test_staged_motion.cpp` | 本轮最新添加，执行监督状态机的纯模拟正常/故障测试 |
| `mannequin_joint_approach.md` | 当前假人任务流程与限制说明，本轮已补两段监督和负载问题 |
| `INCIDENT_20261007.md` | 碰撞事故分析、证据边界、执行禁止原因 |
| `reports/` | 后加观测、规划、执行日志和模拟报告；不是源码，不代表可执行计划授权 |

### 4.6 task_library：后加的 AI 参考与共用逻辑

- `PERCEPTION_TO_ROBOT_CONTRACT.md`：感知到机械臂数据约定。
- `VISUAL_SERVO_DESIGN.md`：棋盘格伺服设计及历次约束。
- `BOARD_PICK_DEMO.md`：棋盘格 demo 设计。
- `AI_PROMPT.md`：AI 任务合成提示，本轮补充禁止绕过事故保护、模拟不等于实机通过。
- `staged_motion.hpp`：**本轮最新添加**，不依赖 SDK 的两段执行监督状态机。
- `staged_motion_replay.hpp`：**本轮最新添加**，虚拟时钟和合成遥测后端，非物理动力学模拟器。
- `staged_motion_sdk.hpp`：**本轮最新添加**，同一状态机的 SDK 接口后端，已编译但尚未实机验证/放行。

这些 header 本身不是 main 程序，不能直接执行。

### 4.7 组织、构建与生成文件

- `example/CMakeLists.txt` 已改为自动递归扫描四个目录：`original_cases`、`function_config`、`task_library`、`Test`。
- `example/README.md` 仍写三个目录且 task_library 不编译，是旧说明，不是当前实际 CMake 行为。
- `task_library/AI_PROMPT.md` 前部也留有“不自动编译”的旧文字；准确规则是：四个目录内完整 main cpp 都会被扫描，header 不单独生成目标。
- `.vscode/tasks.json` 与 `launch.json` 当前仍是 gcc/g++ 单活动文件模板，不会自动带 SDK 依赖和环境；不是可靠 CMake 调试入口。
- 根目录和 `example/` 内还有旧的 CMakeCache/CMakeFiles/Makefile/bin 等构建产物。当前采用 `build/`，不要混用旧 binary。
- `logs/`、`_rokae_log_/`、RGB-D 图片、标定结果和 reports 为运行生成内容。
- 本文及 `NEXT_SESSION_PROMPT.md` 是本次整理新加的交接文档；本次未修改机器人运动速度。

## 5. 构建规则与可靠的离线命令

当前 `build/CMakeCache.txt`：Release、`/usr/bin/g++`、`XCORE_USE_XMATE_MODEL=ON`、
`XCORE_LINK_SHARED_LIBS=OFF`，OpenCV/realsense2 均指向 `.local/xcore-vision`。

实际自动扫描规则：

1. 四个目录递归扫描 `*.cpp`，使用 `CONFIGURE_DEPENDS`。
2. 通过源码正则寻找 main。空文件/没有 main 的 cpp 跳过；main 检测是启发式，不是完整 C++ 解析器。
3. 输出文件名使用 cpp 的无扩展名；CMake 内部 target 会把非 ASCII/符号替换成下划线，纯不可用名称使用哈希。
4. 同名或净化后冲突的 target 配置时报错，不会悄悄改成目录前缀。
5. 模型、视觉依赖也是源码字符串启发式识别。仅在间接 header 里引用依赖时，可能需要调整识别规则。
6. 当前每个 main cpp 建一个可执行文件，不是自动把多个 cpp 组合成一个多文件程序。
7. 新增目标后直接 `cmake --build ... --target 新名字` 可能提示没有规则；先重新配置即可，不必逐文件改 CMakeLists。

```bash
cd /home/arm/xCoreSDK-v0.5.1
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DXCORE_USE_XMATE_MODEL=ON -DCMAKE_PREFIX_PATH=/home/arm/.local/xcore-vision
cmake --build build --target mannequin_approach_plan mannequin_snapshot test_staged_motion tool_load_readback -j2
build/bin/test_staged_motion
env LD_LIBRARY_PATH=/home/arm/.local/xcore-vision/lib PYTHONPATH=/home/arm/.local/share/xmc-pose-python /home/arm/论文/大语言模型/qwen_env/bin/python -m unittest discover -s example/Test -p 'test_*.py' -v
```

上述编译和测试不需要机械臂。不要把 `rt_motion_test` 或 `back` 当离线单元测试运行。

## 6. 标定、工具与坐标约定

### 6.1 标定历史

- 相机曾受到碰撞，用户要求左右臂重新手眼标定。
- 用户先后重标定多次；中间发现棋盘格真实边长是 28.6 mm，不是旧 20 mm。
- 以 20 mm 对这块板算出的结果不可用于运动；相关文件已归档在失效目录。
- 当前活跃左标定：`example/function_config/LeftArm_interactive_calib.yml`。
- 左侧验证参考：`example/function_config/calibration_runs/left_validation_20260930_i2JHhp/calibration_used.yml`。
- 主规划器会比较活跃标定与该验证快照，不能随意替换后绕过一致性检查。
- 左标定三姿态、每姿态 10 帧验证：PnP 板中心跨姿态差约 5.11~7.16 mm；深度法约 2.50~5.72 mm。
- 姿态覆盖含不同旋转方向，记录里平移可观测性条件数约 1.897。
- 这些证明了测试范围内的一致性，不是绝对抓取精度、完整工作区精度或碰撞安全认证。
- 右臂历史验证未通过有效放行。当前右 yml 虽存在，不能因为左右相机装得相似就直接使用。
- 历史 `*_axis_verified.yml` 开过/用过，不代表现在仍存在或适用于碰撞重装后的标定。以当前文件和验证来源为准。

### 6.2 变换链

`H_Flange2Cam` 在本项目使用为 `flange_T_camera`：把相机坐标中的点映射到法兰坐标。

```text
p_base = base_T_flange * flange_T_camera * p_camera
T_world_flange = world_T_base * T_base_flange
p_grasp_base = p_flange_base + R_base_flange * [0, 0, 0.300]
p_flange_goal = p_target_base - R_goal * [0, 0, 0.300]
```

注意 SDK 指令参考/世界坐标和机器人基坐标并不相同。
左臂历史读回的基座在世界中的姿态为 `[0,0,0, 0,pi/2,0]`，旋转矩阵约：

```text
world_R_base = [ 0 0 1
                 0 1 0
                -1 0 0 ]
```

当前方案要求 SDK 活跃工具和外部参考的位姿为零；软件自己补偿 300 mm 工具长度。
如果之后把 SDK TCP 也设为 300 mm，必须同步改变变换/指令约定，不能双重补偿。
同理，不能随意选择带非零 TCP 的命名工具来“修复负载”，而不检查坐标影响。

仅底座/平台移动但相机相对法兰没动，通常不用重做手眼外参，但必须重新识别并按新位姿转换。
相机相对法兰受撞移动则需要重新验证/标定。用户此前明确过一次只是整机移动。

### 6.3 工具测量的演进与当前值

- 曾有“相机到前端约 28~30 cm”的粗估。
- 后来用户说“法兰安装面到夹持中心约 35 cm”，并确认同轴、沿法兰正 Z。
- 再后来用户确认 **300 mm**，当前 `kLeftFlangeToGraspCenterM=0.300`。
- 最新补充：张开最大外宽 150 mm、最大厚度 20 mm、指尖超过夹持中心 100 mm。
- 这些已写入 `example/function_config/left_gripper_geometry.json`。
- 相机光轴方向粗估 `kEstimatedToolProtrusionFromCameraM=0.30` 是另一项旧参数，不能与法兰工具长度混为一谈。
- 150 mm 是用户对最大张开宽度问题的回答，不应自动等同于已测的两指净开口；完整手指截面、内开口和相机/转接板外形仍没有完整模型。
- 指尖比夹持中心前伸 100 mm，意味着中心到达目标表面时，指尖可能已经越过该表面 100 mm；这对最后靠近避碰重要。
- 用户说相机画面左右方向对应夹爪移动方向，安装关系固定。未来姿态/夹持方向可参考，但还不是完整机械 CAD。

### 6.4 40 cm 的含义

预抓取退让距离：`0.20 m 非接触距离 + 0.10 m 工具不确定性 + 0.10 m 感知余量 = 0.40 m`。
这是当时人为设置，不是视觉转换误差。用户问末端为何还很远，并测得约 40 cm，与该参数一致。
最终目标不是再停 40 cm，而是第二段沿固定工具轴靠近 40 cm 到目标夹持中心。

## 7. 起始位、手动成功姿态与返回

### 7.1 固定观察/抓取起始位

文件：`example/function_config/recorded_poses/20261006_143947_before_manual/left.yml`。
不是 SDK HOME，也不是机械臂当下随机姿态。

```text
q_rad = [-0.6079918871189015, -1.5877366279098633, -0.5048874856742348,
         -3.2336080117821173, 2.3175952192481764, -0.9238617407934551]
flange_base_xyz_m ~= [-0.53680305, 0.54515295, 0.58453864]
```

### 7.2 手动肘部成功案例

文件：`example/function_config/recorded_poses/20261006_manual_elbow_analysis/left.yml`。
用户手动把夹持中心对到假人左肘，之后又演示了正对左肩方向、末端触到左胸的姿态。
这些支持“附近目标可以通过合适姿态到达”，不能据此保证从任意观察位的特定 MoveL 都可达。

当前文件肘部 q 约：

```text
[2.3630051252, -0.6870046809, -0.1373840384,
 -5.4734711514, 1.1643419914, -1.1468457421]
flange_base_xyz_m ~= [0.51919475, -0.24817393, 1.15524955]
flange_positive_Z_base ~= [0.99868329, -0.05092350, 0.00620344]
```

当前 demonstrated 方案取这个手动成功案例的旋转，不是重新拟合衣服表面法向。
该轴转换到世界后接近向下，但不等于强制世界 Z 直线。

### 7.3 返回入口

`return_to_grasp_start.sh` 读取上述固定观察位，不看相机。`--plan` 仍需连接机器人，不是纯离线。
`--execute` 会运动；**当前速度参数不一致，见首页 P0，修正/核对前不要运行。**
不要用 `back.cpp` 替代，它是不同的双臂 HOME。
过去一次从随机低姿态返回被工作区检查挡住，不能因此任意扩大边界或自动沿历史轨迹倒放。

## 8. 当前识别与规划流水线

入口 `example/Test/plan_left_elbow_full.sh`：

1. 编译主规划器和 C++ 快照工具，创建唯一 `reports/full_plan/run_*` 目录。
2. 加载 `yolo11m-pose.pt`，左相机采集一次 10 帧 RGB-D；同时记录机械臂关节、法兰位姿、内参和时间戳。
3. 用 COCO 左肘 index 7（左肩 index 5）识别，融合深度反投影，取三维点中位数。
4. 验证至少 8/10 可靠帧，person confidence >=0.5，keypoint >=0.9，位置 P90 <=20 mm，深度邻域扩散 <=40 mm。
5. 将相机点用配对姿态和手眼转换到机器人基坐标，锁定该次目标。
6. 第一步：求 controller 终点 IK、选合法近邻等价关节圈数，规划到 40 cm 预抓取位的关节插值路径。
7. 第二步：从第一步精确的关节终点起，保持姿态，沿接近方向走 40 cm 到零中心间距。
8. 检查 FK、真实软限位余量、相邻 q 连续性、工作区、Jacobian、两臂静止状态。
9. 两段全部通过才提交 `.full_plan.json`；再用实际本地 FK 模型回放新执行状态机。
10. Python 对 JSON/CSV 做离线一致性检查。**以上没有运动下发。**

细节：常规规划要求快照新鲜（C++ 30 s；Python 加载快照 15 s）；capture timestamp 以字符串保存，避免整型精度问题。
采集和规划会连接 SDK；所谓“只读”指没有显式模式/上电/运动命令，不保证 SDK 构造函数完全没有副作用。
SDK 文档写明 Robot 初始化会做运动队列重置，因此也应只在空闲、无其他控制程序时使用。

### 8.1 第二段失败原因与已做修复

旧规划对每个虚拟点调用 controller `calcIk`，但默认选解参考的是**实际当前关节**，不是虚拟的上一规划点。
机械臂还停在观察位时，不能把这等同于“第二段从第一段终点连续求解”。
曾在第二段第 71 个新点（约前进 284 mm，还余 116 mm）出现 controller IK 错误；局部模型能继续解出。
这不是实机第二段已经运动 284 mm 的证据。

已改为：

```text
先对第一段终点做 local IK/FK 往返核对
第 1 个新点 seed = 第一段关节终点
第 n 个新点 seed = 前一个通过检查的关节解
model.getJointPos(base_T_flange, 0.0, seed, solution)
solver 输入使用主值角，输出展开到上一物理关节值附近
任何点失败就整条规划失败，不再回退到以实际观察位为参考的 controller calcIk
```

第 0 点保留第一段精确关节终点；之后 100 个新点（每步约 4 mm）连续求解。
这修复的是规划 seed 链；**不代表已修复厂商内部算法或实际 MoveL 已通过控制器验证。**
没有调用 `setDefaultConfOpt(true)`；轴配置 confData 与显式数值 seed 不是一回事。

### 8.2 新执行状态机与实机后端

`staged_motion.hpp` 的目标流程：

```text
校验计划和起点 -> 准备 -> 第一段下发 -> 遥测监控
-> 匹配该命令的到位事件 + idle 端点稳定至少 0.3 s
-> 再检查实际关节/位姿和另一臂 -> 第二段下发 -> 遥测监控
-> 终点核对 -> stop/power off/idle/manual -> 确认终态
任一步失败 -> 停机清理；不发后续阶段，不自动返回/恢复急停
```

包含关节路径/分支误差、限位、直线姿态、速度、平面距离、另一臂、工具/参考、状态、电源、通信时间间隔和超时监控。
第一段关节速度计划 0.01（1%），第二段 5 mm/s，阶段末停稳，无夹爪动作。

SDK 后端额外检查非零有限负载，运行中不能变更负载，关键下发前检查取消信号。
清理按尽力执行 stop、power off、moveReset 清队列、Idle、manual；每项错误都保留。
SDK `moveAppend` 的成功仅代表发送前/发送阶段没报错，不能直接认作轨迹可执行或执行完成。

**现有 `stagedPlan && execute` 入口仍直接拒绝，早于文件读取和机器人连接。**
后面源码中的 `if(execute) { SdkBackend ... }` 分支只是已接入和编译的实现，当前不可达。
不能以“代码有这个类”为由说实机执行器已验证，也不能从其他文件直接实例化它绕过入口保护。
该主机监控不是安全认证的实时碰撞控制器；阻塞式 SDK 通信期间的停机保障仍需硬件安全系统。

## 9. 真实运动、碰撞与最新证据

### 9.1 真正做过的运动与没做过的运动

- 历史右臂棋盘格跟随、轴向辨识有成功阶段；之后相机碰撞、重标定，不能沿用为当前标定有效性证据。
- 左臂做过非接触靠近、回观察位等试验；用户也多次手动移动/示教到肘、肩胸附近。
- 单段零退让长 MoveAbsJ 发生接触，用户急停。
- 2026-10-08 单独第一段曾以 1% 成功到 40 cm 预抓取位。
- **新的完整两段没有实机跑通，第二段没有依据新完整流程执行成功的记录。**
- 最新两段完整通过是规划与模拟回放，不是再次真实运动。

### 9.2 事故摘要

原来的 40 cm 预抓取：工具估计 350 mm，J1 约 +107°、J4 约 -70°，MoveAbsJ 1%。
事故那次：改成工具 300 mm、零退让，重新从观察位求一个长 MoveAbsJ，J1 约 +174.6°、J4 约 -136.9°，速度 0.5%。
不是在原来轨迹后追加直线 40 cm，而是整个关节路径变化，工具产生大圆弧。

事故时 611 个运动学采样点通过，也未检测出轨迹里工具与假人其他部位的碰撞。
当时没有完整手指、手掌、连杆、线缆与环境的避碰模型。
最后进度日志 83.7936% 不能定位接触时刻。清理返回 `robot is stopping (769)`、`EStop ... (-3)`。

完整分析见 `example/Test/INCIDENT_20261007.md`。
原 `/tmp/xmc_left_elbow_300mm_align_execution_20261007.log` 后来已不存在，事故文档是从此前读到的内容保留的摘要，不是完整恢复日志。
不能断言事故必然来自转换错误、控制器故障或实际运行时跳 IK 分支；证据支持的是轨迹/安全验证不足。

### 9.3 可用报告目录

以下均相对 `example/Test/reports/`：

| 目录/文件 | 证据意义 |
| --- | --- |
| `staged_20261008/stage1_execution_1pct.log` | 第一段真实执行成功；与当前返回脚本的速度不一致问题是两回事 |
| `full_plan/run_osHeopbP/` | 历史完整规划；旧 v1 混合 controller/local 求解记录及后加 seeded diagnostic |
| `full_plan/run_ebwpzhmk/planning.log` | 一次新识别 P90 21.7 mm >20 mm，按规则停止，没有运动 |
| `full_plan/run_PonCeHot/` | 新识别 P90 11.71 mm，408 点第一段 +100 新点第二段通过，仅规划 |
| `full_plan/run_169NSGdK/` | 最新接入状态机后的完整新观测、规划及 FK 回放通过；未下发运动 |

最新目录的文件前缀：`left_elbow_1791446398313244278`。

```text
左肘可用帧: 10/10
深度: 1.572 m
P90: 15.5925 mm
person confidence: 0.93476
keypoint confidence: 0.98344
target_base ~= [0.791558, -0.177990, 1.103950] m
第一段: 408 采样点
第二段: 101 点，含起点和 100 个新解
minimum Jacobian sigma ~= 0.0954951
maximum Jacobian condition ~= 19.2514
```

最新 `.supervisor_replay.txt` 明确写：虚拟遥测/时间、SDK 指令数 0、Controller MoveL NOT TESTED、Collision NOT VALIDATED。
这些现在都属于历史现场记录，不能因文件存在就重新执行。

### 9.4 已跑过的离线测试

最近代码构建成功：`mannequin_approach_plan`、`mannequin_snapshot`、`test_staged_motion`、`tool_load_readback`。
离线测试 57 项：C++ 状态机模拟 27 项；Python 30 项（参数 13、几何 3、计划清单 14）。
另外有 launcher shell 语法检查、工具配置 JSON 解析、真实历史 plan JSON/CSV 一致性检查。

测试证明范围：参数、数据契约、模拟状态/故障处理、部分几何和一致性。
未覆盖真实机器人惯性/停止距离、SDK 后端实际通信时序、碰撞检测有效性、所有连杆扫掠空间和真实夹爪闭合。

## 10. 下一会话应按什么顺序继续

### A. 先做不接机器人的工作

1. 阅读本文、事故报告、最新规划日志，以及三个 `staged_motion*.hpp`。
2. 优先核对返回程序实际 `jointSpeed=0.5` 的来源/意图，统一源码、日志与帮助；不要凭历史说它仍是 1%。
   不要擅自恢复用户更改；明确安全和速度意图后修改，并测试。
3. 审核新 SDK 后端：现在仅编译，没实机覆盖；通信阻塞、取消窗口、到位事件、停机/清队列失败必须逐项检查。
4. 需要时运行离线构建/57 项测试，确认工作树没有更新影响结论。
5. 工具几何 JSON 只是记录数据，不是已实现的完整避碰检测器；不得把里面 `robot_motion_permitted` 改 true 当作放行。
6. SDK 原程序、后加测试程序、旧 build/bin、临时文件要分清；不要清除不属于自己的改动。

### B. 用户重新连接后，先解决具体已知差异

1. 确认机械臂当前空闲状态、实际姿态、控制器版本、另一臂状态和相机安装。不要先自动回起始位。
2. 运行新只读工具组查询（当前尚未实机运行的新工具）：

```bash
cd /home/arm/xCoreSDK-v0.5.1
env LD_LIBRARY_PATH=/home/arm/.local/xcore-vision/lib build/bin/tool_load_readback --left
```

3. 它查询 active SDK toolset 与当前加载 RL 工程的 toolsInfo，不会查遍所有未加载工程，也不能证明碰撞检测已开启。
4. 先找到用户实际设置的质量、重心、惯量及命名工具；区分 SDK 工具组与示教器/RL 工程参数。
   没有确切值不要用假质量补齐；不要为消除零负载报错而随意改参数。
5. 保持 TCP/参考坐标约定一致，不能在修复负载时造成 300 mm 双重补偿。
6. 用户说碰撞检测开启，应核对其实际配置与生效状态；不擅自降低灵敏度或关闭它，也不自行恢复急停。

### C. 完成实际路径和接近验证

1. 工具宽/厚/前伸已记录，利用现有深度数据/补充机械数据评估手指、掌部、相机/法兰、连杆、另一臂及线缆的扫掠。
   外形尺寸不等于完整内开口和碰撞模型；不能凭工具中心点不越平面就宣称全工具无碰撞。
2. 接触后若安装发生变化，重新验证手眼/TCP。已经有左侧三姿态验证，不要无理由重做同样静态测试。
3. 在正确起点重新做一次新鲜观测，完整规划两段，仍保持同一目标和 seed 链。
4. 验证实际控制器从第一段真实终点接受第二段 MoveL，并且连续关节轨迹与模型规划一致；出现错误保留原文和代码。
5. 完成执行入口的正式审查/放行逻辑，不是简单删除 `require(!(stagedPlan && execute), ...)`。
6. 受控实机验证后记录两段真实到位、误差、状态、清理结果与实测夹持中心位置，才能说“可运行/已跑通”。
7. 最后再接入用户后续提供的夹爪 IO 与闭合反馈；当前不能生成假 IO 或虚构抓取成功。

### D. 仍不应做的事

- 不自动用旧识别、旧照片或历史 diagnostic plan 运行。
- 不在第二段前重新识别近景目标并悄悄换目标。
- 不把右臂 yml 文件存在视作右臂标定已通过。
- 不混用 `back` 的 HOME 与用户固定观察位。
- 不因逆解失败就放宽软限位/奇异阈值，或通过调 TCP 长度掩盖路径错误。
- 不把 0.005 解释为 5%：jointSpeed 0.005=0.5%，0.01=1%，0.5=50%；MoveL speed 是另一单位 mm/s。
- 不把模拟成功、发送 API 成功或“所有采样点有解”说成实机已完成/无碰撞。
- 不把当前目标缩成永远停 40 cm；完整夹持定位仍是待完成目标，但必须诚实报告真实阻碍。

## 11. 新会话启动摘要

请结合根目录 `NEXT_SESSION_PROMPT.md` 使用。
本次只完成交接整理，没有机器人连接、运动或参数写入。
任务“解决所有问题，直到可以运行”尚未完成，不能以整理文档、通过模拟或相机断开而标记为完成。
新会话必须以最新源码/实机状态为准；本文中的历史成功、现场确认和姿态记录不是永久有效的启动条件。
