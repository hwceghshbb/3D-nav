# SONIC 遥操 Mod

SONIC 已完整封装在 `mods/com.bxi.sonic` 中。普通仿真和硬件 launch 不需要再启动
`example_sonic_sim2sim.launch.py`，也没有独立的 `pico_runtime.py` 监管层。

Mod 负责一个参数化控制状态、SONIC 模型推理、PICO 数据接入、POSE 到 SMPL reference 的
转换以及可选夹爪命令；MuJoCo、机器人平台 I/O 和主控制器仍由宿主负责。

## 数据路径

```text
PICO 头显/追踪设备
  -> RoboticsServiceProcess
  -> xrobotoolkit_sdk
  -> pico_manager（ZMQ pose）
  -> smpl_bridge（ZMQ smpl_ref）
  -> SonicTeleopPolicy
  -> 29 个具名策略关节
  -> MotorFrame
```

`SonicTeleopPolicy` 使用统一 `ModelSpec.portable_onnx()`，推理后端顺序是：

```text
RKNN -> OpenVINO -> ONNX Runtime
```

模型和 idle reference 分别来自 `assets/sonic.onnx` 和
`assets/stream_reference.npz`。

## 状态和事件

- `com.bxi.sonic/sonic_teleop`：控制机器人本体的 29 个策略关节；是否控制夹爪由
  `hardware_gripper` 参数决定。
- `com.bxi.sonic/activate`：默认 `btn_10=9`。
- `com.bxi.sonic/reset_alignment`：默认 `btn_9=2`。

SONIC 明确声明 ELF3 的 29 关节模型布局。框架按关节名映射到机器人布局；机器人额外
关节由其他命令来源或显式 defaults 提供，不依赖数组位置猜测。

## 框架生命周期

`mod.yaml` 把两个组件放在各自正确的运行边界：

```text
ModNodeManager
├── pico_manager
│   └── 独立厂商 Python：pico/manager_launcher.py
└── smpl_bridge
    └── 宿主 executor 内的原生 rclpy.Node
        depends_on: pico_manager
```

两个节点均为 `lifecycle: state`，关联唯一的 `sonic_teleop` 状态。框架会：

1. 在目标状态 prepare 阶段先启动 `pico_manager`，再启动 `smpl_bridge`。
2. 把 bridge 加入宿主 `MultiThreadedExecutor`，由 50 Hz 非阻塞 timer 排空 ZMQ 输入；
   它不自行初始化 ROS、不接管信号，也没有第二套 spin 循环。
3. 在取消 prepare 或离开 SONIC 状态时先销毁 bridge，再关闭 manager。
4. 对 manager 的普通运行时故障执行有限重启；依赖缺失、解释器不可用等确定性配置
   故障以退出码 `78` 报告，并由框架直接标记 fault，禁止无意义重启。
5. manager 退出后，框架停止并标记依赖它的 bridge。
6. 向 manager 独立进程组发送 `SIGINT`，3 秒后升级为 `SIGTERM`，5 秒后发送
   `SIGKILL`，包括清理仍存活的派生进程。

`pico_manager` 内部仍负责释放 `xrobotoolkit_sdk` 并关闭它自己创建的
`RoboticsServiceProcess`。这是 SDK 资源所有权，不是另一套状态生命周期。

## 必需的 PICO/XR SDK

本 Mod 不需要 PICO OpenXR、Unity、Unreal 或可视化 SDK。真实 PICO 输入只依赖下面
这一套 XRoboToolkit PC Service 运行栈。

### 1. RoboticsService

目标电脑需要安装 Linux x86_64 版本的 XRoboToolkit PC Service。默认路径是：

```text
/opt/apps/roboticsservice/RoboticsServiceProcess
/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so
/opt/apps/roboticsservice/lib/
/opt/apps/roboticsservice/plugins/
/opt/apps/roboticsservice/qml/
```

可通过 `SONIC_XRT_SERVICE_DIR` 修改根目录。当前集成验证过的安装包基线是
`roboticsservice 1.0.0.0 amd64`。

### 2. xrobotoolkit_sdk Python binding

PICO Python 环境必须能导入 `xrobotoolkit_sdk`。当前验证基线是：

```text
xrobotoolkit_sdk 1.0.2
CPython 3.10
Linux x86_64
```

binding 会动态加载 `libPXREARobotSDK.so`。框架根据 `SONIC_XRT_SERVICE_DIR` 自动把
以下已存在目录 prepend 到 manager 的 `LD_LIBRARY_PATH`：

```text
<service-root>/SDK/x64
<service-root>
<service-root>/lib
```

这里的 `${NAME:-default}` 是框架执行的安全环境插值，不经过 shell。若部署需要复杂的
shell 判断或命令替换，应放进 Mod 内的启动脚本并用 `exec` 启动最终进程。

### 3. 头显侧数据源

PICO 头显侧必须运行与 XRoboToolkit PC Service 配套的数据发送程序，并启用 body
tracking。是否需要 PICO Motion Tracker 取决于头显侧采用的全身追踪方案；电脑端只
要求 `xrobotoolkit_sdk.is_body_data_available()` 成功，并能够读取
`get_body_joints_pose()`、控制器按钮、trigger、grip 和摇杆数据。

## PICO Python 环境

除 `xrobotoolkit_sdk` 外，独立 manager 解释器还需要：

```text
numpy>=1.26,<2
scipy>=1.10
pyzmq>=25
msgpack>=1.0
pin>=2.7       # Python import 名称为 pinocchio
```

这些依赖记录在 `requirements-pico.txt`。推荐使用独立 Python 3.10 环境：

```bash
python3.10 -m venv .venv_teleop
.venv_teleop/bin/python -m pip install -U pip
.venv_teleop/bin/python -m pip install -r \
  src/bxi_example_py_elf3/mods/com.bxi.sonic/requirements-pico.txt

# 再安装与平台和 Python ABI 匹配的 xrobotoolkit_sdk wheel/extension。
export SONIC_PICO_PYTHON="$PWD/.venv_teleop/bin/python"
export SONIC_XRT_SERVICE_DIR=/opt/apps/roboticsservice
```

bridge 始终使用宿主 ROS Python，不使用 `SONIC_PICO_PYTHON`，因此厂商环境不需要
安装 `rclpy`、`std_msgs` 或继承宿主 site-packages。这条边界避免 ROS 环境与 Conda/
厂商二进制扩展相互污染。

manager 的人体姿态变换和 SMPL 前向运动学使用 NumPy/SciPy，不运行 Torch 模型，
因此不需要安装 PyTorch 或 CUDA。

### NumPy 与原 Torch 路径性能对比

2026-07-31 使用修改前 Git 基线 `3afd15b223ce4089cc4ed801a660c33ef73d85ab`
中的原始 Torch 实现，与当前 NumPy/SciPy 实现进行了同机对比。测试环境为 Intel
Core i5-12600KF、Python 3.10.12、NumPy 1.26.4、SciPy 1.15.3、
Torch 2.13.0+cu130，`torch.cuda.is_available()` 为 `False`。

持续测试使用同一批 256 组固定随机种子的 PICO 24 关节姿态，预热 300 帧后分别执行
5 轮、每轮 2000 帧，共计 10000 帧。端到端数据包含原路径实际执行的
`detach().cpu().numpy()`；核心计算数据不包含结果取出。Torch 固定为单线程以避免小
矩阵任务受线程调度开销影响；使用原默认 10 线程时结果基本相同。

| 指标 | 原 Torch | NumPy/SciPy | 加速比 |
| --- | ---: | ---: | ---: |
| 独立进程导入均值（8 次） | 1225.8 ms | 174.1 ms | 7.04x |
| 清空静态数据缓存后的首次计算 | 37.83 ms | 1.36 ms | 27.8x |
| 核心计算 mean | 1289.2 us | 411.1 us | 3.14x |
| 端到端 mean | 1292.2 us | 408.3 us | 3.16x |
| 端到端 p50 | 1282.3 us | 404.7 us | 3.17x |
| 端到端 p95 | 1352.6 us | 428.3 us | 3.16x |
| 端到端 p99 | 1425.8 us | 440.8 us | 3.23x |

128 组随机姿态的数值回归中，关节坐标最大绝对误差约为 `4.5e-6`，四元数及 6D
朝向最大绝对误差约为 `4.2e-7`。当前 NumPy 路径约占 50 Hz 控制周期 20 ms 预算的
2%。这些结果用于确认本次迁移没有以性能或精度为代价；不同 CPU、SciPy 版本和系统
负载下的绝对延迟会变化。上述首次对比运行在没有暴露 GPU 设备的隔离环境中，因此只
代表 CPU 路径。

#### Torch CPU、Torch CUDA 与 NumPy 三方对比

随后在同一台宿主机的 RTX 3060 12 GB 上使用 Conda `pytorch` 环境重新进行三方测试。
该环境为 Python 3.10.12、Torch 2.11.0.dev20260210+cu128、CUDA 12.8、
NumPy 2.2.6 和 SciPy 1.15.3。三条路径使用相同输入帧和同一个 Python 进程；预热 500
帧后各测量 5000 帧，并完整重复两次。Torch CPU 固定为单线程。

Torch CUDA 使用两种计时边界：

- `CUDA 端到端` 与原 manager 行为一致，包含 SciPy CPU 预处理、NumPy 到 CUDA Tensor
  上传、CUDA 计算、结果下载及 `detach().cpu().numpy()`，每帧前后同步 CUDA。
- `CUDA 输入常驻` 预先把姿态输入放在 GPU，使用 CUDA Event 测量原 Torch 几何函数；
  不包含姿态预处理、输入上传和结果下载，但保留原实现内部的运算及静态关节数据处理。

以下为第一轮完整结果；第二轮各路径的持续均值见后文范围。

| 路径 | mean | p50 | p95 | p99 |
| --- | ---: | ---: | ---: | ---: |
| 原 Torch CPU 端到端 | 1327.7 us | 1327.6 us | 1386.6 us | 1458.5 us |
| 原 Torch CUDA 端到端 | 2742.7 us | 2703.9 us | 3073.1 us | 3326.7 us |
| 原 Torch CUDA 输入常驻 | 2245.1 us | 2221.4 us | 2496.6 us | 2697.2 us |
| 当前 NumPy/SciPy CPU 端到端 | 398.9 us | 394.9 us | 415.6 us | 458.1 us |

两轮持续测试的 mean 范围为：

| 路径 | mean 范围 | 相对当前 NumPy 路径 |
| --- | ---: | ---: |
| 原 Torch CPU 端到端 | 1263.5–1327.7 us | 慢 3.07–3.33x |
| 原 Torch CUDA 端到端 | 2655.6–2742.7 us | 慢 6.46–6.88x |
| 原 Torch CUDA 输入常驻 | 2245.1–2248.1 us | 慢 5.46–5.63x |
| 当前 NumPy/SciPy CPU 端到端 | 398.9–411.0 us | 基准 |

模块已经导入、静态数据缓存清空后的首次计算波动更大：原 Torch CPU 为
36.6–89.8 ms，原 Torch CUDA 为 291.1–413.3 ms，NumPy/SciPy CPU 为
1.43–2.02 ms。Torch CUDA 与 Torch CPU 的最大绝对误差为 `2.38e-7`，当前 NumPy
与 Torch CPU 的最大绝对误差为 `2.74e-6`。

原 Torch CUDA 在此任务中更慢，主要因为输入 batch 为 1、FK 只有 55 个关节，却会
发起大量细粒度 CUDA 运算；kernel 调度和同步成本高于实际计算量。即使输入预先常驻
GPU，原 eager Torch 路径仍约为 2.25 ms。这里比较的是 SONIC 原始实现，不能外推为
所有 CUDA 实现都更慢；批处理、算子融合、`torch.compile` 或专用 CUDA kernel 可能
得到不同结果，但都需要重新实现和独立验证。

如果选择安装到 Miniconda base，而不是独立 venv：

```bash
/home/kkkk/miniconda3/bin/python -m pip install -r \
  src/bxi_example_py_elf3/mods/com.bxi.sonic/requirements-pico.txt
/home/kkkk/miniconda3/bin/python -m pip install /path/to/xrobotoolkit_sdk.whl
```

SONIC 启动器会自动探测当前解释器、已激活的 venv/Conda 环境、常见的 Miniconda/
Anaconda base，必要时再查询 Conda 环境列表。只有能实际导入该进程全部依赖的解释器
才会启动 manager。因此其他机器不要求使用相同路径，也不要求一定安装在 base。
自动扫描只是开发机 fallback；正式部署推荐显式设置解释器，以便配置可审计、行为可
复现。

如果需要禁止自动选择并固定某个环境，再显式设置：

```bash
export SONIC_PICO_PYTHON=/path/to/python
```

显式解释器缺依赖时不会静默回退，启动器会一次列出所需模块、已检查解释器及失败
原因。

## 环境变量

SONIC 对外只保留部署环境相关变量：

| 变量 | 默认值 | 用途 |
| --- | --- | --- |
| `SONIC_PICO_PYTHON` | 自动探测 | 可选；强制 manager 使用指定解释器 |
| `SONIC_XRT_SERVICE_DIR` | `/opt/apps/roboticsservice` | RoboticsService 根目录 |

算法行为和夹爪硬件配置不使用环境变量，统一写在 `mod.yaml` 的 state `params`。
这样启动进程、状态可用性检查和 policy 使用的是同一份显式配置，启动 shell 中残留的
旧变量也不会悄悄改变动作行为。

## 状态参数

`sonic_teleop` 支持以下 policy 参数：

| 参数 | 默认值 | 用途 |
| --- | ---: | --- |
| `require_live_reference` | `false` | 是否必须有新鲜的 PICO reference 才允许进入 |
| `yaw_bias_rad` | `1.57079632679` | PICO 朝向对齐的 yaw 偏置 |
| `live_reference_timeout_s` | `0.5` | live reference 新鲜度阈值 |
| `idle_frame_start` | `3509` | idle reference 起始帧 |
| `source_blend_seconds` | `0.4` | idle/live 数据源切换的混合时间 |

同一个状态还支持以下夹爪参数：

| 参数 | 默认值 | 用途 |
| --- | ---: | --- |
| `hardware_gripper` | `false` | 是否允许该状态订阅 trigger 并发送夹爪 CAN 命令 |
| `gripper_input_timeout_s` | `0.2` | trigger 输入新鲜度阈值 |
| `gripper_release_threshold` | `0.05` | 进入状态时确认 trigger 已松开的阈值 |
| `gripper_left_bus` | `5` | 左夹爪 CAN 总线号 |
| `gripper_right_bus` | `6` | 右夹爪 CAN 总线号 |
| `gripper_can_id` | `1` | 两侧夹爪电机 CAN ID |
| `gripper_kp` | `20.0` | 夹爪位置环 KP |
| `gripper_kd` | `1.0` | 夹爪位置环 KD |

bridge 始终发布 `pico/left_trigger`、`pico/right_trigger`、`pico/left_grip` 和
`pico/right_grip`。是否真正向夹爪发送 CAN 命令只由 `mod.yaml` 中的
`hardware_gripper` 参数决定；修改后重新构建部署即可，不需要第二个控制状态。

## 内部通信

SONIC 的 ZMQ 只用于同一台机器上的 Mod 内部进程通信，默认拓扑为：

| 数据流 | 地址 | topic |
| --- | --- | --- |
| PICO manager → bridge | `127.0.0.1:5556` | `pose` |
| bridge → policy | `127.0.0.1:5557` | `smpl_ref` |

默认值集中定义在 `pico/runtime_config.py`。manager 和 policy 使用相同协议常量；
bridge 的 endpoint、topic、频率和新鲜度配置在 `mod.yaml` 的 node `params` 中显式
声明，由 `NodeBuildContext` 注入，不读取散落的环境变量，也不再提供 wrapper 命令行
兼容入口。

## 部署检查

先检查文件：

```bash
test -x /opt/apps/roboticsservice/RoboticsServiceProcess && \
  echo "RoboticsServiceProcess OK"

test -f /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so && \
  echo "libPXREARobotSDK.so OK"
```

直接测试 binding 时需要手动设置动态库路径；框架正常启动时会自动注入：

```bash
export SONIC_XRT_SERVICE_DIR=/opt/apps/roboticsservice
export LD_LIBRARY_PATH="$SONIC_XRT_SERVICE_DIR/SDK/x64:$SONIC_XRT_SERVICE_DIR:$SONIC_XRT_SERVICE_DIR/lib:${LD_LIBRARY_PATH:-}"

"${SONIC_PICO_PYTHON:-python3}" -c \
  'import xrobotoolkit_sdk as xrt; print("xrobotoolkit_sdk OK", xrt)'

"${SONIC_PICO_PYTHON:-python3}" -c \
  'import numpy, scipy, zmq, msgpack, pinocchio; print("PICO Python dependencies OK")'
```

运行 SONIC 后可检查服务和端口：

```bash
pgrep -af 'RoboticsServiceProcess|manager_launcher'
ss -lntup | grep -E ':(60061|5556|5557)\b'
```

## 实时 reference 与 idle fallback

默认 `require_live_reference: false`。进入状态时允许使用随 Mod 安装的 idle reference；
PICO 同时按住 `A+B+X+Y` 请求校准后平滑切到 live reference，数据过期后平滑退回
idle。按键请求与身体追踪数据解耦：若按下组合键时身体流尚未就绪，manager 会保留这次
请求，并在第一帧新鲜身体数据到达后自动完成校准，不需要反复按键。

启动日志会明确区分三个阶段：

- `PICO buttons: ...`：manager 已经开始读取控制器；按下或松开 ABXY 会打印当前组合。
- `ABXY accepted; calibration requested`：组合键已被接受。
- `Body tracking data available`：身体流已就绪；若持续显示 `Waiting for body tracking
  data` 或 `no fresh body frame`，应在头显端应用中启用 Body Tracking、确认目标 PC IP，
  然后点击 `Send`；仅能读取 ABXY 不代表身体流已经启动，也不应排查 Xbox/CRSF 配置。

校准完成后再同时按 `A+X`，切入实时 POSE。这里的按键来自
`xrobotoolkit_sdk`，不经过 `remote_controller/config/xbox_default.yaml`。

若设置 `require_live_reference: true`，`is_available()` 只允许在已经收到新鲜 live
reference 时进入。因为可用性检查发生在 state-scoped 节点 prepare 之前，首次进入时
需要把 `pico_manager` 和 `smpl_bridge` 都改成 `lifecycle: mod`，让 PICO 数据流在状态
切换请求之前已经运行。
