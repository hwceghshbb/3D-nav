# 自动深度相机后端

`com.bxi.normal_depth` 支持 Intel RealSense 和 Orbbec Gemini 335，并向深度策略
提供相同的 ROS 2 接口。

## 自动选择

```yaml
camera_backend: auto       # auto / realsense / orbbec
camera_preference: orbbec  # 两种设备同时连接时优先选择
```

启动时节点读取 Linux USB sysfs：

- Intel USB vendor `8086`：选择 RealSense。
- Orbbec `2bc5:0800`：选择 Gemini 335。

`camera_backend` 设为具体后端时会跳过 USB 探测，适合容器未挂载 sysfs时强制
启动。自动探测只发生在节点启动或异常重启时，不进行运行中的热切换。

## Gemini 335 SDK

Gemini 335 后端是 Python `rclpy` 节点，直接使用官方 `pyorbbecsdk2` 管理设备、
Pipeline、标定数据和 SDK 滤镜。不启动 `orbbec_camera_node`，也没有中间 ROS 2
话题或额外进程。其结构与 RealSense 的 `pyrealsense2` 后端一致。

Python 3.10 的 Linux x86_64 和 aarch64 SDK 已分别放在：

```text
vendor/python/linux-x86_64-cpython-310/pyorbbecsdk/
vendor/python/linux-aarch64-cpython-310/pyorbbecsdk/
```

节点从 SDK 深度帧读取实际 `depth_scale` 和内参，直接发布：

```text
/camera/depth/image_raw
/camera/depth/camera_info
/camera/depth/image_64x36
/camera/depth/camera_info_64x36
/camera/depth/image_36x48
/camera/depth/camera_info_36x48
```

深度输出统一为 `16UC1` 毫米，策略继续使用 `depth_uint16_scale: 0.001`。
CameraInfo 使用 SDK 返回的深度内参和畸变参数。RealSense 与 Gemini 335 共用
`depth_w/depth_h/depth_fps`，默认均以 `480x270@60` 打开；Gemini 335 使用
`Y16` 格式。

`enable_color`、`enable_ir` 或 `enable_imu` 打开时，节点直接启用对应 SDK 数据流
并发布到现有 `/camera/...` 话题。

## 常用 Gemini 335 参数

```yaml
orbbec_serial: ""
orbbec_usb_port: ""
orbbec_enable_sdk_filters: true
```

DisparityTransform 是 SDK 对部分深度格式必需的基础转换，会始终保留；开启
`orbbec_enable_sdk_filters` 时还会启用设备推荐的空间、时间、降噪和孔洞填充
滤镜。Linux 主机仍需安装 Mod 中附带的 Orbbec udev 规则。

## RKNN INT8 校准数据

深度模型需要用真实运行分布做 INT8 校准。采集能力位于外部通用工具中，深度策略
自身不包含录制线程、环境变量或控制热路径分支。使用工具启动控制器：

```bash
python3 tools/benchmark/collect_calibration.py \
  --output /tmp/bxi_rknn_calibration \
  --every 5 \
  --max-samples 500 \
  --skip-first 10 \
  -- ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py
```

工具包装所有通过框架 `InferenceRuntime` 打开的后端，因此不局限于深度策略。
它在策略预处理和 observation history 组装之后记录传给 `backend.run()` 的最终输入。
默认 `origin_camera` 模式采集 `dagger2`；要采集 `normal_depth`，将状态的 `mode`
改为 `depth_walk` 后另跑一次。

完整校验和转换命令见 `tools/benchmark/README.md` 的
“Capture and build a representative INT8 model”。
