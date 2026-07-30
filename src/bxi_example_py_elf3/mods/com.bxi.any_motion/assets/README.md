# any_motion assets

固定策略模型放在：

```text
assets/rgmt.onnx
```

动作参考文件默认放在：

```text
assets/motion.npz
```

如需使用其他 NPZ，只修改 `mod.yaml` 中 `states.any_motion.params.npz`。
路径必须位于本 Mod 的 `assets/` 目录内。ONNX 模型由 Mod 固定，不是状态参数。
