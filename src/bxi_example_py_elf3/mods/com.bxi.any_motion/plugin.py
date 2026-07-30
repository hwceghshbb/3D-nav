from __future__ import annotations

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    StateBuildContext,
)

from .state import AnyMotionParams, AnyMotionState, resolve_mod_asset


MODEL_ASSET = "assets/rgmt.onnx"


def _build_state(mod_root, state: StateBuildContext) -> AnyMotionState:
    params = state.dataclass_params(AnyMotionParams)
    return AnyMotionState(
        state.name,
        state.state_id,
        model_path=resolve_mod_asset(mod_root, MODEL_ASSET, ".onnx"),
        motion_path=resolve_mod_asset(mod_root, params.npz, ".npz"),
        params=params,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    return ModDefinition(
        state_factories={
            "any_motion": lambda state: _build_state(context.mod_root, state),
        }
    )


__all__ = ["MODEL_ASSET", "create_mod"]
