from __future__ import annotations

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
    StateBuildContext,
)

from .rgmt import RgmtExternalReferencePolicy
from .state import AnyMotionParams, AnyMotionState, resolve_mod_asset


MODEL_ASSET = "assets/rgmt.onnx"


def _build_state(context: ModLoadContext, state: StateBuildContext) -> AnyMotionState:
    params = state.dataclass_params(AnyMotionParams)
    model_path = resolve_mod_asset(context.mod_root, MODEL_ASSET, ".onnx")
    motion_path = resolve_mod_asset(context.mod_root, params.npz, ".npz")
    policy_key = ResourceKey[RgmtExternalReferencePolicy](f"{state.name}/policy")

    def load_policy(_load_context: ResourceLoadContext) -> RgmtExternalReferencePolicy:
        policy = RgmtExternalReferencePolicy(
            str(motion_path),
            str(model_path),
            start_frame=params.start_frame,
            reference_yaw_mode=params.reference_yaw_mode,
            backend=params.backend,
        )
        if params.end_frame >= 0:
            policy.configure_range(end_frame=params.end_frame)
        return policy

    context.register_resource(policy_key, load_policy, policy="on_demand")
    return AnyMotionState(
        state.name,
        state.state_id,
        policy=context.resource(policy_key),
        model_path=model_path,
        motion_path=motion_path,
        params=params,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    return ModDefinition(
        state_factories={
            "any_motion": lambda state: _build_state(context, state),
        }
    )


__all__ = ["MODEL_ASSET", "create_mod"]
