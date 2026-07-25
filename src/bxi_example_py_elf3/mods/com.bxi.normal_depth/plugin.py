from bxi_example_py_elf3.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
    StateBuildContext,
)

from .amp_depth import (
    HumanoidGaitDepthPolicyIsaaclab,
    HumanoidGaitOriginCameraPolicyIsaaclab,
)
from .state import NormalDepthState


LEGACY_POLICY = ResourceKey[HumanoidGaitDepthPolicyIsaaclab](
    "com.bxi.normal_depth/legacy_policy"
)
ORIGIN_POLICY = ResourceKey[HumanoidGaitOriginCameraPolicyIsaaclab](
    "com.bxi.normal_depth/origin_policy"
)


def _load_legacy_policy(
    context: ResourceLoadContext,
) -> HumanoidGaitDepthPolicyIsaaclab:
    return HumanoidGaitDepthPolicyIsaaclab(
        str(context.asset("assets/normal_depth.onnx"))
    )


def _load_origin_policy(
    context: ResourceLoadContext,
) -> HumanoidGaitOriginCameraPolicyIsaaclab:
    return HumanoidGaitOriginCameraPolicyIsaaclab(
        str(context.asset("assets/dagger2.onnx"))
    )


def _build_normal_depth_state(
    state: StateBuildContext,
    legacy_policy,
    origin_policy,
) -> NormalDepthState:
    mode = state.string_param("mode", "origin_camera")
    if mode == "origin_camera":
        policy = origin_policy
        default_topic = "/camera/depth/image_36x48"
    elif mode == "depth_walk":
        policy = legacy_policy
        default_topic = "/camera/depth/image_64x36"
    else:
        raise ValueError(
            f"state '{state.name}' param 'mode' must be "
            "'origin_camera' or 'depth_walk'"
        )

    return NormalDepthState(
        state.name,
        state.state_id,
        policy,
        mode=mode,
        depth_image_topic=state.string_param("topic", default_topic),
        depth_uint16_scale=state.float_param("depth_uint16_scale", 0.001),
        depth_timeout_sec=state.float_param("depth_timeout_sec", 1.0),
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(LEGACY_POLICY, _load_legacy_policy)
    context.register_resource(ORIGIN_POLICY, _load_origin_policy)
    legacy_policy = context.resource(LEGACY_POLICY)
    origin_policy = context.resource(ORIGIN_POLICY)

    return ModDefinition(
        state_factories={
            "normal_depth": lambda state: _build_normal_depth_state(
                state,
                legacy_policy,
                origin_policy,
            )
        }
    )
