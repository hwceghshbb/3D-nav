from bxi_example_py_elf3.policies import DanceMotionPolicyGravityIsaaclabV3
from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import BalletState


POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3]("com.bxi.ballet/policy")


def _load(context: ResourceLoadContext) -> DanceMotionPolicyGravityIsaaclabV3:
    return DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/ballet.npz")),
        str(context.asset("assets/ballet.onnx")),
        start_frame=60,
        fixed_pos=True,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load, policy="on_demand")
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "ballet": lambda state: BalletState(state.name, state.state_id, policy)
        }
    )
