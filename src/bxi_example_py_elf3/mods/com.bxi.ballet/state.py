from bxi_example_py_elf3.inference.beyondmimic import DanceMotionPolicyGravityIsaaclabV3
from bxi_example_py_elf3.utils.mod_system import ResourceHandle
from bxi_example_py_elf3.utils.state_library import MotionReplayState


class BalletState(MotionReplayState[DanceMotionPolicyGravityIsaaclabV3]):
    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[DanceMotionPolicyGravityIsaaclabV3],
    ) -> None:
        super().__init__(
            name,
            state_id,
            policy,
            finish_trigger="ballet_finished",
            end_frame_trim=330,
            end_transition={
                "profile": "dual_running_blend",
                "duration": 1.0,
                "curve": "smootherstep",
                "sample_from": True,
            },
        )
