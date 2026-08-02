import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
import communication.msg as bxiMsg
import communication.srv as bxiSrv
import nav_msgs.msg 
import sensor_msgs.msg
from threading import Lock
import numpy as np
# import torch
import time
import sys
import os
import math
from collections import deque
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

import onnxruntime as ort
import onnx
import ast
from scipy.spatial.transform import Rotation

from bxi_example_py_elf3.inference.amp_depth import (
    HumanoidGaitDepthPolicy,
    HumanoidGaitOriginCameraPolicy,
)

robot_name = "elf3"

dof_num = 29

joint_name = (
    "waist_y_joint",
    "waist_x_joint",
    "waist_z_joint",
    
    "l_hip_y_joint",   # 左腿_髋关节_z轴
    "l_hip_x_joint",   # 左腿_髋关节_x轴
    "l_hip_z_joint",   # 左腿_髋关节_y轴
    "l_knee_y_joint",   # 左腿_膝关节_y轴
    "l_ankle_y_joint",   # 左腿_踝关节_y轴
    "l_ankle_x_joint",   # 左腿_踝关节_x轴

    "r_hip_y_joint",   # 右腿_髋关节_z轴    
    "r_hip_x_joint",   # 右腿_髋关节_x轴
    "r_hip_z_joint",   # 右腿_髋关节_y轴
    "r_knee_y_joint",   # 右腿_膝关节_y轴
    "r_ankle_y_joint",   # 右腿_踝关节_y轴
    "r_ankle_x_joint",   # 右腿_踝关节_x轴

    "l_shoulder_y_joint",   # 左臂_肩关节_y轴
    "l_shoulder_x_joint",   # 左臂_肩关节_x轴
    "l_shoulder_z_joint",   # 左臂_肩关节_z轴
    "l_elbow_y_joint",   # 左臂_肘关节_y轴
    "l_wrist_x_joint",
    "l_wrist_y_joint",
    "l_wrist_z_joint",
    
    "r_shoulder_y_joint",   # 右臂_肩关节_y轴   
    "r_shoulder_x_joint",   # 右臂_肩关节_x轴
    "r_shoulder_z_joint",   # 右臂_肩关节_z轴
    "r_elbow_y_joint",    # 右臂_肘关节_y轴
    "r_wrist_x_joint",
    "r_wrist_y_joint",
    "r_wrist_z_joint",
    )   

def quaternion_to_euler_array(quat):
    # Ensure quaternion is in the correct format [x, y, z, w]
    x, y, z, w = quat
    
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)
    
    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    # Returns roll, pitch, yaw in a NumPy array in radians
    return np.array([roll_x, pitch_y, yaw_z])

def projected_gravity_from_quat(quaternion, gravity=np.array([0, 0, -9.81])):
    """
    计算重力在机体坐标系中的投影
    
    参数:
        quaternion: 四元数 [w, x, y, z] 或 [x, y, z, w]
        gravity: 世界坐标系中的重力向量 [x, y, z]，默认 [0, 0, -9.81]
    
    返回:
        重力在机体坐标系中的投影向量 [x, y, z]
    """
    # 创建旋转对象（自动处理四元数顺序）
    rot = Rotation.from_quat(quaternion)
    
    rot_inv = rot.inv()
    
    # 将重力向量从世界坐标系转换到机体坐标系
    # apply方法将向量从世界系旋转到机体系
    return rot_inv.apply(gravity)
    # return gravity

class BxiExample(Node):

    def __init__(self):

        super().__init__('bxi_example_py')
        
        self.declare_parameter('/topic_prefix', 'default_value')
        self.topic_prefix = self.get_parameter('/topic_prefix').get_parameter_value().string_value
        print('topic_prefix:', self.topic_prefix)

        
        self.declare_parameter('/onnx_file', 'default_value')
        self.onnx_file = self.get_parameter('/onnx_file').get_parameter_value().string_value        
        print("onnx_file:", self.onnx_file)

        package_share = get_package_share_directory("bxi_example_py_elf3")
        self.declare_parameter("/policy_type", "normal")
        self.policy_type = (
            self.get_parameter("/policy_type").get_parameter_value().string_value
        ).strip().lower()
        self.declare_parameter(
            "/depth_policy_file",
            os.path.join(package_share, "data", "depth_policy", "lyp2", "dagger1.onnx"),
        )
        self.depth_policy_file = (
            self.get_parameter("/depth_policy_file").get_parameter_value().string_value
        )
        self.declare_parameter(
            "/depth_image_topic",
            "/simulation/origin_depth/depth/image_raw",
        )
        self.depth_image_topic = (
            self.get_parameter("/depth_image_topic").get_parameter_value().string_value
        )
        print("policy_type:", self.policy_type)
        if self.policy_type != "normal":
            print("depth_policy_file:", self.depth_policy_file)
            print("depth_image_topic:", self.depth_image_topic)

        qos = QoSProfile(depth=1, durability=qos_profile_sensor_data.durability, reliability=qos_profile_sensor_data.reliability)
        
        self.act_pub = self.create_publisher(bxiMsg.ActuatorCmds, self.topic_prefix+'actuators_cmds', qos)  # CHANGE
        
        self.odom_sub = self.create_subscription(nav_msgs.msg.Odometry, self.topic_prefix+'odom', self.odom_callback, qos)
        # self.joint_sub = self.create_subscription(sensor_msgs.msg.JointState, self.topic_prefix+'joint_states', self.joint_callback, qos)
        self.actuator_sub = self.create_subscription(bxiMsg.ActuatorStates, self.topic_prefix+'actuator_states', self.actuator_callback, qos)
        self.imu_sub = self.create_subscription(sensor_msgs.msg.Imu, self.topic_prefix+'imu_data', self.imu_callback, qos)
        self.touch_sub = self.create_subscription(bxiMsg.TouchSensor, self.topic_prefix+'touch_sensor', self.touch_callback, qos)
        self.joy_sub = self.create_subscription(bxiMsg.MotionCommands, 'motion_commands', self.joy_callback, qos)
        self.depth_sub = None
        self.latest_depth_image = None
        self.latest_depth_frame_id = None
        self.depth_frame_counter = 0
        self.warned_bad_depth_encoding = False
        if self.policy_type != "normal":
            self.depth_sub = self.create_subscription(
                sensor_msgs.msg.Image,
                self.depth_image_topic,
                self.depth_image_callback,
                qos,
            )

        self.rest_srv = self.create_client(bxiSrv.RobotReset, self.topic_prefix+'robot_reset')
        self.sim_rest_srv = self.create_client(bxiSrv.SimulationReset, self.topic_prefix+'sim_reset')
        
        self.timer_callback_group_1 = MutuallyExclusiveCallbackGroup()

        self.num_action = dof_num
        self.num_obs = 96
        self.depth_policy = None
        if self.policy_type == "normal":
            model = onnx.load(self.onnx_file)
            metadata = {}
            for prop in model.metadata_props:
                metadata[prop.key] = prop.value
            # print(model.metadata_props)

            print(metadata)
            self.joint_names = metadata["joint_names"]
            self.joint_stiffness = np.array(ast.literal_eval(metadata["joint_stiffness"]), dtype=np.float32)
            self.joint_damping = np.array(ast.literal_eval(metadata["joint_damping"]), dtype=np.float32)
            self.action_scale = np.array(ast.literal_eval(metadata["action_scale"]), dtype=np.float32)
            self.default_joint_pos = np.array(ast.literal_eval(metadata["default_joint_pos"]), dtype=np.float32)
            # self.default_joint_pos[[7,13]] += 0.05
            # exit()
        elif self.policy_type in ("depth_origin", "origin_camera"):
            self.depth_policy = HumanoidGaitOriginCameraPolicy(self.depth_policy_file)
            self.num_obs = self.depth_policy.num_obs
            self.joint_names = ",".join(joint_name)
            self.joint_stiffness = self.depth_policy.kps.copy()
            self.joint_damping = self.depth_policy.kds.copy()
            self.action_scale = np.ones(self.num_action, dtype=np.float32)
            self.default_joint_pos = self.depth_policy.default_dof_pos.copy()
        elif self.policy_type in ("depth", "depth_walk"):
            self.depth_policy = HumanoidGaitDepthPolicy(self.depth_policy_file)
            self.num_obs = self.depth_policy.num_obs
            self.joint_names = ",".join(joint_name)
            self.joint_stiffness = self.depth_policy.kps.copy()
            self.joint_damping = self.depth_policy.kds.copy()
            self.action_scale = np.ones(self.num_action, dtype=np.float32)
            self.default_joint_pos = self.depth_policy.default_dof_pos.copy()
        else:
            raise RuntimeError(f"unsupported policy_type: {self.policy_type}")

        self.lock_in = Lock()
        self.lock_ou = self.lock_in #Lock()
        self.qpos = np.zeros(self.num_action,dtype=np.double)
        self.qvel = np.zeros(self.num_action,dtype=np.double)
        self.omega = np.zeros(3,dtype=np.double)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.double)
        
        self.target_q = np.zeros(self.num_action, dtype=np.double)
        self.action = np.zeros(self.num_action, dtype=np.double)

        if self.policy_type == "normal":
            policy_input = np.zeros([1, self.num_obs], dtype=np.float32)
            print("policy test")

            self.initialize_onnx(self.onnx_file)
            self.action[:] = self.inference_step(policy_input)
            self.action.fill(0.0)

        self.vx = 0.0
        self.vy = 0
        self.dyaw = 0

        self.step = 0
        self.loop_count = 0
        self.dt = 0.02  # loop @50Hz
        self.inference_period = self.dt
        self.inference_timeout_tolerance = 0.001
        if self.depth_policy is not None:
            self.inference_timeout_tolerance = 0.005
        self.last_inference_frame_time = None
        self.inference_timeout_count = 0
        self.timer = self.create_timer(self.dt, self.timer_callback, callback_group=self.timer_callback_group_1)

    # 初始化部分（完整版）
    def initialize_onnx(self, model_path):
        # 配置执行提供者（根据硬件选择最优后端）
        providers = [
            'CUDAExecutionProvider',  # 优先使用GPU
            'CPUExecutionProvider'    # 回退到CPU
        ] if ort.get_device() == 'GPU' else ['CPUExecutionProvider']
        
        # 启用线程优化配置
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4  # 设置计算线程数
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        
        # 创建推理会话
        self.session = ort.InferenceSession(
            model_path,
            providers=providers,
            sess_options=options
        )
        
        # 预存输入输出信息
        self.input_info = self.session.get_inputs()[0]
        self.output_info = self.session.get_outputs()[0]
        
        # 预分配输入内存（可选，适合固定输入尺寸）
        self.input_buffer = np.zeros(
            self.input_info.shape,
            dtype=np.float32
        )

    # 循环推理部分（极速版）
    def inference_step(self, obs_data):
        # 使用预分配内存（如果适用）
        np.copyto(self.input_buffer, obs_data)  # 比直接赋值更安全
        
        # 极简推理（比原版快5-15%）
        return self.session.run(
            [self.output_info.name], 
            {self.input_info.name: self.input_buffer}
        )[0][0]  # 直接获取第一个输出的第一个样本
 
    def timer_callback(self):
        
        # ptyhon 与 rclpy 多线程不太友好，这里使用定时间+简易状态机运行a
        if self.step == 0:
            self.robot_reset(1, False) # first reset
            print('robot reset 1!')
            self.step = 1
            return
        elif self.step == 1 and self.loop_count >= (10./self.dt): # 延迟10s
            self.robot_reset(2, True) # first reset
            print('robot reset 2!')
            self.loop_count = 0
            self.step = 2
            self.reset_inference_timeout_monitor()
            return
        
        if self.step == 1:
            soft_start = self.loop_count/(1./self.dt) # 1秒关节缓启动
            if soft_start > 1:
                soft_start = 1
                
            soft_joint_kp = self.joint_stiffness * soft_start #* 0.2
            soft_joint_kd = self.joint_damping #* 0.2
                
            msg = bxiMsg.ActuatorCmds()
            msg.header.frame_id = robot_name
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.actuators_name = joint_name
            msg.pos = self.default_joint_pos.tolist()
            msg.vel = np.zeros(dof_num, dtype=np.float32).tolist()
            msg.torque = np.zeros(dof_num, dtype=np.float32).tolist()
            msg.kp = soft_joint_kp.tolist()
            msg.kd = soft_joint_kd.tolist()
            self.act_pub.publish(msg)
            
        elif self.step == 2:
            with self.lock_in:
                q = self.qpos.copy()
                dq = self.qvel.copy()
                quat = self.quat.copy()
                omega = self.omega.copy()
                depth_image = None if self.latest_depth_image is None else self.latest_depth_image.copy()
                depth_frame_id = self.latest_depth_frame_id
                
                x_vel_cmd = self.vx
                y_vel_cmd = self.vy
                yaw_vel_cmd = self.dyaw
            
            # count_lowlevel = self.loop_count
                    
            obs = np.zeros([1, self.num_obs], dtype=np.float32)
            
            quat_norm = np.linalg.norm(quat)
            if not np.isfinite(quat_norm) or quat_norm < 1.0e-6:
                print("check safe error, stopping actuator commands!")
                self.publish_safe_stop()
                self.step = 3
                return
            quat = quat / quat_norm
            eu_ang = quaternion_to_euler_array(quat)
            eu_ang[eu_ang > math.pi] -= 2 * math.pi
            projected_gravity = projected_gravity_from_quat(quat, np.array([0, 0, -1]))

            if (np.abs(eu_ang[0]) > (math.pi/3.0)) or (
                np.abs(eu_ang[1]) > (math.pi/3.0)):
                print("check safe error, stopping actuator commands!")
                self.publish_safe_stop()
                self.step = 3
                return

            if self.depth_policy is None:
                obs[0, :3] = omega
                obs[0, 3:6] = projected_gravity
                obs[0, 6:6+self.num_action] = (q-self.default_joint_pos)
                obs[0, 6+(self.num_action*1):6+(self.num_action*2)] = dq
                obs[0, 6+(self.num_action*2):6+(self.num_action*3)] = self.action

                obs[0, -3] = x_vel_cmd
                obs[0, -2] = y_vel_cmd
                obs[0, -1] = yaw_vel_cmd

                # obs = np.clip(obs, -env_cfg.normalization.clip_observations, env_cfg.normalization.clip_observations)

                # self.hist_obs.append(obs)
                # self.hist_obs.popleft()

                policy_input = np.zeros([1, self.num_obs], dtype=np.float32)

                policy_input = obs

                self.action[:] = self.inference_step(policy_input)
                self.check_inference_frame_timeout()
                # self.action = np.clip(self.action, -env_cfg.normalization.clip_actions, env_cfg.normalization.clip_actions)
                self.target_q = self.action * self.action_scale
                qpos = self.default_joint_pos.copy()
                qpos[:] += self.target_q[:]

                kp = self.joint_stiffness #* 0.9
                kd = self.joint_damping #* 0.2
            else:
                quat_wxyz = np.array([quat[3], quat[0], quat[1], quat[2]], dtype=np.float32)
                cmd_vel = np.array([x_vel_cmd, y_vel_cmd, yaw_vel_cmd], dtype=np.float32)
                qpos = self.depth_policy.inference_step(
                    q,
                    dq,
                    quat_wxyz,
                    omega,
                    cmd_vel,
                    depth_image,
                    depth_frame_id=depth_frame_id,
                )
                self.action[:] = self.depth_policy.action
                self.check_inference_frame_timeout()
                kp = self.depth_policy.kps
                kd = self.depth_policy.kds
            
            msg = bxiMsg.ActuatorCmds()
            msg.header.frame_id = robot_name
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.actuators_name = joint_name
            msg.pos = qpos.tolist()
            msg.vel = np.zeros(dof_num, dtype=np.float32).tolist()
            msg.torque = np.zeros(dof_num, dtype=np.float32).tolist()
            msg.kp = kp.tolist()
            msg.kd = kd.tolist()
            self.act_pub.publish(msg)
            self.last_action=self.action.copy()

        self.loop_count += 1

    def publish_safe_stop(self):
        msg = bxiMsg.ActuatorCmds()
        msg.header.frame_id = robot_name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.actuators_name = joint_name
        msg.pos = self.default_joint_pos.tolist()
        msg.vel = np.zeros(dof_num, dtype=np.float32).tolist()
        msg.torque = np.zeros(dof_num, dtype=np.float32).tolist()
        msg.kp = np.zeros(dof_num, dtype=np.float32).tolist()
        msg.kd = np.zeros(dof_num, dtype=np.float32).tolist()
        self.act_pub.publish(msg)

    def robot_reset(self, reset_step, release):
        req = bxiSrv.RobotReset.Request()
        req.reset_step = reset_step
        req.release = release
        req.header.frame_id = robot_name
    
        while not self.rest_srv.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
            
        self.rest_srv.call_async(req)
        
    def sim_robot_reset(self):        
        req = bxiSrv.SimulationReset.Request()
        req.header.frame_id = robot_name

        base_pose = Pose()
        base_pose.position.x = 0.0
        base_pose.position.y = 0.0
        base_pose.position.z = 1.0
        base_pose.orientation.x = 0.0
        base_pose.orientation.y = 0.0
        base_pose.orientation.z = 0.0
        base_pose.orientation.w = 1.0        

        joint_state = JointState()
        joint_state.name = joint_name
        joint_state.position = np.zeros(dof_num, dtype=np.float32).tolist()
        joint_state.velocity = np.zeros(dof_num, dtype=np.float32).tolist()
        joint_state.effort = np.zeros(dof_num, dtype=np.float32).tolist()
        
        req.base_pose = base_pose
        req.joint_state = joint_state
    
        while not self.sim_rest_srv.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
            
        self.sim_rest_srv.call_async(req)
    
    def joint_callback(self, msg):
        joint_pos = msg.position
        joint_vel = msg.velocity
        joint_tor = msg.effort
        
        with self.lock_in:
            self.qpos[:] = np.array(joint_pos[:])
            self.qvel[:] = np.array(joint_vel[:])

    def actuator_callback(self, msg):
        joint_pos = msg.position
        joint_vel = msg.velocity
        joint_tor = msg.effort
        drv_temperature = msg.driver_temperature
        motor_temperature = msg.motor_temperature
        
        with self.lock_in:
            self.qpos[:] = np.array(joint_pos[:])
            self.qvel[:] = np.array(joint_vel[:])

    def joy_callback(self, msg):
        with self.lock_in:
            self.vx = msg.vel_des.x * 3
            self.vx = np.clip(self.vx, -2.0, 3.0)
            self.vy = msg.vel_des.y * 2
            self.dyaw = msg.yawdot_des * 2

    def depth_image_callback(self, msg):
        depth = self.image_to_depth_array(msg)
        if depth is None:
            return
        with self.lock_in:
            self.depth_frame_counter += 1
            self.latest_depth_frame_id = self.depth_frame_counter
            self.latest_depth_image = depth
        
    def imu_callback(self, msg):
        quat = msg.orientation
        avel = msg.angular_velocity
        acc = msg.linear_acceleration

        quat_tmp1 = np.array([quat.x, quat.y, quat.z, quat.w]).astype(np.double)

        with self.lock_in:
            self.quat = quat_tmp1
            self.omega = np.array([avel.x, avel.y, avel.z])

    def touch_callback(self, msg):
        foot_force = msg.value
        
    def odom_callback(self, msg): # 全局里程计（上帝视角，仅限仿真使用）
        base_pose = msg.pose
        base_twist = msg.twist

    # ---------------------------------------------------------------------------- #
    #                                     工具类函数                                    #
    # ---------------------------------------------------------------------------- #
    def image_to_depth_array(self, msg):
        if msg.encoding in ("32FC1", "32FC"):
            dtype = np.float32
            scale = 1.0
        elif msg.encoding in ("16UC1", "mono16"):
            dtype = np.uint16
            scale = 0.001
        else:
            if not self.warned_bad_depth_encoding:
                print(f"unsupported depth image encoding: {msg.encoding}")
                self.warned_bad_depth_encoding = True
            return None

        channels = 1
        row_values = msg.step // np.dtype(dtype).itemsize
        array = np.frombuffer(msg.data, dtype=dtype)
        if array.size < row_values * msg.height:
            return None
        array = array.reshape(msg.height, row_values)[:, : msg.width * channels]
        return (array.astype(np.float32) * scale).reshape(msg.height, msg.width)

    def reset_inference_timeout_monitor(self):
        self.last_inference_frame_time = None
        self.inference_timeout_count = 0

    def check_inference_frame_timeout(self):
        now = time.perf_counter()
        last = self.last_inference_frame_time
        self.last_inference_frame_time = now
        if last is None or self.inference_period <= 0.0:
            return

        frame_delay = now - last
        timeout_threshold = self.inference_period + self.inference_timeout_tolerance
        if frame_delay <= timeout_threshold:
            return

        self.inference_timeout_count += 1
        print(
            "[INFERENCE TIMEOUT] "
            f"delay={frame_delay * 1000.0:.2f}ms, "
            f"limit={self.inference_period * 1000.0:.2f}ms "
            f"({1.0 / self.inference_period:.1f}Hz), "
            f"tolerance={self.inference_timeout_tolerance * 1000.0:.2f}ms, "
            f"over={(frame_delay - self.inference_period) * 1000.0:.2f}ms, "
            f"count={self.inference_timeout_count}"
        )

def main(args=None):
   
    time.sleep(5)
    
    rclpy.init(args=args)
    node = BxiExample()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
