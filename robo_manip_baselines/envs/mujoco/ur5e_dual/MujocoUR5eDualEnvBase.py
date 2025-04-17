from os import path

import mujoco
import numpy as np
from gymnasium.spaces import Box, Dict

from robo_manip_baselines.common import ArmConfig
from robo_manip_baselines.teleop import GelloInputDevice, SpacemouseInputDevice,KeyboardInputDevice

from ..MujocoEnvBase import MujocoEnvBase


class MujocoUR5eDualEnvBase(MujocoEnvBase):
    default_camera_config = {
        "azimuth": -135.0,
        "elevation": -45.0,
        "distance": 1.8,
        "lookat": [-0.2, -0.2, 0.8],
    }
    observation_space = Dict(
        {
            "joint_pos": Box(low=-np.inf, high=np.inf, shape=(14,), dtype=np.float64),
            "joint_vel": Box(low=-np.inf, high=np.inf, shape=(14,), dtype=np.float64),
            "wrench": Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float64),
        }
    )

    def setup_robot(self, init_qpos):
        self.init_qpos[: len(init_qpos)] = init_qpos
        self.init_qvel[:] = 0.0

        mujoco.mj_kinematics(self.model, self.data)

        self.body_config_list = [
            ArmConfig(
                arm_urdf_path=path.join(
                    path.dirname(__file__), "../../assets/common/robots/ur5e/ur5e.urdf"
                ),
                arm_root_pose=self.get_body_pose("left/ur5e_root_frame"),
                ik_eef_joint_id=6,
                arm_joint_idxes=np.arange(0, 6),
                gripper_joint_idxes=np.array([6]),
                gripper_joint_idxes_in_gripper_joint_pos=np.array([0]),
                eef_idx=0,
                init_arm_joint_pos=self.init_qpos[0:6],
                init_gripper_joint_pos=np.zeros(1),
            ),
            ArmConfig(
                arm_urdf_path=path.join(
                    path.dirname(__file__), "../../assets/common/robots/ur5e/ur5e.urdf"
                ),
                arm_root_pose=self.get_body_pose("right/ur5e_root_frame"),
                ik_eef_joint_id=6,
                arm_joint_idxes=np.arange(7, 13),
                gripper_joint_idxes=np.array([13]),
                gripper_joint_idxes_in_gripper_joint_pos=np.array([1]),
                eef_idx=1,
                init_arm_joint_pos=self.init_qpos[14:20],
                init_gripper_joint_pos=np.zeros(1),
            ),
        ]

    def setup_input_device(self, input_device_name, motion_manager, overwrite_kwargs):
        if input_device_name == "spacemouse":
            InputDeviceClass = SpacemouseInputDevice
        elif input_device_name == "gello":
            InputDeviceClass = GelloInputDevice
        elif input_device_name == "keyboard":
            InputDeviceClass = KeyboardInputDevice
        else:
            raise ValueError(
                f"[{self.__class__.__name__}] Invalid input device key: {input_device_name}"
            )

        default_kwargs = self.get_input_device_kwargs(input_device_name)

        return [
            InputDeviceClass(
                body_manager,
                **{
                    **default_kwargs.get(device_idx, {}),
                    **overwrite_kwargs.get(device_idx, {}),
                },
            )
            for device_idx, body_manager in enumerate(motion_manager.body_manager_list)
        ]

    def get_input_device_kwargs(self, input_device_name):
        return {}

    def _get_obs(self):
        left_obs = self._get_obs_single_arm("left")
        right_obs = self._get_obs_single_arm("right")
        return {
            key: np.concatenate([left_obs[key], right_obs[key]])
            for key in left_obs.keys()
        }

    def _get_obs_single_arm(self, left_right):
        arm_joint_name_list = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        gripper_joint_name_list = [
            "right_driver_joint",
            "right_spring_link_joint",
            "left_driver_joint",
            "left_spring_link_joint",
        ]

        arm_joint_pos = np.array(
            [
                self.data.joint(left_right + "/" + joint_name).qpos[0]
                for joint_name in arm_joint_name_list
            ]
        )
        arm_joint_vel = np.array(
            [
                self.data.joint(left_right + "/" + joint_name).qvel[0]
                for joint_name in arm_joint_name_list
            ]
        )
        gripper_qpos = np.array(
            [
                self.data.joint(left_right + "/" + joint_name).qpos[0]
                for joint_name in gripper_joint_name_list
            ]
        )
        gripper_joint_pos = np.rad2deg(gripper_qpos.mean(keepdims=True)) / 45.0 * 255.0
        gripper_joint_vel = np.zeros(1)
        force = self.data.sensor(left_right + "/" + "force_sensor").data.flat.copy()
        torque = self.data.sensor(left_right + "/" + "torque_sensor").data.flat.copy()

        return {
            "joint_pos": np.concatenate(
                (arm_joint_pos, gripper_joint_pos), dtype=np.float64
            ),
            "joint_vel": np.concatenate(
                (arm_joint_vel, gripper_joint_vel), dtype=np.float64
            ),
            "wrench": np.concatenate((force, torque), dtype=np.float64),
        }
