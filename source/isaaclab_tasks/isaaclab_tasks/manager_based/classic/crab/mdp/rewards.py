# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING
import math

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi
from isaaclab.utils.math import euler_xyz_from_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# ====================================================================== 
# ====================================================================== 

def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""

    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    # wrap the joint positions to (-pi, pi)
    joint_pos = wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])
    
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)

# ====================================================================== 
# ====================================================================== 

def root_orientation_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize root orientation deviation from a target yaw angle.
    
    Args:
        env: The environment instance.
        target: The target yaw angle in radians.
        asset_cfg: The asset configuration.
        
    Returns:
        The squared L2 error between current and target orientation.
    """
    # extract the used quantities
    asset: Articulation = env.scene[asset_cfg.name]
    # get the current orientation quaternion
    current_quat = asset.data.root_quat_w
    # convert quaternion to euler angles (roll, pitch, yaw)
    euler = euler_xyz_from_quat(current_quat)
    # extract yaw angle (rotation around Z-axis)
    current_yaw = euler[:, 2]
    # wrap angles to [-pi, pi]
    current_yaw = wrap_to_pi(current_yaw)
    # compute squared error
    return torch.square(current_yaw - target)
