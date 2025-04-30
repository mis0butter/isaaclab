# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg

# ---------------------------------- 
# math stuff 
# ---------------------------------- 

import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# ====================================================================== 
# ====================================================================== 
    
def get_body_index(asset: 'Articulation', body_name: str) -> int:
    """Get the index of a body in an articulation by name.
    
    Args:
        asset: The articulation asset.
        body_name: The name of the body to find.
        
    Returns:
        The index of the body in the articulation's body_names list.
        
    Raises:
        ValueError: If the body name is not found in the articulation.
    """
    try:
        return asset.body_names.index(body_name)
    except ValueError:
        raise ValueError(f"Body name '{body_name}' not found in articulation. "
                         f"Available bodies: {asset.body_names}")

# ====================================================================== 
# ====================================================================== 

def joint_pos_target_l2(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize joint position deviation from a target value."""

    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    # wrap the joint positions to (-pi, pi)
    joint_pos = math_utils.wrap_to_pi(asset.data.joint_pos[:, asset_cfg.joint_ids])

    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)

# ====================================================================== 
# ====================================================================== 

def orientation_command_exp_error(
    env: ManagerBasedRLEnv, 
    command_name: str, 
    asset_cfg: SceneEntityCfg, 
    std: float) -> torch.Tensor: 
    """Penalize tracking orientation error using shortest path.
    The function computes the orientation error between the desired orientation (from the command) and the
    current orientation of the asset's body (in world frame). The orientation error is computed as the shortest
    path between the desired and current orientations.
    """

    # extract the asset (to enable type hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name) 

    # obtain the desired and current orientations
    des_quat_b = command[:, 3:7]
    des_quat_w = math_utils.quat_mul(asset.data.root_link_state_w[:, 3:7], des_quat_b) 

    # get the current orientation quaternion 
    base_link_idx = get_body_index(asset, "base_link")
    curr_quat_w = asset.data.body_link_state_w[:, base_link_idx, 3:7]
    
    return torch.exp(-math_utils.quat_error_magnitude(curr_quat_w, des_quat_w).pow(2)/std) 

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
    euler = math_utils.euler_xyz_from_quat(current_quat)
    # extract yaw angle (rotation around Z-axis)
    current_yaw = euler[:, 2]
    # wrap angles to [-pi, pi]
    current_yaw = math_utils.wrap_to_pi(current_yaw)
    # compute squared error
    return torch.square(current_yaw - target)
