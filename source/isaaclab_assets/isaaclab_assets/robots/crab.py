# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the SSTI Crab robot."""


import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

# ====================================================================== 
# ====================================================================== 

# ---------------------------------- 
# Configuration
# ---------------------------------- 

CRAB_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/june/research/IsaacLab/source/isaaclab_assets/data/robots/SSTI/crab/crab.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=1,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),  # Raised slightly off ground
        joint_pos={
            # Front right leg
            "front_right__cluster_1_roll": 0.0,
            "front_right__cluster_1_pitch": -0.5,
            "front_right__cluster_2_roll": 0.0,
            "front_right__cluster_2_pitch": -0.5,
            "front_right__cluster_3_roll": 0.0,
            "front_right__cluster_3_pitch": -0.5,
            "front_right__cluster_3_wrist": 0.0,
            # Front left leg
            "front_left__cluster_1_roll": 0.0,
            "front_left__cluster_1_pitch": -0.5,
            "front_left__cluster_2_roll": 0.0,
            "front_left__cluster_2_pitch": -0.5,
            "front_left__cluster_3_roll": 0.0,
            "front_left__cluster_3_pitch": -0.5,
            "front_left__cluster_3_wrist": 0.0,
            # Back right leg
            "back_right__cluster_1_roll": 0.0,
            "back_right__cluster_1_pitch": -0.5,
            "back_right__cluster_2_roll": 0.0,
            "back_right__cluster_2_pitch": -0.5,
            "back_right__cluster_3_roll": 0.0,
            "back_right__cluster_3_pitch": -0.5,
            "back_right__cluster_3_wrist": 0.0,
            # Back left leg
            "back_left__cluster_1_roll": 0.0,
            "back_left__cluster_1_pitch": -0.5,
            "back_left__cluster_2_roll": 0.0,
            "back_left__cluster_2_pitch": -0.5,
            "back_left__cluster_3_roll": 0.0,
            "back_left__cluster_3_pitch": -0.5,
            "back_left__cluster_3_wrist": 0.0,
        }
    ),
    actuators={
        # Front right leg actuators
        "fr_cluster1": ImplicitActuatorCfg(
            joint_names_expr=["front_right__cluster_1_roll", "front_right__cluster_1_pitch"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "fr_cluster2": ImplicitActuatorCfg(
            joint_names_expr=["front_right__cluster_2_roll", "front_right__cluster_2_pitch"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "fr_cluster3": ImplicitActuatorCfg(
            joint_names_expr=["front_right__cluster_3_roll", "front_right__cluster_3_pitch", "front_right__cluster_3_wrist"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        # Front left leg actuators
        "fl_cluster1": ImplicitActuatorCfg(
            joint_names_expr=["front_left__cluster_1_roll", "front_left__cluster_1_pitch"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "fl_cluster2": ImplicitActuatorCfg(
            joint_names_expr=["front_left__cluster_2_roll", "front_left__cluster_2_pitch"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "fl_cluster3": ImplicitActuatorCfg(
            joint_names_expr=["front_left__cluster_3_roll", "front_left__cluster_3_pitch", "front_left__cluster_3_wrist"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        # Back right leg actuators
        "br_cluster1": ImplicitActuatorCfg(
            joint_names_expr=["back_right__cluster_1_roll", "back_right__cluster_1_pitch"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "br_cluster2": ImplicitActuatorCfg(
            joint_names_expr=["back_right__cluster_2_roll", "back_right__cluster_2_pitch"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "br_cluster3": ImplicitActuatorCfg(
            joint_names_expr=["back_right__cluster_3_roll", "back_right__cluster_3_pitch", "back_right__cluster_3_wrist"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        # Back left leg actuators
        "bl_cluster1": ImplicitActuatorCfg(
            joint_names_expr=["back_left__cluster_1_roll", "back_left__cluster_1_pitch"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "bl_cluster2": ImplicitActuatorCfg(
            joint_names_expr=["back_left__cluster_2_roll", "back_left__cluster_2_pitch"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "bl_cluster3": ImplicitActuatorCfg(
            joint_names_expr=["back_left__cluster_3_roll", "back_left__cluster_3_pitch", "back_left__cluster_3_wrist"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
    },
)
"""Configuration for the SSTI Crab robot."""
