# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR 

# ====================================================================== 
# Configuration 
# ====================================================================== 

# from isaaclab_assets.robots.crab import CRAB_CFG
# from isaaclab_assets import CRAB_CFG 

# ---------------------------------- 
# Configuration
# ---------------------------------- 

CRAB_CFG = ArticulationCfg(
    # spawn=sim_utils.UsdFileCfg(
    #     usd_path="/home/june/research/IsaacLab/source/isaaclab_assets/data/robots/SSTI/crab/crab/crab.usd",
    #     # usd_path="/home/june/research/IsaacLab/source/isaaclab_assets/data/robots/SSTI/crab_.usd",
    #     rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #         rigid_body_enabled=True,
    #         max_linear_velocity=1000.0,
    #         max_angular_velocity=1000.0,
    #         max_depenetration_velocity=100.0,
    #         enable_gyroscopic_forces=True,
    #     ),
    #     articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    #         enabled_self_collisions=False,
    #         solver_position_iteration_count=4,
    #         solver_velocity_iteration_count=1,
    #         sleep_threshold=0.005,
    #         stabilization_threshold=0.001,
    #     ),
    # ),
    spawn=sim_utils.UrdfFileCfg(
        asset_path="/home/june/research/IsaacLab/source/isaaclab_assets/data/robots/SSTI/crab/crab.urdf",  # Replace with your URDF file path
        fix_base=False,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            target_type="position",
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=400.0,
                damping=1.0
            )
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            # rigid_body_enabled=True,
            disable_gravity=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
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
        pos=(0.0, 0.0, 10.0),  # Raised slightly off ground
        joint_pos={

            # Front left leg
            "arm1_j1": 0.0,
            "arm1_j2": -0.5,
            "arm1_j3": 0.0,
            "arm1_j4": -0.5,
            "arm1_j5": 0.0,
            "arm1_j6": -0.5,
            "arm1_j7": 0.0,

            # Back left leg
            "arm2_j1": 0.0,
            "arm2_j2": -0.5,
            "arm2_j3": 0.0,
            "arm2_j4": -0.5,
            "arm2_j5": 0.0,
            "arm2_j6": -0.5,
            "arm2_j7": 0.0,

            # Back right leg
            "arm3_j1": 0.0,
            "arm3_j2": -0.5,
            "arm3_j3": 0.0,
            "arm3_j4": -0.5,
            "arm3_j5": 0.0,
            "arm3_j6": -0.5,
            "arm3_j7": 0.0,
            
            # Front right leg
            "arm4_j1": 0.0,
            "arm4_j2": -0.5,
            "arm4_j3": 0.0,
            "arm4_j4": -0.5,
            "arm4_j5": 0.0,
            "arm4_j6": -0.5,
            "arm4_j7": 0.0,
        }
    ),
    actuators={
        # Front right leg actuators
        "fr_cluster1": ImplicitActuatorCfg(
            joint_names_expr=["arm1_j1", "arm1_j2"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "fr_cluster2": ImplicitActuatorCfg(
            joint_names_expr=["arm1_j3", "arm1_j4"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "fr_cluster3": ImplicitActuatorCfg(
            joint_names_expr=["arm1_j5", "arm1_j6", "arm1_j7"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        # Front left leg actuators
        "fl_cluster1": ImplicitActuatorCfg(
            joint_names_expr=["arm2_j1", "arm2_j2"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "fl_cluster2": ImplicitActuatorCfg(
            joint_names_expr=["arm2_j3", "arm2_j4"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "fl_cluster3": ImplicitActuatorCfg(
            joint_names_expr=["arm2_j5", "arm2_j6", "arm2_j7"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        # Back right leg actuators
        "br_cluster1": ImplicitActuatorCfg(
            joint_names_expr=["arm3_j1", "arm3_j2"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "br_cluster2": ImplicitActuatorCfg(
            joint_names_expr=["arm3_j3", "arm3_j4"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "br_cluster3": ImplicitActuatorCfg(
            joint_names_expr=["arm3_j5", "arm3_j6", "arm3_j7"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        # Back left leg actuators
        "bl_cluster1": ImplicitActuatorCfg(
            joint_names_expr=["arm4_j1", "arm4_j2"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "bl_cluster2": ImplicitActuatorCfg(
            joint_names_expr=["arm4_j3", "arm4_j4"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
        "bl_cluster3": ImplicitActuatorCfg(
            joint_names_expr=["arm4_j5", "arm4_j6", "arm4_j7"],
            effort_limit_sim=400.0,
            # velocity_limit=100.0,
            stiffness=20.0,
            damping=1.0,
        ),
    },
)
# """Configuration for the SSTI Crab robot."""

# ====================================================================== 
# Scene configuration 
# ====================================================================== 

class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot
    Crab = CRAB_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    # Crab: ArticulationCfg = CRAB_CFG.replace(prim_path="{ENV_REGEX_NS}/Crab")



# ====================================================================== 
# Simulator 
# ====================================================================== 

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():

        sim.step() 



# ====================================================================== 
# Main 
# ====================================================================== 

def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])

    # design scene
    scene_cfg = NewRobotsSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene)

# ---------------------------------- 
# ---------------------------------- 

if __name__ == "__main__":

    main()
    simulation_app.close()
