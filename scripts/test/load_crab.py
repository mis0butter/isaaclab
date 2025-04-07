import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on spawning and interacting with an articulation.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext

# ---------------------------------- 
# load crab cfg 
# ---------------------------------- 
# from isaaclab_assets.robots.crab import CRAB_CFG
from isaaclab_assets import CRAB_CFG

# ====================================================================== 
# ====================================================================== 

def design_scene():
    """Design the scene with the crab robot."""

    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)

    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Create origin 
    origin = [0.0, 0.0, 2.0]

    # Origin 1
    prim_utils.create_prim("/World/Origin", "Xform", translation=origin)

    # Articulation
    crab_cfg = CRAB_CFG.copy()
    crab_cfg.prim_path = "/World/Origin/Robot"
    crab = Articulation(cfg=crab_cfg)

    # return the scene information
    scene_entities = {"crab": crab}

    return scene_entities, origin

# ====================================================================== 
# ====================================================================== 

def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, Articulation], origin: torch.Tensor):
    """Run the simulator."""

    # Extract scene entities
    robot = entities["crab"]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0 

    # Simulation loop
    while simulation_app.is_running():

        # -- write data to sim
        robot.write_data_to_sim()

        # Perform step
        sim.step()

        # Increment counter
        count += 1

        # Update buffers
        robot.update(sim_dt)

# ====================================================================== 
# ====================================================================== 

def main():
    """Main function."""

    # Load kit helper with gravity turned off 
    # sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim_cfg = sim_utils.SimulationCfg(
        device=args_cli.device,
        gravity=(0.0, 0.0, 0.0)  # Set gravity to zero in all directions
    )
    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Design scene
    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene_entities, scene_origins)

# ====================================================================== 
# ====================================================================== 

if __name__ == "__main__":
    
    # run the main function
    main()

    # close sim app
    simulation_app.close()
            
