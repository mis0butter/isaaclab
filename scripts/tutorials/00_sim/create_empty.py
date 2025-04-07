# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to create a simple stage in Isaac Sim.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py

"""

"""Launch Isaac Sim Simulator first."""

# ====================================================================== 
# ====================================================================== 

# argparse is used to parse command-line arguments 
# makes it easy to write user-friendly command-line interfaces 
import argparse

# app.AppLauncher wraps around isaacsim.SimulationApp class to launch simulator 
# provides mechanisms to configure simulator using command-line arguments and 
# environment variables 
from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher   = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

# import python modules 
from isaaclab.sim import SimulationCfg, SimulationContext

# ====================================================================== 
# ====================================================================== 

def main():
    """Main function."""

    # ---------------------------------- 
    # Initialize the simulation context 
    # ---------------------------------- 

    # create simulation config - an object that contains all the configuration 
    # is an instance of the simulation context 
    sim_cfg = SimulationCfg(dt = 0.01)

    # create simulation context 
    # we have only configured physics acting on simulated scene 
    # includes device to use for sim, gravity, and other solver params 
    # have not added sensors, robots, objects, etc. yet 
    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # ---------------------------------- 
    # Simulate physics 
    # ---------------------------------- 

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()

# ====================================================================== 
# ====================================================================== 

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

