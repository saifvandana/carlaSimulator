import time
import argparse
import glob
import sys
import os
from numpy import random
from DReyeVR_utils import find_ego_vehicle
import math

try:
    sys.path.append(
        glob.glob(
            "../carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass

import carla

sys.path.append('../')
sys.path.insert(0, r'C:\DReyeVR\carla\PythonAPI\carla')

def spawn_other_vehicles(client, max_vehicles, world, traffic_manager):
    spawn_points = world.get_map().get_spawn_points()

    blueprints = world.get_blueprint_library().filter("vehicle.*")
    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    vehicle_list = []
    batch = []
    for n, transform in enumerate(spawn_points):
        if n >= max_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute("color"):
            color = random.choice(blueprint.get_attribute("color").recommended_values)
            blueprint.set_attribute("color", color)
        if blueprint.has_attribute("driver_id"):
            driver_id = random.choice(
                blueprint.get_attribute("driver_id").recommended_values
            )
            blueprint.set_attribute("driver_id", driver_id)
        try:
            blueprint.set_attribute("role_name", "autopilot")
        except IndexError:
            pass

        batch.append(
            SpawnActor(blueprint, transform).then(
                SetAutopilot(FutureActor, True, traffic_manager.get_port())
            )
        )
    synchronous_master = False
    for response in client.apply_batch_sync(batch, synchronous_master):
        if response.error:
            print(f"ERROR: {response.error}")
        else:
            vehicle_list.append(response.actor_id)
    print(f"successfully spawned {len(vehicle_list)} vehicles")
    return vehicle_list

def main():
    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10000.0)

        # Get the world object
        world = client.get_world()

        # Load a new map
        new_map = 'Town06'
        world = client.load_world(new_map)

        DReyeVR_vehicle = find_ego_vehicle(world)
        DReyeVR_vehicle.attributes['role_name'] = 'hero'
        settings = world.get_settings()

        
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True 
        # settings.actor_active_distance = 300
        # settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        # traffic_manager.set_hybrid_physics_mode(True)
        # traffic_manager.set_hybrid_physics_radius(50.0)
        # traffic_manager.set_respawn_dormant_vehicles(True)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        other_vehicles = spawn_other_vehicles(
            client, 10, world, traffic_manager
        )
        world.tick()

        while True:
            world.tick()
    finally:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("\ndone.")