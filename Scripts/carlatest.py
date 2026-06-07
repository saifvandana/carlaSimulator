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
            "C:/Carla/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg"
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
sys.path.insert(0, r'C:\Carla\carla\PythonAPI\carla')

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

    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10000.0)

    # Get the world object
    world = client.get_world()

    # Load a new map
    new_map = 'Town11'
    world = client.load_world(new_map)

    # bp = world.get_blueprint_library().find('sensor.camera.rgb')
    # bp.set_attribute('role_name', 'hero')

    # Spawn the ego vehicle
    bp = random.choice(world.get_blueprint_library().filter('vehicle'))
    bp.set_attribute('role_name', 'hero')
    spawn_point = random.choice(world.get_map().get_spawn_points())
    ego = world.try_spawn_actor(bp, spawn_point)

    if ego:
        print(f"Vehicle spawned at {spawn_point.location}")
        
        # Move the spectator to focus on the ego vehicle
        spectator = world.get_spectator()
        
        # Set the location and orientation of the spectator camera
        transform = ego.get_transform()
        location = transform.location + carla.Location(x=3, y=1.5, z=0.8)  # Adjust offset as needed
    else:
        print("Failed to spawn vehicle")

    DReyeVR_vehicle = find_ego_vehicle(world)
    DReyeVR_vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    DReyeVR_vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    DReyeVR_vehicle.set_transform(carla.Transform(location, transform.rotation))
    DReyeVR_vehicle.attributes['role_name'] = 'hero'

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_hybrid_physics_mode(True)
    traffic_manager.set_global_distance_to_leading_vehicle(1.0)
    other_vehicles = spawn_other_vehicles(
        client, 50, world, traffic_manager
    )


    while True:
        world.wait_for_tick()

if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("\ndone.")