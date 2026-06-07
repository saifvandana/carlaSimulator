import glob
import os
import sys
import time
import socket
import struct
from tkinter import *
from copy import deepcopy

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
import math
sys.path.append('../')
sys.path.insert(0,r'C:\DReyeVR\carla\PythonAPI\carla')
# To import a behavior agent
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner
import argparse
from numpy import random
from DReyeVR_utils import find_ego_vehicle

UDP_IP = "localhost"
UDP_PORT = 3002

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def set_DReyeVR_autopilot(world, traffic_manager):
    
    DReyeVR_vehicle = find_ego_vehicle(world)
    if DReyeVR_vehicle is not None:
        DReyeVR_vehicle.set_autopilot(False, traffic_manager.get_port())
        print("Successfully set autopilot on ego vehicle")
    return DReyeVR_vehicle

def compute_distance(world,ego_vehicle, wayP):
    spawn_points = world.get_map().get_spawn_points()
    sampling_resolution = 2
    grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
    a = carla.Location(ego_vehicle.get_location())
    b = carla.Location(spawn_points[wayP].location)
    w1 = grp.trace_route(a,b)
    return w1

def draw_ways(wayP,world):
    for w in wayP:
        t = w[0].transform
        begin = t.location + carla.Location(z=0.1)
        angle = math.radians(t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, thickness=0.1, arrow_size=0.1, life_time=1)

def spawn_other_vehicles(client, max_vehicles, world, traffic_manager, position):
    spawn_points = world.get_map().get_spawn_points()
    sampling_resolution = 2
    grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
    waypoints = [261,262]
    ways = []
    for i in range(0,len(waypoints)-1):
        a = carla.Location(spawn_points[waypoints[i]].location)
        b = carla.Location(spawn_points[waypoints[i+1]].location)
        w1 = grp.trace_route(a,b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
        print(i)
        ways+=w1
    for w in ways:
        t = w[0].transform
        begin = t.location + carla.Location(z=0.1)
        angle = math.radians(t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        # world.debug.draw_arrow(begin, end, thickness=0.1, arrow_size=0.1, life_time=0)
    random.shuffle(spawn_points)
    blueprints = world.get_blueprint_library().filter("vehicle.*")
    blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
    blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
    blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
    blueprints = [x for x in blueprints if not x.id.endswith('t2')]
    blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
    blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
    blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]
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

class Window(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master = master
        self.label = Label(text="", fg="Red", font=("Helvetica", 18), bg="black")
        self.label.place(relx=0.5, rely=0.5, anchor=CENTER)

    def update_clock(self, displayText):
        self.label.configure(text=displayText)

def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)",
    )
    argparser.add_argument(
        "-n",
        "--number-of-vehicles",
        metavar="N",
        default=60,
        type=int,
        help="number of vehicles (default: 10)",
    )
    argparser.add_argument(
        "--tm-port",
        metavar="P",
        default=8000,
        type=int,
        help="port to communicate with TM (default: 8000)",
    )
    argparser.add_argument(
        "-s", "--seed", metavar="S", type=int, help="Random device seed"
    )
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    random.seed(args.seed if args.seed is not None else int(time.time()))

    # Load a specific map
    world = client.load_world('Town03')
    other_vehicles = []
    ego_vehicle = None  
    root = Tk()
    app = Window(root)
    #Make the window borderless
    root.overrideredirect(True)
    root.attributes("-topmost", True)
    root.config(bg = '#000000')
    root.wm_title("Tkinter window")
    root.geometry("400x150+1200+830")
    # root.mainloop()
    try:
        world = client.get_world()
        world.set_weather(carla.WeatherParameters.ClearNoon)
        m = world.get_map()
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_hybrid_physics_mode(False)
        # traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        ego_vehicle = set_DReyeVR_autopilot(world, traffic_manager)
        # spawn other vehicles
        other_vehicles = spawn_other_vehicles(
            client, args.number_of_vehicles, world, traffic_manager, ego_vehicle.get_location()
        )
        wayDirections = {122:"keep straight in the roundabout",230:"take the next right and keep straight", 183:"Take the next right", 
                         98:"take the next left", 56:"Continue straight",
                         141:"Take the next right", 192:"take right again and keep straight",76:"take left and continue straight",65:"Continue straight",109:"take the next left", 
                         68:"Take the next right",179:"take right again",194:"Continue straight", 58:"Take next right", 48:"take right again"}
                         
        wayD = deepcopy(wayDirections)
        while True:
            root.update()
            world.wait_for_tick()
            # print(ego_vehicle.get_location())
            if bool(wayD):
                ways = compute_distance(world,ego_vehicle,list(wayD.keys())[0])
                dist_to_dest = len(ways)
                egoLoc = ego_vehicle.get_location()
                # draw_ways(ways[:6],world)
                if(dist_to_dest<15):
                    first_key = next(iter(wayD))
                    first_value = wayD.pop(first_key)
                    app.update_clock(first_value)
                    print(wayD)
            else:
                wayD = deepcopy(wayDirections)
                print("looping...")    
            try:
                acceleration = ego_vehicle.get_acceleration()

                x_accel = acceleration.x
                y_accel = acceleration.y
                z_accel = 9.8

                data = struct.pack('fff', x_accel, -y_accel, z_accel)

                udp_socket.sendto(data, (UDP_IP, UDP_PORT))

            except KeyboardInterrupt:
                break

    finally:
        if ego_vehicle is not None:
            ego_vehicle.set_autopilot(False, traffic_manager.get_port())
        print("\ndestroying %d vehicles" % len(other_vehicles))
        client.apply_batch([carla.command.DestroyActor(x) for x in other_vehicles])
        udp_socket.close()
        time.sleep(0.5)


if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("\ndone.")