import time
import argparse
import glob
import sys
import os
from numpy import random
from DReyeVR_utils import find_ego_vehicle
import math
import keyboard
import asyncio
import websockets
import json
import pygame
import logging
import networkx as nx

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

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("VehicleServer")

def calculate_rpm(speed_kmph, wheel_radius_cm):
    speed_cm_per_min = (speed_kmph * 1000 * 100) / 60
    circumference_cm = 2 * math.pi * wheel_radius_cm
    rpm = speed_cm_per_min / circumference_cm
    
    return rpm / 100

def spawn_other_vehicles(client, max_vehicles, world, traffic_manager):
    spawn_points = world.get_map().get_spawn_points()

    #blueprints = world.get_blueprint_library().filter("vehicle.*")
    #print(blueprints)
    #blueprints = sorted(blueprints, key=lambda bp: bp.id)
    
    blueprints = world.get_blueprint_library().filter("vehicle.*")
    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    # Define a set with the IDs you want to exclude
    exclusions = {
        "vehicle.carlamotors.carlacola",
        "vehicle.carlamotors.firetruck",
        "vehicle.carlamotors.european_hgv",
        "vehicle.vespa.zx125",
        "vehicle.gazelle.omafiets",
        "vehicle.diamondback.century",
        "vehicle.bh.crossbike"
    }

    # Filter out any blueprint whose id is in the exclusions set
    blueprints = [bp for bp in blueprints if bp.id not in exclusions]

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
    print(f"successfully spawned {vehicle_list} vehicles")
    return vehicle_list

def get_environment_conditions(world):
    """
    Returns current weather conditions, temperature, and time in CARLA.
    
    """
    # Get current weather and time
    weather = world.get_weather()
    tod = world.get_snapshot().timestamp
    
    # get current time
    from datetime import datetime
    time_now = datetime.now().strftime("%H:%M")
    
    # Determine general weather condition
    weather_condition = "Clear"
    if weather.precipitation > 50:
        weather_condition = "Heavy Rain" if weather.precipitation > 75 else "Rain"
    elif weather.fog_density > 50:
        weather_condition = "Foggy"
    elif weather.cloudiness > 50:
        weather_condition = "Cloudy"
    elif weather.wetness > 30:
        weather_condition = "Rainy"
    
    # Estimate temperature based on sun position (simplified model)
    temp = 20 + (weather.sun_altitude_angle / 4)  # Base 20°C with sun influence
    
    return {
        'weather': weather_condition,
        'temperature': round(temp, 1),
        'time': time_now,
        'detailed_weather': {
            'cloudiness': weather.cloudiness,
            'precipitation': weather.precipitation,
            'fog_density': weather.fog_density,
            'wind_intensity': weather.wind_intensity,
            'sun_altitude': weather.sun_altitude_angle
        }
    }


def get_vehicle_light_state(vehicle):
    """
    Returns the current light state of a CARLA vehicle.

    """
    light_state = vehicle.get_light_state()
    
    return {
        'headlights': 'high_beam' if light_state == carla.VehicleLightState.HighBeam 
                     else 'low_beam' if light_state == carla.VehicleLightState.LowBeam 
                     else 'off',
        'fog_lights': bool(light_state & carla.VehicleLightState.Fog),
        'position_lights': bool(light_state & carla.VehicleLightState.Position),
        'left_blinker': bool(light_state & carla.VehicleLightState.LeftBlinker),
        'right_blinker': bool(light_state & carla.VehicleLightState.RightBlinker),
        'brake_lights': bool(light_state & carla.VehicleLightState.Brake),
        'reverse_lights': bool(light_state & carla.VehicleLightState.Reverse),
        'interior_light': bool(light_state & carla.VehicleLightState.Interior),
        'light_state': light_state  # Raw bitmask value
    }

def is_wrong_way(vehicle, waypoint):
    """Check if vehicle is going wrong way in this lane"""
    vehicle_forward = vehicle.get_transform().get_forward_vector()
    lane_forward = waypoint.transform.get_forward_vector()
    dot_product = vehicle_forward.x * lane_forward.x + vehicle_forward.y * lane_forward.y
    return dot_product < 0

def is_off_road(vehicle, margin=0.5):
    # Get vehicle dimensions
    bb = vehicle.bounding_box
    vehicle_width = 2 * abs(bb.extent.y)  # y is lateral in vehicle coordinates
    
    # Get current waypoint and lane boundaries
    world = vehicle.get_world()
    map = world.get_map()
    vehicle_location = vehicle.get_location()
    current_waypoint = map.get_waypoint(vehicle_location)
    
    if not current_waypoint:
        return (False, 0, 0)  # Not on any road
    
    # Get left and right boundaries
    left_boundary = current_waypoint.get_left_lane()
    right_boundary = current_waypoint.get_right_lane()
    
    # Calculate distances to boundaries
    vehicle_transform = vehicle.get_transform()
    
    # Left distance calculation
    if left_boundary:
        left_location = left_boundary.transform.location
        left_distance = vehicle_location.distance(left_location)
    else:
        left_distance = float('inf')  # No left boundary (maybe leftmost lane)
    
    # Right distance calculation
    if right_boundary:
        right_location = right_boundary.transform.location
        right_distance = vehicle_location.distance(right_location)
    else:
        right_distance = float('inf')  # No right boundary (maybe rightmost lane)
    
    # Check if vehicle is within lane boundaries considering its width
    required_space = vehicle_width/2 + margin
    in_lane = (left_distance >= required_space) and (right_distance >= required_space)
    
    return in_lane

def validate_vehicle_lane(vehicle):
    world = vehicle.get_world()
    map = world.get_map()
    vehicle_location = vehicle.get_location()
    current_waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    result = {
        'valid': True,
        'off_road': False,
        'lane departure': False,
    }
    
    # Check for lane departure
    vehicle_forward = vehicle.get_transform().get_forward_vector()
    lane_forward = current_waypoint.transform.get_forward_vector()
    
    dot_product = vehicle_forward.x * lane_forward.x + vehicle_forward.y * lane_forward.y
    result['lane departure'] = dot_product < 0  # Opposite directions
    
    # Check for loff road
    result['off_road'] = is_off_road(vehicle)
    
    # Check if we've fully entered another lane
    current_road_id = current_waypoint.road_id
    current_lane_id = current_waypoint.lane_id
    
    closest_waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if closest_waypoint and (closest_waypoint.road_id != current_road_id or closest_waypoint.lane_id != current_lane_id):
        if is_wrong_way(vehicle, closest_waypoint):
            result['lane departure'] = True
    
    # Determine overall validity
    result['valid'] = not any([result['off_road'], result['lane departure']])
    
    return result
import networkx as nx
from math import atan2, degrees

def get_road_distance(vehicle, destination_location):
    """
    Road distance to destination calculation using CARLA's topology.
    """
    world = vehicle.get_world()
    map = world.get_map()
    topology = map.get_topology()
    
    start_wp = map.get_waypoint(vehicle.get_location())
    end_wp = map.get_waypoint(destination_location)

    # Navigation
    navigation = {
        'next_turn': 'straight',
        'turn_distance': 0
    }
    
    if not start_wp or not end_wp:
        # print("straight distance")
        return vehicle.get_location().distance(destination_location), navigation
    
    # Convert topology to graph
    graph = nx.Graph()
    
    for segment in topology:
        entry_wp, exit_wp = segment[0], segment[1]
        graph.add_edge(entry_wp, exit_wp, weight=entry_wp.transform.location.distance(exit_wp.transform.location))
    
    try:
        # Find shortest path using Dijkstra's algorithm
        path = nx.shortest_path(graph, source=start_wp, target=end_wp, weight='weight')
        
        # Calculate total distance
        total_distance = 0.0
        for i in range(len(path)-1):
            total_distance += path[i].transform.location.distance(path[i+1].transform.location)
        
        # Detect turns in the path (next 3 waypoints)
        if len(path) >= 3:
            # Get vehicle's current forward vector
            vehicle_forward = vehicle.get_transform().get_forward_vector()
            
            # Get vectors between waypoints
            vec1 = path[1].transform.location - path[0].transform.location
            vec2 = path[2].transform.location - path[1].transform.location
            
            # Calculate angles
            angle1 = degrees(atan2(vec1.y, vec1.x))
            angle2 = degrees(atan2(vec2.y, vec2.x))
            angle_diff = (angle2 - angle1 + 180) % 360 - 180
            
            # Determine turn direction
            if angle_diff < -15:  # Left turn threshold
                navigation['next_turn'] = 'left'
                navigation['turn_distance'] = path[0].transform.location.distance(path[1].transform.location)
            elif angle_diff > 15:  # Right turn threshold
                navigation['next_turn'] = 'right'
                navigation['turn_distance'] = path[0].transform.location.distance(path[1].transform.location)
            else:
                navigation['turn_distance'] = 0

        # if navigation['turn_distance'] == 0:
        #     return total_distance, None
        # else:
            return total_distance, navigation
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return vehicle.get_location().distance(destination_location), navigation

def get_nearest_traffic_light(vehicle):
    """
    Get the state and distance of the nearest traffic light to the vehicle
    
    """
    world = vehicle.get_world()
    traffic_lights = world.get_actors().filter('traffic.traffic_light*')
    
    # Find nearest traffic light
    vehicle_location = vehicle.get_location()
    nearest_light = None
    min_distance = float('inf')

    if not traffic_lights:
        return {'state': 'None', 'distance': -1, 'actor': None}
    
    for light in traffic_lights:
        light_location = light.get_location()
        distance = vehicle_location.distance(light_location)
        
        if distance < min_distance:
            min_distance = distance
            nearest_light = light
    
    # Get traffic light state
    state_map = {
        carla.TrafficLightState.Green: 'Green',
        carla.TrafficLightState.Yellow: 'Yellow',
        carla.TrafficLightState.Red: 'Red',
        carla.TrafficLightState.Off: 'Off'
    }
    
    return {
        'state': state_map.get(nearest_light.get_state(), 'Unknown'),
        'distance': min_distance,
        'actor': nearest_light
    }

def detect_lane_departure(vehicle, min_departure_distance=1):
    """
   lane departure detection for bidirectional roads
    
    """
    world = vehicle.get_world()
    map = world.get_map()
    vehicle_location = vehicle.get_location()
    vehicle_transform = vehicle.get_transform()
    
    # Get current waypoint with enhanced projection
    current_waypoint = map.get_waypoint(
        vehicle_location,
        project_to_road=True,
        lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Bidirectional))
    
    if not current_waypoint:
        return {
            'is_departing': True,
            'direction': 'offroad',
            'distance_to_left': 0.0,
            'distance_to_right': 0.0,
            'current_lane_width': 0.0,
            'lane_type': 'Offroad',
            'road_id': -1,
            'is_wrong_way': False
        }
    
    # Check if going wrong way in bidirectional lane
    is_wrong_way = False
    if current_waypoint.lane_type == carla.LaneType.Bidirectional:
        lane_direction = current_waypoint.transform.get_forward_vector()
        vehicle_direction = vehicle_transform.get_forward_vector()
        dot_product = lane_direction.x * vehicle_direction.x + lane_direction.y * vehicle_direction.y
        is_wrong_way = dot_product < 0  # Opposite directions
    
    # Calculate vehicle's lateral offset (sign-aware)
    lane_width = current_waypoint.lane_width
    waypoint_transform = current_waypoint.transform
    offset = vehicle_location - waypoint_transform.location
    right_vector = waypoint_transform.get_right_vector()
    lateral_offset = offset.dot(right_vector)  # Positive = right side, Negative = left side
    
    # Get boundary distances (considering lane ID direction)
    if current_waypoint.lane_id > 0:  # Standard direction
        left_distance = abs(-lane_width/2 - lateral_offset)
        right_distance = abs(lane_width/2 - lateral_offset)
    else:  # Opposite direction lanes
        left_distance = abs(lane_width/2 - lateral_offset)
        right_distance = abs(-lane_width/2 - lateral_offset)
    
    # Determine departure direction
    direction = 'none'
    is_departing = False
    
    if left_distance < min_departure_distance:
        direction = 'left'
        is_departing = True
    elif right_distance < min_departure_distance:
        direction = 'right'
        is_departing = True
    
    # Override for wrong-way driving
    if is_wrong_way:
        direction = 'wrong_way'
        is_departing = True

    
    # Override for off-road driving
    off_road = is_off_road(vehicle)
    if off_road:
        direction = 'off_road'
        is_departing = True
    
    return {
        'is_departing': is_departing,
        'direction': direction,
        'distance_to_left': left_distance,
        'distance_to_right': right_distance,
        'current_lane_width': lane_width,
        'lane_type': str(current_waypoint.lane_type).split('.')[-1],
        'road_id': current_waypoint.road_id,
        'is_wrong_way': is_wrong_way
    }

def get_driving_direction(vehicle):
    """
    Detect if vehicle is moving toward left or right side of current lane
    Returns:
        str: 'left', 'right', or 'center'
    """
    # Get current waypoint
    world = vehicle.get_world()
    waypoint = world.get_map().get_waypoint(
        vehicle.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    
    if not waypoint:
        return 'center'  # Off-road case
    
    # Calculate lateral offset
    vehicle_loc = vehicle.get_location()
    wp_loc = waypoint.transform.location
    wp_right = waypoint.transform.get_right_vector()
    
    offset = vehicle_loc - wp_loc
    lateral_offset = offset.dot(wp_right)  # Positive = right side, Negative = left side
    
    # Determine direction with threshold (0.5m)
    if lateral_offset < -0.5:
        return 'left'
    elif lateral_offset > 0.5:
        return 'right'
    else:
        return 'center'
    
class NavigationRoute:
    def __init__(self, world, draw_obj):
        self.world = world
        self.map = world.get_map()
        self.route_markers = []
        self.current_marker = 0
        self.debug_objects = []
        self.draw_obj = draw_obj
        
    def create_route(self, points, radius=5.0):
        """Create a route from list of locations"""
        self.clear()
        
        for i, point in enumerate(points):
            # Project to road surface
            waypoint = self.map.get_waypoint(point['location'], project_to_road=True)
            if not waypoint:
                # print(f"Warning: Point {i} not near road - skipping")
                continue
                
            marker = {
                'location': waypoint.transform.location,
                'radius': radius,
                'name': point.get('name', f"WP_{i}"),
                'turn': point.get('turn', 'straight'),
                'reached': False
            }
            
            if self.draw_obj:
                # Visualize with different colors
                if i == 0:
                    color = carla.Color(0, 255, 0)  # Green start
                elif i == len(points)-1:
                    color = carla.Color(255, 0, 0)  # Red end
              
                
                self.debug_objects.append(
                    self.world.debug.draw_point(
                        waypoint.transform.location + carla.Location(z=0.5),
                        size=0.2,
                        color=color,
                        life_time=0.0
                    )
                )
                     
            self.route_markers.append(marker)
        
        # print(f"Created route with {len(self.route_markers)} points")
        
    def check_progress(self, vehicle):
        """Check vehicle position against route"""
        if not self.route_markers:
            return None
            
        vehicle_loc = vehicle.get_location()
        current_target = self.route_markers[self.current_marker]
        
        distance = vehicle_loc.distance(current_target['location'])
        
        # Check if reached current marker
        if distance <= current_target['radius']:
            current_target['reached'] = True
            self.debug_objects.append(
                self.world.debug.draw_string(
                    current_target['location'] + carla.Location(z=3),
                    "REACHED!",
                    color=carla.Color(255, 0, 0),
                    life_time=3.0
                )
            )
            
            # Move to next marker if available
            if self.current_marker < len(self.route_markers) - 1:
                self.current_marker += 1
                # print(f"Reached point {self.current_marker-1}, next point {self.current_marker}")
                return {'status': 'reached', 'point_index': self.current_marker-1}
            else:
                # print("Route completed!")
                return {'status': 'complete'}
        
        # Calculate direction to next point
        direction = current_target['location'] - vehicle_loc
        direction.z = 0  # Ignore vertical
        distance_2d = math.sqrt(direction.x**2 + direction.y**2)
        
        return {
            'status': 'enroute',
            'point_index': self.current_marker,
            'name': current_target['name'],
            'distance': distance_2d,
            'direction': current_target['turn']
        }
        
    def clear(self):
        """Clear current route"""
        for obj in self.debug_objects:
            obj.destroy()
        self.route_markers = []
        self.current_marker = 0
        self.debug_objects = []

def precise_respawn(vehicle, location, yaw=None, pitch=0.0, roll=0.0):

    try:
        # Convert inputs
        if not isinstance(location, carla.Location):
            location = carla.Location(*location)
        
        # Get or calculate yaw if not specified
        if yaw is None:
            waypoint = vehicle.get_world().get_map().get_waypoint(
                location,
                project_to_road=True
            )
            yaw = waypoint.transform.rotation.yaw if waypoint else 0.0
        
        # Create precise transform
        transform = carla.Transform(
            location,
            carla.Rotation(
                pitch=float(pitch),
                yaw=float(yaw),
                roll=float(roll)
        ))
        
        # Disable physics for clean move
        vehicle.set_simulate_physics(False)
        
        # Apply transform
        vehicle.set_transform(transform)
        
        # Verify orientation
        time.sleep(0.1)
        current_yaw = vehicle.get_transform().rotation.yaw
        yaw_diff = abs((current_yaw - yaw + 180) % 360 - 180)
        
        if yaw_diff > 5.0:  # More than 5 degrees off
            # Correction attempt
            correction = carla.Transform(
                location,
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
            )
            vehicle.set_transform(correction)
            time.sleep(0.1)
            yaw_diff = abs((vehicle.get_transform().rotation.yaw - yaw + 180) % 360 - 180)
        
        # Restore state
        vehicle.set_simulate_physics(True)
        
        if yaw_diff <= 10.0:  # Acceptable threshold
            # print(f"Respawn successful | Target yaw: {yaw:.1f}° | Actual yaw: {current_yaw:.1f}°")
            return True
        else:
            # print(f"Orientation mismatch: {yaw_diff:.1f}° difference")
            return False
            
    except Exception as e:
        print(f"Respawn failed: {str(e)}")
        try:
            vehicle.set_simulate_physics(True)
        except:
            pass
        return False

import threading

def voice_alert(text):
    def _speak():
        with threading.Lock():
            os.system(f'PowerShell -Command "Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak(\'{text}\');"')

    threading.Thread(target=_speak, daemon=True).start()

import pyttsx3

engine = pyttsx3.init()
engine.setProperty('rate', 150)

engine_lock = threading.Lock()

def voice_command(text):
    def _speak():
        with engine_lock:
            engine.say(text)
            engine.runAndWait()

    threading.Thread(target=_speak, daemon=True).start()

##########################################################################
class CarlaDataProvider:
    def __init__(self):
        self.client = None
        self.world = None
        self.vehicle = None
        self.simulation_running = False
        self.destination = carla.Location(x=-176.524918, y=259.331543, z=0.001704)
        self.last_data = None
        self.fuel_efficiency = 8.5
        self.tank_capacity = 60
        self.current_fuel = self.tank_capacity       
        self.last_location = carla.Location(x=0, y=0, z=0)
        self.total_distance = 0.0              
        self.count = 0
        self.drive_mode = True
        self.inlane = True
        self.destination_distance = 0
        self.route = {}
        self.speed = {}
        self.signs= {}
        self.route_signs = [
                {
                'location': carla.Location(x=146.524261, y=155.344589, z=0.001704),
                'name': "engine-warning"
            },
            {
                'location': carla.Location(x=-83.076134, y=-71.907326, z=0.001719),
                'name': "high-beam"
            },
            {
                'location': carla.Location(x=-176.524918, y=259.331543, z=0.001704),
                'name': "distance-to-empty middle"
            },
            
        ]
        self.speed_points = [
            {
                'location': carla.Location(x=147.036362, y=328.031311, z=0.001703),
                'name': "90"
            },
            {
                'location': carla.Location(x=146.524261, y=155.344589, z=0.001704),
                'name': "30"
            },
            {
                'location': carla.Location(x=540.631409, y=328.403931, z=0.001714),
                'name': "90"
            },
            {
                'location': carla.Location(x=543.133606, y=284.738739, z=0.001724),
                'name': "50" #highway
            },
            {
                'location': carla.Location(x=597.113037, y=567.470093, z=0.001739),
                'name': "90"
            },
            {
                'location': carla.Location(x=-83.076134, y=-71.907326, z=0.001719),
                'name': "90"
            },
            {
                'location': carla.Location(x=11.216847, y=64.452568, z=0.001692),
                'name': "50" #highway exit
            },
            {
                'location': carla.Location(x=-102.909363, y=127.547020, z=0.001731),
                'name': "90"
            },
            {
                'location': carla.Location(x=-190.177383, y=127.681396, z=0.001763),
                'name': "60"
            },
            {
                'location': carla.Location(x=-206.335480, y=127.092857, z=0.001780),
                'name': "30"
            },
            {
                'location': carla.Location(x=-231.149033, y=210.595779, z=0.001730),
                'name': "30"
            },
        ]
        self.route_points = [
            {
                'location': carla.Location(x=147.744904, y=341.236816, z=0.001735),
                'turn': 'straight',
                'name': "Start"
            },
            {
                'location': carla.Location(x=146.257706, y=37.636017, z=0.001594),
                'turn': 'right',
                'name': "FirstTurn"
            },
            {
                'location': carla.Location(x=294.444763, y=25.745869, z=0.001751),
                'turn': 'right',
                'name': "SecondTurn"
            },
            {
                'location': carla.Location(x=299.811646, y=75.293945, z=0.001749),
                'turn': 'left',
                'name': "ThirdTurn"
            },
             {
                'location': carla.Location(x=476.724884, y=83.235382, z=0.001731),
                'turn': 'right',
                'name': "fourthTurn"
            },
             {
                'location': carla.Location(x=477.238892, y=148.960892, z=0.001757),
                'turn': 'right',
                'name': "fifthTurn"
            },
             {
                'location': carla.Location(x=241.058472, y=154.899796, z=0.001708),
                'turn': 'left',
                'name': "sixthTurn"
            },
            {
                'location': carla.Location(x=235.379776, y=211.170441, z=0.001698),
                'turn': 'left',
                'name': "seventhTurn"
            },
            {
                'location': carla.Location(x=474.375916, y=223.631516, z=0.001741),
                'turn': 'right',
                'name': "eigthTurn"
            },
            {
                'location': carla.Location(x=482.904449, y=346.844330, z=0.001729),
                'turn': 'left',
                'name': "ninthTurn"
            },
            {
                'location': carla.Location(x=543.133606, y=284.738739, z=0.001724),
                'turn': 'right',
                'name': "tenthTurn"
            },
            {
                'location': carla.Location(x=11.216847, y=64.452568, z=0.001692),
                'turn': 'right',
                'name': "eleventhTurn"
            },
            {
                'location': carla.Location(x=-102.909363, y=127.547020, z=0.001731),
                'turn': 'right',
                'name': "twelveTurn"
            },
            {
                'location': carla.Location(x=-231.149033, y=210.595779, z=0.001730),
                'turn': 'left',
                'name': "thirteenthTurn"
            },
            {
                'location': carla.Location(x=-47.058300, y=216.165909, z=0.001754),
                'turn': 'right',
                'name': "thirteenthTurn"
            },
            {
                'location': carla.Location(x=-44.961567, y=259.453125, z=0.001729),
                'turn': 'right',
                'name': "thirteenthTurn"
            },
            {
                'location': carla.Location(x=-176.524918, y=259.331543, z=0.001704),
                'name': "Finish"
            }
        ]

        self.shared_data = {"speed_kmph": 0, 
                            "right_direction": False, 
                            "left_direction": False, 
                            "straight_direction": True,
                            "turn_signal": "", 
                            "turn_signal_distance": "", 
                            "route_completed": False,
                            "distance_to_empty": "",
                            "warning_crossing_distance": "", 
                            "warning_pedestrian_distance": "",
                            "warning_stop": "",
                            "traffic_state": "", 
                            "traffic_light_distance": 0,
                            "current_destination_distance": 0,
                            "total_destination_distance": 0, 
                            "distance_travelled": 80,
                            "rpm": 0, 
                            "speed_kmph": 0, 
                            "speed_limit": "speed_30", 
                            "speed_limit_distance": 0,
                            "distance_to_empty": "", 
                            "fog_light": False,
                            "high_beam": False,
                            "eco_mode": False,
                            "tpms": False,
                            "fuel_warning": False,
                            "engine_warning": False,
                            "high_beam": False,
                            "washer_fluid_warning": False,
                            "hazard_indicator": False,
                            "reverse_light": "OFF",
                            "right_blinker": "OFF",
                            "left_blinker": "OFF",
                            "hazard_indicator": False, 
                            "construction_distance": "",
                            "in_lane": True,
                            "right_lane_departure": False,
                            "left_lane_departure": False,
                            "off_road": False,
                            "drive_mode": "D",
                            "current_time": "12:00",
                            "weather_condition" : "clear",
                            "temperature": "9°C",
                            "fuel_status": "335",
                            "updated": False}
        
    async def connect_to_carla(self, host='localhost', port=2000):
        """Connect to CARLA simulator"""
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10000.0)
            self.world = self.client.get_world()

            # Load a new map
            new_map = 'bigmapfinal'
            self.world = self.client.load_world(new_map)
            self.simulation_running = True
            logger.info("Connected to CARLA")

            # Create route and speed points
            self.route = NavigationRoute(self.world, draw_obj=True)
            self.route.create_route(self.route_points) 
            self.speed = NavigationRoute(self.world, draw_obj=False)
            self.speed.create_route(self.speed_points)
            self.signs = NavigationRoute(self.world, draw_obj=False)
            self.signs.create_route(self.route_signs)

            self.vehicle = find_ego_vehicle(self.world)
            success = precise_respawn(self.vehicle, carla.Location(x=227.076004, y=348.574677, z=0.001707), yaw=None)
            self.vehicle.attributes['role_name'] = 'hero'
            settings = self.world.get_settings()
            self.destination_distance, nav = get_road_distance(self.vehicle, self.destination)
            self.shared_data["total_destination_distance"] = self.destination_distance
            traffic_manager = self.client.get_trafficmanager(8000)
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True 
            self.world.apply_settings(settings)

            traffic_manager.global_percentage_speed_difference(-10.0)
            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            other_vehicles = spawn_other_vehicles(
                self.client, 5, self.world, traffic_manager
            )
            # self.world.tick()
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to CARLA: {str(e)}")
            return False
    
    def update_fuel(self):
        """Update fuel consumption based on distance traveled"""
        current_location = self.vehicle.get_location()

        # Calculate distance since last update (in km)
        distance_km = self.last_location.distance(current_location) / 1000.0
        self.total_distance += distance_km

        # Calculate fuel used (simple linear model)
        fuel_used = distance_km / self.fuel_efficiency
        self.current_fuel -= fuel_used

        # Store current state for next update
        self.last_location = current_location

         # Prevent negative fuel
        self.current_fuel = max(0.0, self.current_fuel)

    def get_remaining_fuel_km(self):
        """Calculate remaining distance possible with current fuel"""
        return self.current_fuel * self.fuel_efficiency

    def get_vehicle_data(self):
        """Retrieve current vehicle data from CARLA"""
        if not self.simulation_running:
            return None
            
        try:
            self.world.tick()
            physics_control = self.vehicle.get_physics_control()
            wheelR = physics_control.wheels[0].radius
            velocity = (self.vehicle.get_velocity())
            waypoint = self.world.get_map().get_waypoint(self.vehicle.get_location())
            lms_speed = waypoint.get_landmarks_of_type(100,"274")
            lms_stop = waypoint.get_landmarks_of_type(30,"206")
            lms_pedestrian = waypoint.get_landmarks_of_type(30,"133")
            lms_levelcrossing = waypoint.get_landmarks_of_type(30,"150")
            # traffic_state = (self.vehicle.get_traffic_light())
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
            speedKMPH = round(speed*3.6,2)
            rpm = calculate_rpm(speedKMPH,wheelR)
            all_landmarks = waypoint.get_landmarks(30)
            # print("speed: ", speedKMPH)
            # print("rpm: ", rpm)

            # get speed limit
            # print(all_landmarks[1].name)
            #print(self.vehicle.get_location())
            try:
                if lms_speed[0].distance <= 2:
                    speed_limit = ""
                else:
                    speed_limit = lms_speed[0].name
                    self.shared_data["speed_limit"] = speed_limit.split('_')[1]
                    self.shared_data["speed_limit_distance"] = lms_speed[0].distance
                # print(speed_limit)
            except:
                self.shared_data["speed_limit"] = ""
                self.shared_data["speed_limit_distance"] = ""

            # get crossing warning
            try:
                if lms_levelcrossing[0].distance <= 2:
                    self.shared_data["warning_crossing_distance"] = ""
                else:
                    self.shared_data["warning_crossing_distance"] = lms_levelcrossing[0].distance
            except:
                self.shared_data["warning_crossing_distance"] = ""

            # get pedestrian warning
            try:
                if lms_pedestrian[0].distance <= 2:
                    self.shared_data["warning_pedestrian_distance"] = ""
                else:
                    self.shared_data["warning_pedestrian_distance"] = lms_pedestrian[0].distance
            except:
                self.shared_data["warning_pedestrian_distance"] = ""

            # get stop warning
            try:
                if lms_stop[0].distance <= 2:
                    self.shared_data["warning_stop"] = ""
                else:
                    self.shared_data["warning_stop"] = lms_stop[0].distance
            except:
                self.shared_data["warning_stop"] = ""

            # # get traffic info
            try:
                traffic_light_info = get_nearest_traffic_light(self.vehicle)
                if traffic_light_info['distance'] <= 2 or traffic_light_info['distance'] > 50:
                    self.shared_data["traffic_state"] = ""
                else:
                    self.shared_data["traffic_state"] = traffic_light_info['state']
                    self.shared_data["traffic_light_distance"] = round(traffic_light_info['distance'], 1)
            except:
                self.shared_data["traffic_state"] = ""

            # lane and road departure
            # try:
            #     lane_status = validate_vehicle_lane(self.vehicle)
            #     off_road = is_off_road(self.vehicle)
                
            #     if not lane_status['valid']:
            #         if lane_status['lane departure']:
            #             print("Vehicle has departed lane!")
            #             self.shared_data["in_lane"] = False
            #     else:
            #         self.shared_data["in_lane"] = True
            #     if not off_road:
            #         print("Vehicle is off road!")
            #         self.shared_data["off_road"] = True
            #     else:
            #         self.shared_data["off_road"] = False
            # except:
            #     self.shared_data["in_lane"] = True
            #     self.shared_data["off_road"] = False
            try:
                departure = detect_lane_departure(self.vehicle)
                if departure['is_departing']:
                    self.shared_data["in_lane"] = False
                    direction = get_driving_direction(self.vehicle)
                    # print(f"Departing {direction} side!")
                    if direction == 'right':
                        self.shared_data["right_lane_departure"] = True
                    else:
                        self.shared_data["left_lane_departure"] = True
                    # if departure['distance_to_left'] < departure['distance_to_right']:
                    #     print("Vehicle has departed lane from the left!")
                    #     self.shared_data["left_lane_departure"] = True
                    # else:
                    #     print("Vehicle has departed lane from the right!")
                    #     self.shared_data["right_lane_departure"] = True
                else:
                    self.shared_data["in_lane"] = True
                    self.shared_data["left_lane_departure"] = False
                    self.shared_data["right_lane_departure"] = False
            except:
                self.shared_data["in_lane"] = True
                self.shared_data["left_lane_departure"] = False
                self.shared_data["right_lane_departure"] = False
            
            # Get road distance to destination
            road_distance, nav = get_road_distance(self.vehicle, self.destination)
            # print(nav)
            # print(f"Total Road distance: {self.destination_distance:.2f} meters")
            # print(f"Remaining Road distance: {road_distance:.2f} meters")
            self.shared_data["current_destination_distance"] = round(road_distance, 1)
            # result, self.last_data = get_navigation_info(self.vehicle, self.destination, self.last_data)
            # print(result)
            # print(f"Traveled: {result['distance_traveled']:.1f}m")
            # print(f"Remaining: {result['distance_to_destination'] or 'UNREACHABLE'}m")
            # print(f"Next turn: {result['next_turn']} in {result['turn_distance']:.1f}m")
            

            # Get fuel status
            self.update_fuel()
            remaining_km = self.get_remaining_fuel_km()
            self.shared_data["fuel_status"] = int(remaining_km)
            # print(f"Remaining range: {remaining_km:.1f} km")

            # Retrieve information
            conditions = get_environment_conditions(self.world)
            self.shared_data["current_time"] = conditions['time']
            self.shared_data["weather_condition"] = conditions['weather']
            self.shared_data["temperature"] = conditions['temperature'] 
            # print(f"Current conditions: {conditions['weather']}, {conditions['temperature']}°C, Time: {conditions['time']}")

            #vehicle status light signal
            # lights = get_vehicle_light_state(self.vehicle)
            # self.shared_data["left_blinker"] = "ON" if lights['left_blinker'] else "OFF"
            # self.shared_data["right_blinker"] = "ON" if lights['right_blinker'] else "OFF"
            # self.shared_data["fog_light"] = "ON" if lights['fog_lights'] else "OFF"
            # self.shared_data["reverse_light"] = "ON" if lights['reverse_lights'] else "OFF"
            # print(f"Headlights: {lights['headlights']}")
            # print(f"Left blinker: {'ON' if lights['left_blinker'] else 'OFF'}")
            # print(f"Right blinker: {'ON' if lights['right_blinker'] else 'OFF'}")
            # print(f"Reverse Light: {'ON' if lights['reverse_lights'] else 'OFF'}")
            # print(f"Fog Light: {'ON' if lights['fog_lights'] else 'OFF'}")

            # Vehicle Drive Mode
            # self.shared_data["drive_mode"] = "R" if lights['reverse_lights'] else "D"
            v_mode = ["D","R"]
            # pygame.init()
            # pygame.joystick.init()
            # joystick = pygame.joystick.Joystick(0)
            # joystick.init()
            # if keyboard.is_pressed('z') or keyboard.is_pressed('alt') or (pygame.joystick.get_count() > 0 and pygame.joystick.Joystick(0).get_button(2)):
            if keyboard.is_pressed('z') or keyboard.is_pressed('alt'):
                if self.drive_mode: 
                    self.count += 1
                    # print("vehicle mode : ", v_mode[self.count % 2])
                    self.shared_data["drive_mode"] = v_mode[self.count % 2]
                    self.drive_mode = False
            else:
                self.drive_mode = True
 
            # control = self.vehicle.get_control()
            # if control.reverse:
            #     print("Vehicle is in reverse mode")
            # else:
            #     print("Vehicle is in drive mode")

            # Route check
            progress = self.route.check_progress(self.vehicle)   
            if progress:
                if progress['status'] == 'enroute':
                    if progress['distance'] < 200 and progress['direction'] != 'straight':
                        self.shared_data["turn_signal"] = progress['direction']
                        self.shared_data["turn_signal_distance"] = int(progress['distance'])
                        # print(f"Next point: {progress['name']} | Direction: {progress['direction']} | Distance: {progress['distance']:.1f}m")

                    else:
                        self.shared_data["turn_signal"] = 'straight'
                        self.shared_data["turn_signal_distance"] = int(progress['distance'])
                        # print(f"Next point: {progress['name']} | Direction: straight | Distance: {progress['distance']:.1f}m")
                    
                elif progress['status'] == 'complete':
                    self.shared_data["route_completed"] = True
                    # print("All points reached!")
            
            # speed sign check
            progress_speed = self.speed.check_progress(self.vehicle)   
            if progress_speed:
                if progress_speed['status'] == 'enroute':
                    if progress_speed['distance'] >= 200:
                        self.shared_data["speed_limit"] = ""
                        self.shared_data["speed_limit_distance"] = ""
                    else:
                        self.shared_data["speed_limit"] = progress_speed['name']
                        self.shared_data["speed_limit_distance"] = int(progress_speed['distance'])
                        # print(f"Next point: {progress['name']} | Direction: {progress['direction']} | Distance: {progress['distance']:.1f}m")

            # # road sign check
            progress_signs = self.signs.check_progress(self.vehicle)   
            if progress_signs:
                if progress_signs['status'] == 'enroute':
                    if progress_signs['distance'] < 10: 
                        if progress_signs['name'] == "distance-to-empty middle":
                            self.shared_data["distance_to_empty"] = "middle"
                            voice_alert("Check distance travelled")
                        if progress_signs['name'] == "engine-warning":
                            self.shared_data["engine_warning"] = True
                            voice_alert("Check engine warning")
                        if progress_signs['name'] == "high-beam":
                            self.shared_data["high_beam"] = True
                            voice_alert("Check light signals")
                                       
            # Update data
            self.shared_data["rpm"] = round(rpm, 1)
            self.shared_data["speed_kmph"] = int(speedKMPH)
            self.shared_data["updated"] = True
            # print(self.shared_data)

            return self.shared_data
            
        except Exception as e:
            logger.error(f"Error getting vehicle data: {str(e)}")
            return None

connected_clients = set()  # Global set to track all connected clients
carla_provider = None     # Shared CARLA provider instance
broadcast_task = None     # Global broadcast task

async def broadcast_carla_data():
    """Continuously broadcast data to all connected clients"""
    while True:
        if connected_clients:  # Only process if we have clients
            try:
                data = carla_provider.get_vehicle_data()
                if data:
                    message = json.dumps(data)
                    # Create list of clients to remove (disconnected ones)
                    disconnected_clients = set()
                    
                    # Send to all connected clients
                    for client in connected_clients:
                        try:
                            await client.send(message)
                        except (websockets.exceptions.ConnectionClosed, 
                               websockets.exceptions.WebSocketException):
                            disconnected_clients.add(client)
                    
                    # Remove disconnected clients
                    for client in disconnected_clients:
                        connected_clients.discard(client)
                        logger.info(f"Cleaned up disconnected client: {client.remote_address}")
                        
            except Exception as e:
                logger.error(f"Broadcast error: {e}")
        
        await asyncio.sleep(0.01)  # ~10 updates per second

async def carla_data_handler(websocket):
    """Handle WebSocket connections and manage client lifecycle"""
    global carla_provider, broadcast_task
    
    logger.info(f"New client connected: {websocket.remote_address}")
    connected_clients.add(websocket)
    
    try:
        # Initialize CARLA connection if this is the first client
        if len(connected_clients) == 1:
            carla_provider = CarlaDataProvider()
            connected = await carla_provider.connect_to_carla()
            
            if not connected:
                error_msg = json.dumps({"error": "Failed to connect to CARLA"})
                await websocket.send(error_msg)
                connected_clients.remove(websocket)
                return
            
            # Start broadcast task if not already running
            if broadcast_task is None:
                broadcast_task = asyncio.create_task(broadcast_carla_data())
        
        # Keep connection alive
        async for _ in websocket:
            pass
            
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"Client disconnected: {websocket.remote_address}")
    except Exception as e:
        logger.error(f"Error with client {websocket.remote_address}: {e}")
    finally:
        # Clean up client connection
        connected_clients.discard(websocket)
        
        # Shutdown CARLA if no more clients
        if len(connected_clients) == 0 and carla_provider is not None:
            if broadcast_task is not None:
                broadcast_task.cancel()
                try:
                    await broadcast_task
                except asyncio.CancelledError:
                    pass
                broadcast_task = None
            
            carla_provider.simulation_running = False
            carla_provider = None

#############################################################
async def main():
    # try:
    #     # Connect to the CARLA server
    #     client = carla.Client('localhost', 2000)
    #     client.set_timeout(10000.0)

    #     # Get the world object
    #     world = client.get_world()

    #     # Load a new map
    #     new_map = 'Town02'
    #     world = client.load_world(new_map)

    #     DReyeVR_vehicle = find_ego_vehicle(world)
    #     DReyeVR_vehicle.attributes['role_name'] = 'hero'
    #     settings = world.get_settings()

        
    #     traffic_manager = client.get_trafficmanager(8000)
    #     traffic_manager.set_synchronous_mode(True)
    #     settings.synchronous_mode = True 
    #     # settings.actor_active_distance = 300
    #     # settings.fixed_delta_seconds = 0.05
    #     world.apply_settings(settings)
    #     # traffic_manager.set_hybrid_physics_mode(True)age
    #     # traffic_manager.set_hybrid_physics_radius(50.0)
    #     # traffic_manager.set_respawn_dormant_vehicles(True)
    #     traffic_manager.global_percentage_speed_difference(-10.0)
    #     traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    #     other_vehicles = spawn_other_vehicles(
    #         client, 5, world, traffic_manager
    #     )
    #     physics_control = DReyeVR_vehicle.get_physics_control()
    #     wheelR = physics_control.wheels[0].radius
    #     world.tick()

    #     # All variables
    #     count = 0
    #     drive_mode = True
    #     inlane = True
    #     destination = carla.Location(x=100.5, y=200.3, z=0.5)

    #     # Create fuel tracker (parameters for a truck would be different)
    #     fuel_tracker = FuelTracker(DReyeVR_vehicle, fuel_efficiency=12.0, tank_capacity=70)

        # while True:
        #     world.tick()
        #     velocity = (DReyeVR_vehicle.get_velocity())
        #     # speed_limit = (DReyeVR_vehicle.get_speed_limit())
        #     waypoint = world.get_map().get_waypoint(DReyeVR_vehicle.get_location())
        #     lms_speed = waypoint.get_landmarks_of_type(100,"274")
        #     lms_stop = waypoint.get_landmarks_of_type(100,"206")
        #     lms_pedestrian = waypoint.get_landmarks_of_type(100,"133")
        #     lms_levelcrossing = waypoint.get_landmarks_of_type(100,"150")
        #     traffic_state = (DReyeVR_vehicle.get_traffic_light())
        #     speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        #     speedKMPH = round(speed*3.6,2)
        #     rpm = calculate_rpm(speedKMPH,wheelR)

        #     # # get drive mode D and R
        #     # v_mode = ["D","R"]
        #     # try:
        #     #     if keyboard.is_pressed('z') or keyboard.is_pressed('alt'):
        #     #         if drive_mode: 
        #     #             count += 1
        #     #             print("vehicle mode : ", v_mode[count % 2])
        #     #             drive_mode = False
        #     #     else:
        #     #         drive_mode = True
        #     # except:
        #     #     continue

        #     # get landmarks
        #     # landmarks= waypoint.get_landmarks(10)
        #     # try:
        #     #     print(landmarks[0].name)
        #     #     print(landmarks[0].distance)
        #     #     print(landmarks[0].type)
        #     # except:
        #     #     continue


        #     #get speed limit
        #     # try:
        #     #     if lms_speed[0].distance <= 2:
        #     #         speed_limit = None
        #     #     speed_limit = lms_speed[0].name
        #     #     print(speed_limit)
        #     # except:
        #     #     continue
            
      
        """Start the WebSocket server"""
        port = 8765
        host = "0.0.0.0"  # Listen on all network interfaces
        
        logger.info(f"Starting CARLA data server on ws://{host}:{port}")
        async with websockets.serve(carla_data_handler, host, port):
            await asyncio.Future()  # run forever
    
    # finally:
    #         settings = world.get_settings()
    #         settings.synchronous_mode = False
    #         settings.no_rendering_mode = False
    #         settings.fixed_delta_seconds = 0.0
    #         world.apply_settings(settings)
if __name__ == "__main__":

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        print("\ndone.")