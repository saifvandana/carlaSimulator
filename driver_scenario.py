# scenario3_signaldelay.py
import carla
import time
import threading
import math
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Tuple
import random
import pygame

class SignalState(Enum):
    RED = 0
    YELLOW = 1
    GREEN = 2
    LEFT_ARROW_RED = 3  # Red + left turn arrow

@dataclass
class LeadVehicle:
    actor: carla.Vehicle
    behavior_type: str  # "delayed_start", "partial_move", "right_turn_blocked"
    start_time: float  # When it was first stopped at light

class Scenario3_SignalDelay:
    def __init__(self, host='localhost', port=2000):
        
        # CARLA setup
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.load_world('Town05')  
        self.blueprint_library = self.world.get_blueprint_library()
        
        # Actors
        self.ego_vehicle = None
        self.lead_vehicles = []
        self.vehicles_behind = []  # For horn pressure
        self.pedestrians = []
        self.illegal_turn_vehicle = None
        
        # Traffic light reference
        self.intersection_traffic_light = None
        
        # Scenario timing
        self.scenario_start_time = None
        self.current_time = 0
        self.scenario_duration = 500  # seconds (end after final event)
        
        # Signal timing (as specified)
        self.signal_cycle = {
            'red': 60,
            'green': 55,
            'yellow': 5,
            'left_arrow_red': 30  # After green, red + left arrow
        }
        
        # Event tracking
        self.rating_prompts_shown = set()
        self.horn_count = 0
        self.last_horn_time = 0
        
        # State
        self.lead_vehicle_moved = False
        self.first_green_start_time = None
        self.second_green_start_time = None
        self.ego_at_light = False
        
        # Speed limit monitoring
        self.speed_limit = 50  # km/h
        self.speed_warning_active = False
        
        # Setup UI and sounds
        self.setup_audio_system()
        
    def setup_audio_system(self):
        """Setup pygame for sounds and UI"""
        try:
            import pygame
            pygame.mixer.init()
            self.horn_sound = None  # Load horn sound file
            self.beep_sound = None  # Load beep for sync
            self.screen = pygame.display.set_mode((800, 200))
            pygame.font.init()
            self.font = pygame.font.Font(None, 36)
        except:
            print("Warning: Pygame not available, using console output")
            self.screen = None
        
    def play_horn(self, from_behind: bool = True):
        """Play horn sound from vehicle behind"""
        if time.time() - self.last_horn_time < 0.5:  # Debounce
            return
            
        self.last_horn_time = time.time()
        self.horn_count += 1
        
        if from_behind:
            print(f"\n HONK! Vehicle behind you is honking (Event #{self.horn_count})")
        else:
            print(f"\n Horn from surrounding traffic")
            
        
    def setup_urban_road(self):
        """Setup the 2-lane urban road with correct lane markings"""
        # Get map and identify the intersection
        world_map = self.world.get_map()
        waypoints = world_map.generate_waypoints(2.0)
        
        # Find intersection point where Scenario 3 occurs
        # For Town05, there's a large intersection at x≈-50, y≈150
        # You'll need to adjust coordinates based on your spawn point
        
        # Set speed limit signs
        for vehicle in self.world.get_actors().filter('traffic.speed_limit.*'):
            vehicle.destroy()
        
        # Create speed limit signs at regular intervals
        # speed_limit_sign = self.world.spawn_actor(
        #     self.blueprint_library.find('static.speed_limit.50'),
        #     carla.Transform(carla.Location(x=-100, y=150, z=1))
        # )
        
        # Get the traffic light at the intersection
        traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
        for tl in traffic_lights:
            # Find the one controlling our intersection
            location = tl.get_location()
            if abs(location.x + 50) < 20 and abs(location.y - 150) < 20:
                self.intersection_traffic_light = tl
                break
                
        # Set custom timing for traffic light
        if self.intersection_traffic_light:
            self.set_traffic_light_timing()
            
    def set_traffic_light_timing(self):
        """Override traffic light timing to match scenario requirements"""
        self.manual_light_control = True
        self.start_light_controller()
        
    def start_light_controller(self):
        """Manually control traffic light cycle"""
        def control_cycle():
            if not self.intersection_traffic_light:
                return
                
            while self.current_time < self.scenario_duration:
                # Red: 60 seconds
                self.intersection_traffic_light.set_state(carla.TrafficLightState.Red)
                time.sleep(self.signal_cycle['red'])
                
                if self.current_time > self.scenario_duration:
                    break
                    
                # Green: 55 seconds
                self.intersection_traffic_light.set_state(carla.TrafficLightState.Green)
                time.sleep(self.signal_cycle['green'])
                
                if self.current_time > self.scenario_duration:
                    break
                    
                # Yellow: 5 seconds
                self.intersection_traffic_light.set_state(carla.TrafficLightState.Yellow)
                time.sleep(self.signal_cycle['yellow'])
                
        threading.Thread(target=control_cycle, daemon=True).start()
        
    def spawn_lead_vehicles(self):
        """Spawn 3 vehicles ahead of ego with specific behaviors"""
        # Get ego's location and waypoint
        ego_transform = self.ego_vehicle.get_transform()
        forward = ego_transform.get_forward_vector()
        
        # Spawn Vehicle 1 (directly ahead, will not move at green)
        spawn_loc1 = ego_transform.location + forward * 25.0  # 25m ahead
        blueprint1 = self.blueprint_library.filter('vehicle.toyota.prius')[0]
        vehicle1 = self.world.spawn_actor(blueprint1, carla.Transform(spawn_loc1))
        vehicle1.set_autopilot(False)
        
        # Spawn Vehicle 2 (ahead of vehicle 1)
        spawn_loc2 = spawn_loc1 + forward * 15.0
        blueprint2 = self.blueprint_library.filter('vehicle.mini.cooper_s')[0]
        vehicle2 = self.world.spawn_actor(blueprint2, carla.Transform(spawn_loc2))
        vehicle2.set_autopilot(False)
        
        # Spawn Vehicle 3 (right-turn ahead)
        spawn_loc3 = spawn_loc2 + forward * 12.0
        blueprint3 = self.blueprint_library.filter('vehicle.mercedes.coupe')[0]
        vehicle3 = self.world.spawn_actor(blueprint3, carla.Transform(spawn_loc3))
        vehicle3.set_autopilot(False)
        
        # Store with behaviors
        self.lead_vehicles = [
            LeadVehicle(vehicle1, "delayed_start", self.current_time),
            LeadVehicle(vehicle2, "partial_move", self.current_time),
            LeadVehicle(vehicle3, "right_turn_blocked", self.current_time)
        ]
        
        return self.lead_vehicles
        
    def spawn_traffic_behind(self):
        """Spawn vehicles behind ego that will honk when delayed"""
        ego_transform = self.ego_vehicle.get_transform()
        backward = -ego_transform.get_forward_vector()
        
        vehicles_behind = []
        for i in range(3):  # 3 vehicles behind
            spawn_loc = ego_transform.location + backward * (15.0 + i*8.0)
            blueprint = self.blueprint_library.filter('vehicle.audi.a2')[0]
            vehicle = self.world.spawn_actor(blueprint, carla.Transform(spawn_loc))
            vehicle.set_autopilot(True)  # They will follow traffic
            vehicles_behind.append(vehicle)
            
        self.vehicles_behind = vehicles_behind
        return vehicles_behind
        
    def spawn_jaywalking_pedestrian(self, timing: float):
        """Spawn pedestrian that crosses illegally"""
        def create_pedestrian():
            # Wait for specified time
            time.sleep(timing)
            
            # Spawn pedestrian near crosswalk but jaywalking
            ped_bp = self.blueprint_library.filter('walker.pedestrian.0001')[0]
            
            # Spawn at illegal crossing point (not at crosswalk)
            spawn_loc = carla.Location(x=-45, y=140, z=0.5)  # Adjust coordinates
            pedestrian = self.world.spawn_actor(ped_bp, carla.Transform(spawn_loc))
            
            # Make them walk across the road
            destination = carla.Location(x=-45, y=160, z=0.5)
            
            # Start walking
            ped_controller = self.world.spawn_actor(
                self.blueprint_library.find('controller.ai.walker'),
                carla.Transform(),
                pedestrian
            )
            ped_controller.start()
            ped_controller.go_to_location(destination)
            
            self.pedestrians.append((pedestrian, ped_controller))
            
            return pedestrian
            
        threading.Thread(target=create_pedestrian).start()
        
    def spawn_illegal_left_turn_vehicle(self, timing: float):
        """Spawn vehicle that makes illegal left turn"""
        def create_illegal_turn():
            time.sleep(timing)
            
            # Spawn from left side, turning illegally
            spawn_loc = carla.Location(x=-60, y=145, z=0.5)
            vehicle_bp = self.blueprint_library.filter('vehicle.bmw.isetta')[0]
            vehicle = self.world.spawn_actor(vehicle_bp, carla.Transform(spawn_loc))
            
            # Make it turn left from wrong lane (illegal)
            # This requires waypoint navigation - simplified here
            vehicle.set_autopilot(False)
            
            # Force movement (simplified)
            def illegal_movement():
                time.sleep(0.5)
                new_loc = vehicle.get_location()
                new_loc.x += 0.5
                new_loc.y += 0.3
                vehicle.set_location(new_loc)
                
            for _ in range(40):  # Move across intersection
                threading.Thread(target=illegal_movement).start()
                time.sleep(0.1)
                
            self.illegal_turn_vehicle = vehicle
            
        threading.Thread(target=create_illegal_turn).start()
        
    def check_lead_vehicle_behavior(self):
        """Monitor lead vehicles and implement delayed start logic"""
        # Vehicle 1: Only moves when light turns yellow
        if self.lead_vehicles and not self.lead_vehicles[0].behavior_type == "moved":
            light = self.intersection_traffic_light
            if light and light.get_state() == carla.TrafficLightState.Yellow:
                # Vehicle 1 finally moves
                self.lead_vehicles[0].actor.set_autopilot(True)
                self.lead_vehicles[0].behavior_type = "moved"
                print("Lead vehicle 1 finally moves at yellow!")
                
        # Vehicle 3: Blocked because of right turn
        if len(self.lead_vehicles) > 2:
            # Check if there's oncoming traffic for right turn
            # Simplified: wait 10 seconds after green
            if self.current_time > self.first_green_start_time + 10:
                self.lead_vehicles[2].actor.set_autopilot(True)
                
    def monitor_speed(self):
        """Monitor speed and enforce limit"""
        while self.current_time < self.scenario_duration:
            if self.ego_vehicle:
                speed = self.get_vehicle_speed()
                if speed > self.speed_limit + 5:  # 5 km/h buffer
                    if not self.speed_warning_active:
                        print(f"\n SPEED WARNING: {speed:.1f} km/h. Speed limit is {self.speed_limit} km/h")
                        self.display_message(f"Speed limit: {self.speed_limit} km/h", duration=2)
                        self.speed_warning_active = True
                else:
                    self.speed_warning_active = False
            time.sleep(0.5)
            
    def get_vehicle_speed(self) -> float:
        """Get current speed in km/h"""
        velocity = self.ego_vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return speed
        
    def display_message(self, message: str, duration: float = 3):
        """Display on screen and console"""
        print(f"\n[INSTRUCTION] {message}")
        
        if self.screen:
            # Pygame display
            self.screen.fill((0, 0, 0))
            text_surface = self.font.render(message, True, (255, 255, 255))
            self.screen.blit(text_surface, (20, 50))
            pygame.display.flip()
            
            if duration:
                threading.Timer(duration, self.clear_screen).start()
                
    def clear_screen(self):
        if self.screen:
            self.screen.fill((0, 0, 0))
            pygame.display.flip()
            
    def prompt_rating(self, event_type: str, timing_key: float):
        """Show anger rating prompt"""
        # Don't show if shown before
        if timing_key in self.rating_prompts_shown:
            return
            
        self.rating_prompts_shown.add(timing_key)
        
        prompt_messages = {
            180: "How much anger or frustration did you feel due to the delayed start of the lead vehicle?",
            273: "How much anger or frustration did you feel due to the honking from the vehicle behind you?",
            460: "How much anger or frustration did you feel due to the traffic rule violations you just experienced?"
        }
        
        message = prompt_messages.get(timing_key, "Rate your frustration (1-5)")
        
        print("\n" + "="*70)
        print(f"ANGER RATING PROMPT")
        print(f"{message}")
        print("1 = Not at all angry    2 = Slightly angry    3 = Moderately angry")
        print("4 = Very angry          5 = Extremely angry")
        print("="*70)
        
        # Get rating (in real implementation, use UI buttons)
        rating = input("Enter rating (1-5): ")
        
        # Log rating
        with open(f"scenario3_ratings_{time.strftime('%Y%m%d_%H%M%S')}.csv", "a") as f:
            timestamp = self.current_time
            f.write(f"{timestamp},{event_type},{rating}\n")
            
        print(f"✓ Rating recorded: {rating}/5\n")
        
    def check_and_prompt_horns(self):
        """Check if horns should sound (at 120s and 270s)"""
        # Horn pressure at 120s, every 10s until merge (in this case, until 300s)
        if self.current_time >= 120 and self.current_time < 300:
            # Every 10 seconds
            if int(self.current_time) % 10 == 0 and int(self.current_time) != self.last_horn_time:
                self.play_horn(from_behind=True)
                self.last_horn_time = int(self.current_time)
                
        # Second horn sequence from 270-300s
        if self.current_time >= 270 and self.current_time < 300:
            if int(self.current_time) % 10 == 0 and int(self.current_time) != self.last_horn_time:
                self.play_horn(from_behind=True)
                
    def update_timers(self):
        """Update scenario timers"""
        if self.scenario_start_time:
            self.current_time = time.time() - self.scenario_start_time
            
    def run_timeline(self):
        """Execute the complete scenario timeline"""
        # Initial instruction at 1s
        threading.Timer(1, lambda: self.display_message(
            "The speed limit is 50 km/h. Stay in lane 2.", duration=5
        )).start()
        
        # Approach phase (0-50s)
        print("\n APPROACHING INTERSECTION - Drive in lane 2")
        time.sleep(50)
        
        # Instruction at red light (50s)
        self.display_message("Stay in lane 2 and proceed straight. Watch for pedestrians.", duration=4)
        
        # Wait for first green cycle
        print(" At red light - lead vehicles are stopped ahead")
        
        # First green occurs naturally from traffic light cycle
        # Monitor for green and track timing
        light_state = carla.TrafficLightState.Red
        first_green_recorded = False
        
        while self.current_time < self.scenario_duration:
            self.update_timers()
            
            # Track traffic light for first green
            if self.intersection_traffic_light:
                current_state = self.intersection_traffic_light.get_state()
                
                # First green detection
                if not first_green_recorded and current_state == carla.TrafficLightState.Green:
                    self.first_green_start_time = self.current_time
                    first_green_recorded = True
                    print(f"\n FIRST GREEN at {self.current_time:.1f}s")
                    print("But lead vehicle 1 is NOT moving...")
                    
                # Second green detection
                if self.first_green_start_time and not self.second_green_start_time:
                    if current_state == carla.TrafficLightState.Green:
                        # Check if this is a new green cycle (not first)
                        if self.current_time - self.first_green_start_time > self.signal_cycle['green'] + self.signal_cycle['red']:
                            self.second_green_start_time = self.current_time
                            print(f"\n SECOND GREEN at {self.current_time:.1f}s")
                            print("Vehicle 2 moves forward")
                            
                            # Vehicle 2 moves during second green
                            if len(self.lead_vehicles) > 1:
                                self.lead_vehicles[1].actor.set_autopilot(True)
                            
            # Check if lead vehicle 1 moves at yellow
            if self.intersection_traffic_light:
                if self.intersection_traffic_light.get_state() == carla.TrafficLightState.Yellow:
                    if len(self.lead_vehicles) > 0 and self.lead_vehicles[0].behavior_type == "delayed_start":
                        self.lead_vehicles[0].actor.set_autopilot(True)
                        self.lead_vehicles[0].behavior_type = "moved"
                        print("\n Lead vehicle FINALLY moves - at yellow light!")
                        
                        # Prompt initial delay rating at ~180s
                        if self.current_time > 170 and self.current_time < 190:
                            self.prompt_rating("delay", 180)
            
            # Check horn conditions
            self.check_and_prompt_horns()
            
            # Check if we've completed all events
            if self.current_time > 500:
                break
                
            time.sleep(0.1)  # 10Hz update
            
    def final_events(self):
        """Trigger final interference events"""
        # Jaywalking pedestrian
        self.spawn_jaywalking_pedestrian(timing=450)  # 450 seconds into scenario
        
        # Illegal left turn vehicle
        self.spawn_illegal_left_turn_vehicle(timing=458)  # 2-3 seconds before final rating
        
        # Schedule final rating prompt at 460s
        threading.Timer(460, lambda: self.prompt_rating("violations", 460)).start()
        
        print("\n Final events: Jaywalking pedestrian + Illegal left-turn vehicle approaching!")
        
    def cleanup(self):
        """Destroy all actors"""
        print("\n Cleaning up scenario...")
        
        # Destroy ego
        if self.ego_vehicle:
            self.ego_vehicle.destroy()
            
        # Destroy lead vehicles
        for lv in self.lead_vehicles:
            if lv.actor:
                lv.actor.destroy()
                
        # Destroy vehicles behind
        for vehicle in self.vehicles_behind:
            vehicle.destroy()
            
        # Destroy pedestrians
        for ped, controller in self.pedestrians:
            if controller:
                controller.stop()
                controller.destroy()
            if ped:
                ped.destroy()
                
        # Destroy illegal turn vehicle
        if self.illegal_turn_vehicle:
            self.illegal_turn_vehicle.destroy()
            
        print("✓ Cleanup complete")
        
    def ego_at_intersection(self) -> bool:
        """Check if ego vehicle is at the intersection"""
        loc = self.ego_vehicle.get_location()
        # Define intersection bounds (adjust based on your map)
        return (abs(loc.x + 50) < 30 and abs(loc.y - 150) < 30)
        
    def run(self):
        """Main execution"""
        print("\n" + "="*70)
        print("SCENARIO 3: Signal Delay + Lane Blocking + Pedestrian + Illegal Turn")
        print("="*70)
        print("\n Urban road with 2 lanes (Lane1: straight, Lane2: straight/right)")
        print(f" Traffic light cycle: Red(60s) → Green(55s) → Yellow(5s)")
        print(f" Speed limit: {self.speed_limit} km/h")
        print("\n Important: Stay in LANE 2 the entire time!")
        print("\nPress Enter when ready to begin...")
        input()
        
        # Setup environment
        self.setup_urban_road()
        
        # Find spawn point in lane 2
        spawn_points = self.world.get_map().get_spawn_points()
        # Select appropriate spawn point (adjust index based on Town05)
        spawn_point = spawn_points[15]  # Near intersection start
        spawn_point.location.y += 1.5  # Adjust to lane 2
        
        # Spawn ego vehicle
        ego_bp = self.blueprint_library.filter('vehicle.audi.tt')[0]
        self.ego_vehicle = self.world.spawn_actor(ego_bp, spawn_point)
        
        # Spawn other vehicles
        self.spawn_lead_vehicles()
        self.spawn_traffic_behind()
        
        # Start timing
        self.scenario_start_time = time.time()
        
        # Play beep for sync
        print("\n🔔 BEEP - Scenario starting")
        
        # Start monitoring threads
        speed_thread = threading.Thread(target=self.monitor_speed, daemon=True)
        speed_thread.start()
        
        # Run main timeline
        try:
            self.run_timeline()
            self.final_events()
            
            # Wait for final events to complete
            time.sleep(30)
            
        except KeyboardInterrupt:
            print("\n\n Scenario interrupted by user")
        except Exception as e:
            print(f"\n Error in scenario: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
            
        print("\n Scenario 3 Complete!")
        print("Thank you for participating")

# Standalone runner
if __name__ == "__main__":
    # Test connection
    try:
        client = carla.Client('localhost', 2000)
        client.get_server_version()
        print(f"✓ Connected to CARLA server")
    except:
        print(" CARLA server not running. Start CARLA first:")
        print("  ./CarlaUE4.sh -quality-level=Low")
        exit(1)
        
    scenario = Scenario3_SignalDelay()
    scenario.run()