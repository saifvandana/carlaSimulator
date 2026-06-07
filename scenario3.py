"""
Phase 7 - Illegal Turn Vehicle (Town05)
=========================================
Builds on Phase 6. Adds a vehicle coming from the east (westbound)
that makes an illegal left turn at the junction, cutting across
and blocking the ego's path.

Illegal turn vehicle:
  Spawn : idx=93  (1.0, -91.4)  road=6 lane=-1 yaw=-179.9 (heading west)
  Behaviour:
    - Spawns and drives west toward junction
    - At junction (x < -35) makes illegal left turn heading north
    - Stops blocking the ego's lane

Trigger: fired at GREEN 3 (same time as Q3 departs and pedestrian crosses)
         so ego faces multiple simultaneous obstructions.

Usage:
    python phase7_illegal_turn.py
    (horn.wav must be in same directory)
"""

from typing import Optional

import carla
import time
import math
import os
import random
import threading

# ── Config ────────────────────────────────────────────────────────────────────

HOST     = "127.0.0.1"
PORT     = 2000
MAP_NAME = "Town05"

EGO_BP        = "vehicle.lincoln.mkz_2017"
EGO_SPAWN_IDX = 1

QUEUE_BPS = [
    "vehicle.tesla.model3",
    "vehicle.audi.tt",
    "vehicle.mercedes.coupe",
]

QUEUE_POSITIONS = [
    carla.Location(x=-66.0, y=-84.5, z=0.5),
    carla.Location(x=-74.0, y=-84.5, z=0.5),
    carla.Location(x=-82.0, y=-84.5, z=0.5),
]

CRUISE_SPEED_KMH = 30.0

# Illegal turn vehicles — multiple, staggered, parked off-road until GREEN 3
# Each entry: (spawn_idx, yaw, activation_delay_s_after_green3, turn_x, turn_target, stop_y)
ILLEGAL_CONFIGS = [
    {
        "bp"      : "vehicle.seat.leon",
        "idx"     : 93,          # (1.0, -91.4) westbound lane=-1
        "yaw"     : -179.9,
        "delay"   : 0.0,         # activates immediately on GREEN 3
        "turn_x"  : -49.0,
        "turn_tgt": carla.Location(x=-55.0, y=-78.0, z=0.5),
        "stop_y"  : -86.5,
    },
    {
        "bp"      : "vehicle.audi.a2",
        "idx"     : 89,         # (-18.8, -91.4)  westbound lane=-1
        "yaw"     : -179.9,
        "delay"   : 0.2,         # activates 8s after GREEN 3
        "turn_x"  : -49.0,
        "turn_tgt": carla.Location(x=-55.0, y=-78.0, z=0.5),
        "stop_y"  : -87.5,
    },
]

TL_POSITIONS    = [(-39.6, -78.1), (21.2, -78.8)]
TL_MATCH_RADIUS = 2.0

APPROACH_TL_POSITIONS = [
    (-140.4, -78.1),   # first junction
    (-113.1, -78.9),   # second junction
    ( -61.1, -78.1),   # third junction
]
APPROACH_TL_RADIUS = 2.0

RED_DURATION    = 60.0
YELLOW_DURATION =  5.0
GREEN_DURATION  = 55.0
CYCLE_TOTAL     = RED_DURATION + YELLOW_DURATION + GREEN_DURATION

NAV_REFRESH_S = 3.0
SPAWN_LOC     = carla.Location(x=-44.2, y=-39.6, z=0.5)
JUNCTION_LOC  = carla.Location(x=-44.2, y=-79.0, z=0.5)
STOPLINE_LOC  = carla.Location(x=-39.6, y=-84.4, z=0.5)
LABEL_LOC     = carla.Location(x=-39.0, y=-84.0, z=6.0)
INSTRUCTION_LOC = carla.Location(x=-39.0, y=-84.0, z=10.0)

# HORN_WAV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "horn.wav")
HORN_WAV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "preview.mp3")


HORN_WINDOWS = [
    (67.0,  120.0, 20.0),
    (190.0, 240.0,  10.0),
]

PED_TRIGGER_T  = 300.0
PED_SPAWN_LOC  = carla.Location(x=-64.0, y=-80.0, z=1.0)
PED_DEST_LOC   = carla.Location(x=-64.0, y=-89.0, z=0.97)
PED_WALK_SPEED = 1.2

Q1_SPOT = carla.Location(x=-66.0, y=-84.5, z=0.5)
Q2_SPOT = carla.Location(x=-74.0, y=-84.5, z=0.5)


# ── Audio ─────────────────────────────────────────────────────────────────────

class HornPlayer:
    def __init__(self, wav_path):
        self._sound = None
        try:
            import pygame
            pygame.mixer.init(frequency=44100, size=-16, channels=1, buffer=512)
            self._sound = pygame.mixer.Sound(wav_path)
            print(f"[Horn] Loaded {wav_path}")
        except Exception as e:
            print(f"[Horn] WARNING: {e}")

    def play(self):
        if self._sound is None:
            return
        threading.Thread(target=self._sound.play, daemon=True).start()


class HornScheduler:
    def __init__(self, player):
        self.player     = player
        self._last_honk = -999.0

    def update(self, t):
        for (start, end, interval) in HORN_WINDOWS:
            if start <= t <= end:
                if t - self._last_honk >= interval:
                    self.player.play()
                    self._last_honk = t
                    print(f"[Horn] HONK at t={t:.1f}s")
                break


# ── Illegal turn vehicle ──────────────────────────────────────────────────────

class IllegalTurnVehicle:
    """
    Parked with hand brake until GREEN 3 + individual delay.
    Then drives west, turns left at junction, blocks 3s, clears.
    """

    PHASE_IDLE     = "idle"
    PHASE_APPROACH = "approach"
    PHASE_TURNING  = "turning"
    PHASE_BLOCKING = "blocking"
    PHASE_CLEARING = "clearing"
    BLOCK_DURATION = 1.0

    def __init__(self, vehicle, world_map, cfg):
        self.vehicle      = vehicle
        self.world_map    = world_map
        self.cfg          = cfg
        self.phase        = self.PHASE_IDLE
        self._block_start = None
        self._green3_t    = None

    def activate(self, t):
        self._green3_t = t
        print(f"[IllegalTurn] {self.cfg['bp']} armed — "
              f"departs in {self.cfg['delay']:.0f}s")

    def _steer_toward(self, target_loc, speed_kmh=20.0):
        loc    = self.vehicle.get_location()
        tf     = self.vehicle.get_transform()
        fwd    = tf.get_forward_vector()
        to_tgt = target_loc - loc
        cross  = fwd.x * to_tgt.y - fwd.y * to_tgt.x
        steer  = max(-1.0, min(1.0, cross * 0.5))
        v      = self.vehicle.get_velocity()
        speed  = math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6
        deficit  = speed_kmh - speed
        throttle = max(0.0, min(1.0, deficit * 0.05 + 0.25))
        brake    = 1.0 if deficit < -5.0 else 0.0
        return carla.VehicleControl(
            throttle=throttle, steer=steer, brake=brake
        )

    def run_step(self, t):
        loc = self.vehicle.get_location()

        if self.phase == self.PHASE_IDLE:
            # Stay parked with hand brake
            self.vehicle.apply_control(
                carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
            )
            # Check if delay has passed since GREEN 3 activation
            if self._green3_t is not None and \
                    t - self._green3_t >= self.cfg["delay"]:
                self.phase = self.PHASE_APPROACH
                self.vehicle.apply_control(
                    carla.VehicleControl(hand_brake=False)
                )
                print(f"[IllegalTurn] {self.cfg['bp']} departing at t={t:.1f}s")

        elif self.phase == self.PHASE_APPROACH:
            if loc.x <= self.cfg["turn_x"]:
                self.phase = self.PHASE_TURNING
                print(f"[IllegalTurn] {self.cfg['bp']} turning at "
                      f"({loc.x:.1f}, {loc.y:.1f})")
            else:
                wp = self.world_map.get_waypoint(loc, project_to_road=True)
                if wp:
                    nexts = wp.next(8.0)
                    if nexts:
                        self.vehicle.apply_control(
                            self._steer_toward(nexts[0].transform.location)
                        )

        elif self.phase == self.PHASE_TURNING:
            if loc.y >= self.cfg["stop_y"]:
                self.phase        = self.PHASE_BLOCKING
                self._block_start = time.time()
                print(f"[IllegalTurn] {self.cfg['bp']} blocking at "
                      f"({loc.x:.1f}, {loc.y:.1f})")
            else:
                self.vehicle.apply_control(
                    self._steer_toward(self.cfg["turn_tgt"], speed_kmh=20.0)
                )

        elif self.phase == self.PHASE_BLOCKING:
            self.vehicle.apply_control(
                carla.VehicleControl(throttle=0.0, brake=1.0)
            )
            if self._block_start and \
                    time.time() - self._block_start >= self.BLOCK_DURATION:
                self.phase = self.PHASE_CLEARING
                print(f"[IllegalTurn] {self.cfg['bp']} clearing")

        elif self.phase == self.PHASE_CLEARING:
            wp = self.world_map.get_waypoint(loc, project_to_road=True)
            if wp:
                nexts = wp.next(8.0)
                if nexts:
                    self.vehicle.apply_control(
                        self._steer_toward(
                            nexts[0].transform.location, speed_kmh=20.0
                        )
                    )


# ── Pedestrian ────────────────────────────────────────────────────────────────

class PedestrianAgent:
    def __init__(self, world, client):
        self.world      = world
        self.client     = client
        self.walker     = None
        self.controller = None
        self._activated = False
        self._spawned   = False

    def spawn(self):
        bpl        = self.world.get_blueprint_library()
        walker_bp  = random.choice(list(bpl.filter("walker.pedestrian.*")))
        spawn_tf   = carla.Transform(
            carla.Location(x=PED_SPAWN_LOC.x, y=PED_SPAWN_LOC.y, z=1.0),
            carla.Rotation(yaw=90.0)
        )
        batch   = [carla.command.SpawnActor(walker_bp, spawn_tf)]
        results = self.client.apply_batch_sync(batch, True)
        if not results or results[0].error:
            print(f"[Pedestrian] WARNING: Spawn failed.")
            return False
        self.walker = self.world.get_actor(results[0].actor_id)
        ctrl_bp     = bpl.find("controller.ai.walker")
        self.controller = self.world.spawn_actor(
            ctrl_bp, carla.Transform(), attach_to=self.walker
        )
        self.controller.start()
        self.controller.set_max_speed(0.0)
        print(f"[Pedestrian] Spawned at ({self.walker.get_location().x:.1f}, "
              f"{self.walker.get_location().y:.1f}) — waiting for t={PED_TRIGGER_T:.0f}s")
        self._spawned = True
        return True

    def update(self, t):
        if not self._spawned or self.walker is None or not self.walker.is_alive:
            return
        if not self._activated and t >= PED_TRIGGER_T:
            self._activated = True
            self.controller.set_max_speed(PED_WALK_SPEED)
            self.controller.go_to_location(PED_DEST_LOC)
            print(f"[Pedestrian] Walking at t={t:.1f}s")

    def destroy(self):
        if self.controller and self.controller.is_alive:
            self.controller.stop()
            self.controller.destroy()
        if self.walker and self.walker.is_alive:
            self.walker.destroy()


# ── Helpers ───────────────────────────────────────────────────────────────────

def get_speed_kmh(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6


def update_spectator(world, ego):
    tf  = ego.get_transform()
    fwd = tf.get_forward_vector()
    cam_loc = carla.Location(
        x=tf.location.x - fwd.x * 10.0,
        y=tf.location.y - fwd.y * 10.0,
        z=tf.location.z + 4.0
    )
    world.get_spectator().set_transform(carla.Transform(cam_loc, tf.rotation))


def spawn_ego(world, client):
    bpl = world.get_blueprint_library()
    bp  = bpl.find(EGO_BP)
    bp.set_attribute("role_name", "hero")

    # idx=107 (-162.5, -84.6) yaw=0.1 road=8 lane=2
    # heading east, same lane as queue vehicles — confirmed valid spawn
    sp    = world.get_map().get_spawn_points()[107]
    actor = world.try_spawn_actor(bp, sp)
    if actor is None:
        raise RuntimeError("Could not spawn ego at idx=107.")

    time.sleep(0.5)
    loc = actor.get_location()
    print(f"[Spawn] Ego at ({loc.x:.1f}, {loc.y:.1f}) heading east")
    return actor

def find_ego_vehicle(world: carla.libcarla.World) -> Optional[carla.libcarla.Vehicle]:
    DReyeVR_vehicle = None
    ego_vehicles = list(world.get_actors().filter("harplab.dreyevr_vehicle.*"))
    if len(ego_vehicles) >= 1:
        DReyeVR_vehicle = ego_vehicles[0]  # TODO: support for multiple ego vehicles?
        # print("ego vehicle= ", DReyeVR_vehicle)
    else:
        model: str = "harplab.dreyevr_vehicle.model3"
        print(f'No EgoVehicle found, spawning one: "{model}"')
        bp = world.get_blueprint_library().find(model)
        transform = world.get_map().get_spawn_points()[0]
        DReyeVR_vehicle = world.spawn_actor(bp, transform)
    return DReyeVR_vehicle

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
        vehicle.get_world().debug.draw_string(
                carla.Location(x=location.x, y=location.y, z=location.z + 3.0),
                "Speed limit: 50 km/h", draw_shadow=True,
                color=carla.Color(255, 50, 50), life_time=10.0
            )
        
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


def spawn_npc(world, client, bp_id, safe_idx, target_idx=None,
              target_loc=None, target_yaw=-179.9):
    """Generic safe-spawn + teleport for any NPC."""
    bpl          = world.get_blueprint_library()
    bp           = bpl.find(bp_id)
    spawn_points = world.get_map().get_spawn_points()

    actor = world.try_spawn_actor(bp, spawn_points[safe_idx])
    if actor is None:
        print(f"[Spawn] WARNING: safe spawn failed for {bp_id} at idx={safe_idx}")
        return None
    time.sleep(0.5)

    if target_idx is not None:
        tf = spawn_points[target_idx]
        tf.location.z += 0.5
    elif target_loc is not None:
        tf = carla.Transform(
            target_loc,
            carla.Rotation(yaw=target_yaw)
        )
    else:
        return actor

    actor.set_simulate_physics(False)
    time.sleep(0.2)
    actor.set_transform(tf)
    time.sleep(0.5)
    actor.set_simulate_physics(True)
    time.sleep(0.5)
    return actor


def spawn_queue_vehicles(world, client):
    bpl      = world.get_blueprint_library()
    yaw_east = carla.Rotation(pitch=0.0, yaw=0.1, roll=0.0)
    vehicles = []

    for i, (bp_id, target_loc) in enumerate(zip(QUEUE_BPS, QUEUE_POSITIONS)):
        bp = bpl.find(bp_id)
        target_tf = carla.Transform(target_loc, yaw_east)

        batch   = [carla.command.SpawnActor(bp, target_tf)]
        results = client.apply_batch_sync(batch, True)

        if not results or results[0].error:
            print(f"[Queue] WARNING: Q{i+1} spawn failed — "
                  f"{results[0].error if results else 'no result'}")
            vehicles.append(None)
            continue

        actor = world.get_actor(results[0].actor_id)
        actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        loc = actor.get_location()
        print(f"[Queue] Q{i+1} at ({loc.x:.2f}, {loc.y:.2f})")
        vehicles.append(actor)

    return vehicles


def find_traffic_lights_by_position(world, positions, radius):
    all_tls = world.get_actors().filter("traffic.traffic_light*")
    found   = []
    for (tx, ty) in positions:
        best_tl, best_dist = None, radius
        for tl in all_tls:
            loc  = tl.get_location()
            dist = math.sqrt((loc.x - tx)**2 + (loc.y - ty)**2)
            if dist < best_dist:
                best_dist = dist
                best_tl   = tl
        if best_tl:
            found.append(best_tl)
            print(f"[TLFinder] ({tx}, {ty}) -> id={best_tl.id}  "
                  f"dist={best_dist:.2f} m")
        else:
            print(f"[TLFinder] WARNING: No TL within {radius} m of ({tx}, {ty})")
    return found


# ── Waypoint driver ───────────────────────────────────────────────────────────

class WaypointDriver:
    LOOKAHEAD_M = 8.0
    KP_THROTTLE = 0.05
    MAX_STEER   = 1.0

    JUNCTION_ENTRY_X  = -58
    RIGHT_TURN_TARGET = carla.Location(x=-58.7, y=-42.2, z=0.0)
    RIGHT_TURN_DONE_Y = -98.0

    def __init__(self, vehicle, world_map, target_speed_kmh=30.0):
        self.vehicle      = vehicle
        self.world_map    = world_map
        self.target_speed = target_speed_kmh
        self._turning     = False
        self._turn_done   = False

    def _steer_toward(self, target_loc):
        loc    = self.vehicle.get_location()
        tf     = self.vehicle.get_transform()
        fwd    = tf.get_forward_vector()
        to_tgt = target_loc - loc
        cross  = fwd.x * to_tgt.y - fwd.y * to_tgt.x
        steer  = max(-self.MAX_STEER, min(self.MAX_STEER, cross * 0.4))
        speed  = get_speed_kmh(self.vehicle)
        deficit  = self.target_speed - speed
        throttle = max(0.0, min(1.0, deficit * self.KP_THROTTLE + 0.25))
        brake    = 1.0 if deficit < -5.0 else 0.0
        return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)

    def run_step(self):
        loc  = self.vehicle.get_location()
        wp   = self.world_map.get_waypoint(loc, project_to_road=True)
        if wp is None:
            return
        nexts = wp.next(self.LOOKAHEAD_M)
        if not nexts:
            return
        self.vehicle.apply_control(
            self._steer_toward(nexts[0].transform.location)
        )

    def run_step_right_turn(self):
        loc = self.vehicle.get_location()
        if not self._turning and not self._turn_done:
            if loc.x >= self.JUNCTION_ENTRY_X:
                self._turning = True
                print(f"[Q3] Entering junction — turning right")
            else:
                self.run_step()
                return
        if self._turning and not self._turn_done:
            if loc.y < self.RIGHT_TURN_DONE_Y:
                self._turning   = False
                self._turn_done = True
                print(f"[Q3] Right turn complete")
            else:
                self.vehicle.apply_control(
                    self._steer_toward(self.RIGHT_TURN_TARGET)
                )
                return
        if self._turn_done:
            self.run_step()



# ── Traffic light controller ──────────────────────────────────────────────────

PHASE_COLORS = {
    "RED":    carla.Color(255,  60,  60),
    "YELLOW": carla.Color(255, 220,   0),
    "GREEN":  carla.Color( 60, 220,  60),
}

class ApproachLightController:
    """
    Keeps the traffic lights on the ego approach road permanently GREEN
    so ego doesn't get stopped before reaching the scenario junction.
    Uses set_state() every tick — no freeze so other directions unaffected.
    """
 
    def __init__(self, world):
        self.lights = []
        all_tls = world.get_actors().filter("traffic.traffic_light*")
        for (tx, ty) in APPROACH_TL_POSITIONS:
            best_tl, best_dist = None, APPROACH_TL_RADIUS
            for tl in all_tls:
                loc  = tl.get_location()
                dist = math.sqrt((loc.x - tx)**2 + (loc.y - ty)**2)
                if dist < best_dist:
                    best_dist = dist
                    best_tl   = tl
            if best_tl:
                self.lights.append(best_tl)
                print(f"[ApproachTL] Found light at ({tx}, {ty}) "
                      f"-> id={best_tl.id}  keeping GREEN")
            else:
                print(f"[ApproachTL] WARNING: No light found at ({tx}, {ty})")
 
    def update(self):
        """Call every tick to keep approach lights green."""
        for tl in self.lights:
            tl.set_state(carla.TrafficLightState.Green)
 

class TrafficLightController:
    PHASE_RED    = "RED"
    PHASE_YELLOW = "YELLOW"
    PHASE_GREEN  = "GREEN"

    def __init__(self, lights):
        self.lights      = lights
        self.phase       = self.PHASE_GREEN
        self._last_phase = None
        for tl in self.lights:
            # tl.freeze(True)
            tl.set_state(carla.TrafficLightState.Red)
            print(f"[TLController] Controlling {len(self.lights)} lights — starting RED")
        # print(f"[TLController] {len(self.lights)} lights frozen — starting RED")

    def update(self, t):
        pos = t % CYCLE_TOTAL
        if pos < RED_DURATION:
            self.phase = self.PHASE_RED
            state      = carla.TrafficLightState.Red
        elif pos < RED_DURATION + YELLOW_DURATION:
            self.phase = self.PHASE_YELLOW
            state      = carla.TrafficLightState.Yellow
        else:
            self.phase = self.PHASE_GREEN
            state      = carla.TrafficLightState.Green

         # Always set state every tick — overrides CARLA's internal controller
        for tl in self.lights:
            tl.set_state(state)

        if self.phase != self._last_phase:
            print(f"[TLController] t={t:.1f}s  ->  {self.phase}")
            self._last_phase = self.phase

    def time_remaining(self, t):
        pos = t % CYCLE_TOTAL
        if pos < RED_DURATION:
            return RED_DURATION - pos
        elif pos < RED_DURATION + YELLOW_DURATION:
            return (RED_DURATION + YELLOW_DURATION) - pos
        else:
            return CYCLE_TOTAL - pos

    def unfreeze(self):
        # Nothing to unfreeze — just let CARLA's controller resume naturally
        print("[TLController] Released — CARLA controller resumes.")


# ── Queue controller ──────────────────────────────────────────────────────────

class QueueController:
    def __init__(self, vehicles, world_map):
        self.vehicles  = vehicles
        self.world_map = world_map
        self.drivers   = [
            WaypointDriver(v, world_map, CRUISE_SPEED_KMH) if v else None
            for v in vehicles
        ]
        self._yellow_count   = 0
        self._green_count    = 0
        self._prev_phase     = None
        self._q1_departed    = False
        self._q2_departed    = False
        self._q3_departed    = False
        self._q2_creep1_done = False
        self._q3_creep1_done = False
        self._q3_creep2_done = False
 
    def _hold(self, vehicle):
        if vehicle and vehicle.is_alive:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
 
    def _set_indicator(self, vehicle, direction):
        if vehicle is None or not vehicle.is_alive:
            return
        if direction == "right":
            state = carla.VehicleLightState.RightBlinker
        elif direction == "left":
            state = carla.VehicleLightState.LeftBlinker
        else:
            state = carla.VehicleLightState.NONE
        vehicle.set_light_state(carla.VehicleLightState(state))
 
    def _flash_indicator(self, vehicle, direction, t):
        if int(t * 2) % 2 == 0:
            self._set_indicator(vehicle, direction)
        else:
            self._set_indicator(vehicle, None)
 
    def _creep_toward(self, vehicle, target, threshold=1.5):
        if vehicle is None or not vehicle.is_alive:
            return True
        loc  = vehicle.get_location()
        dist = math.sqrt((loc.x - target.x)**2 + (loc.y - target.y)**2)
        if dist < threshold:
            self._hold(vehicle)
            return True
        tf     = vehicle.get_transform()
        fwd    = tf.get_forward_vector()
        to_tgt = target - loc
        cross  = fwd.x * to_tgt.y - fwd.y * to_tgt.x
        steer  = max(-1.0, min(1.0, cross * 0.4))
        vehicle.apply_control(carla.VehicleControl(
            throttle=0.35, steer=steer, brake=0.0
        ))
        return False
 
    def update(self, phase, t):
        if phase == "YELLOW" and self._prev_phase != "YELLOW":
            self._yellow_count += 1
            print(f"[Queue] YELLOW {self._yellow_count} at t={t:.1f}s")
        if phase == "GREEN" and self._prev_phase != "GREEN":
            self._green_count += 1
            print(f"[Queue] GREEN {self._green_count} at t={t:.1f}s")
        self._prev_phase = phase

        if not self._q3_departed:
            self._flash_indicator(self.vehicles[2], "right", t)

        if self._yellow_count == 1:# and phase == "YELLOW":
            if not self._q1_departed:
                self._q1_departed = True
                self._set_indicator(self.vehicles[0], None)
                print(f"[Queue] Q1 departing on YELLOW 1")
            if not self._q2_creep1_done:
                self._q2_creep1_done = self._creep_toward(self.vehicles[1], Q1_SPOT)
            if not self._q3_creep1_done:
                self._q3_creep1_done = self._creep_toward(self.vehicles[2], Q2_SPOT)

        elif self._green_count == 1 and phase == "GREEN":
            self._hold(self.vehicles[1])
            self._hold(self.vehicles[2])

        elif self._yellow_count == 2:# and phase == "YELLOW":
            if not self._q2_departed:
                self._q2_departed = True
                self._set_indicator(self.vehicles[1], None)
                print(f"[Queue] Q2 departing on YELLOW 2")
            if not self._q3_creep2_done:
                self._q3_creep2_done = self._creep_toward(self.vehicles[2], Q1_SPOT)

        elif self._green_count == 2 and phase == "GREEN":
            self._hold(self.vehicles[2])

        elif self._green_count >= 3 and not self._q3_departed:
            self._q3_departed = True
            self._set_indicator(self.vehicles[2], "right")
            print(f"[Queue] Q3 departing on GREEN 3 for right turn")

        elif phase == "RED":
            if not self._q2_departed:
                self._hold(self.vehicles[1])
            if not self._q3_departed:
                self._hold(self.vehicles[2])

        if self._q1_departed and self.vehicles[0] and self.vehicles[0].is_alive:
            self.drivers[0].run_step()
        if self._q2_departed and self.vehicles[1] and self.vehicles[1].is_alive:
            self.drivers[1].run_step()
        if self._q3_departed and self.vehicles[2] and self.vehicles[2].is_alive:
            self.drivers[2].run_step_right_turn()


def draw_phase_label(world, phase, remaining):
    world.debug.draw_string(
        LABEL_LOC, f"  {phase}  {remaining:.0f}s",
        draw_shadow=True,
        color=PHASE_COLORS.get(phase, carla.Color(255, 255, 255)),
        life_time=1.2
    )

import pygame
import numpy as np
 
 
# ── Settings ──────────────────────────────────────────────────────────────────
WIDTH, HEIGHT   = 1280, 720
GREEN           = (0, 255, 70)
WHITE           = (255, 255, 255)
SHADOW          = (0, 60, 0)
MSG_SHADOW      = (60, 60, 60)
FONT_SIZE       = 300
MSG_FONT_SIZE   = 45
 
COUNTDOWN_STEPS = ["3", "2", "1", "GO!"]
STEP_DURATION   = 1.5
GO_DURATION     = 1.5
MSG_DURATION    = 3.0       # seconds each message stays on screen
 
# (trigger_time_seconds, message_text)
TIMED_MESSAGES = [
    (0.2,  "The speed limit is 50 km/h. Stay in lane 2"),
    (50,  "Stay in lane 2 and proceed straight. Watch for pedestrians."),
]
 
SCENARIO_DURATION = 100     # total seconds ego vehicle drives
 
# ── Camera ────────────────────────────────────────────────────────────────────
camera_surface = None
 
def camera_callback(image):
    global camera_surface
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    array = array[:, :, :3][:, :, ::-1]          # BGRA → RGB
    camera_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
 
# ── Pygame helpers ────────────────────────────────────────────────────────────
# Colour key used as the transparent background (must not appear in text)
TRANSPARENT_COLOR = (1, 1, 1)
 
def get_carla_window_position():
    """
    Find the CARLA/UE4 window and return its (x, y) top-left position
    so Pygame can be placed on the same screen.
    Returns (0, 0) if the window cannot be found.
    """
    try:
        import ctypes
        import ctypes.wintypes
 
        found = []
 
        def callback(hwnd, _):
            if ctypes.windll.user32.IsWindowVisible(hwnd):
                buf = ctypes.create_unicode_buffer(256)
                ctypes.windll.user32.GetWindowTextW(hwnd, buf, 256)
                title = buf.value.lower()
                if "CarlaUE4" in title or "carla" in title or "unreal" in title:
                    rect = ctypes.wintypes.RECT()
                    ctypes.windll.user32.GetWindowRect(hwnd, ctypes.byref(rect))
                    found.append((rect.left, rect.top))
            return True
 
        WNDENUMPROC = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_int, ctypes.c_int)
        ctypes.windll.user32.EnumWindows(WNDENUMPROC(callback), 0)
 
        if found:
            print(f"[INFO] CARLA window found at {found[0]}")
            return found[0]
    except Exception as e:
        print(f"[WARN] Could not detect CARLA window position: {e}")
 
    return (0, 0)


def open_pygame_window():
    """
    Open a borderless, transparent-background Pygame window.
    The TRANSPARENT_COLOR is set as the window colour key so every pixel
    of that colour becomes see-through, leaving only the text visible.
    """
    # Centre the Pygame window on the same screen as CARLA.
    # We temporarily init the display just to read the desktop size,
    # then use that to compute the centred position before the real init.
    carla_x, carla_y = -8, -20#get_carla_window_position()
    try:
        if not pygame.display.get_init():
            pygame.display.init()
        screen_w, screen_h = pygame.display.get_desktop_sizes()[0]
    except Exception:
        screen_w, screen_h = 1920, 1080   # safe fallback
    centre_x = carla_x + (screen_w  // 2) - (WIDTH  // 2)
    centre_y = carla_y + (screen_h // 2) - (HEIGHT // 2)
    os.environ["SDL_VIDEO_WINDOW_POS"] = f"{centre_x},{centre_y}"
 
    # Only init the display subsystem — never the mixer.
    # Calling pygame.init() would reset the mixer and kill any
    # audio (e.g. horn sounds) already running in another module.
    if not pygame.get_init():
        pygame.display.init()
        pygame.font.init()
    screen = pygame.display.set_mode(
        (WIDTH, HEIGHT),
        pygame.NOFRAME,          # no title bar / border
    )
    pygame.display.set_caption("CARLA Scenario")
 
    # Make TRANSPARENT_COLOR invisible at the OS level (Windows + most Linux)
    hwnd_set = False
    try:
        import ctypes
        hwnd = pygame.display.get_wm_info()["window"]
        # WS_EX_LAYERED = 0x80000, LWA_COLORKEY = 0x1
        ctypes.windll.user32.SetWindowLongW(hwnd, -20,
            ctypes.windll.user32.GetWindowLongW(hwnd, -20) | 0x80000)
        ctypes.windll.user32.SetLayeredWindowAttributes(
            hwnd, RGB(*TRANSPARENT_COLOR), 0, 0x1)
        hwnd_set = True
    except Exception:
        pass   # non-Windows: colour key won't be OS-transparent but text still shows
 
    # Always set Pygame's own colour key so blit compositing is correct
    screen.set_colorkey(TRANSPARENT_COLOR)
 
    clock    = pygame.time.Clock()
    font_big = pygame.font.SysFont("Arial", FONT_SIZE,     bold=True)
    font_msg = pygame.font.SysFont("Arial", MSG_FONT_SIZE, bold=True)
    return screen, clock, font_big, font_msg

 
def RGB(r, g, b):
    """Pack r,g,b into a single COLORREF int for Win32."""
    return r | (g << 8) | (b << 16)
 
 
def draw_text_centred(screen, font, text, color, shadow_color, offset=(8, 8), padding_top=40):
    """Draw text horizontally centred at the top of the screen."""
    shadow_surf = font.render(text, True, shadow_color)
    text_surf   = font.render(text, True, color)
    cx = WIDTH // 2 - text_surf.get_width() // 2
    cy = padding_top
    screen.blit(shadow_surf, (cx + offset[0], cy + offset[1]))
    screen.blit(text_surf,   (cx, cy))
 
 
def draw_background(screen):
    """Fill with the transparent colour key — no camera feed, no overlay."""
    screen.fill(TRANSPARENT_COLOR)
 
 
def close_pygame():
    # Only quit the display subsystem, not the mixer.
    # pygame.quit() would shut down audio and break horn sounds.
    if pygame.display.get_init():
        pygame.display.quit()
 
 
# ── Countdown ─────────────────────────────────────────────────────────────────
def show_countdown(world):
    """Display 3-2-1-GO! in a Pygame window. Closes the window after GO!"""
    screen, clock, font_big, _ = open_pygame_window()
    durations = [STEP_DURATION] * 3 + [GO_DURATION]
 
    for label, duration in zip(COUNTDOWN_STEPS, durations):
        t_start = time.time()
        while time.time() - t_start < duration:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    close_pygame()
                    return
            draw_background(screen)
            draw_text_centred(screen, font_big, label, GREEN, SHADOW)
            pygame.display.flip()
            clock.tick(60)
            world.tick()
        print(f"[countdown] {label}")
 
    close_pygame()

# ── Timed message ─────────────────────────────────────────────────────────────
def show_message(world, clock_ref, message):
    """
    Open a Pygame window, display `message` for MSG_DURATION seconds,
    then close it. Keeps ticking the CARLA world while open.
    """
    screen, clock, _, font_msg = open_pygame_window()
    t_start = time.time()
 
    while time.time() - t_start < MSG_DURATION:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
        draw_background(screen)
        draw_text_centred(screen, font_msg, message, WHITE, MSG_SHADOW)
        pygame.display.flip()
        clock.tick(60)
        world.tick()
 
    close_pygame()
    print(f"[message] '{message}' closed")

# ── Spectator ─────────────────────────────────────────────────────────────────
def set_spectator_behind_vehicle(world, vehicle, distance=10.0, height=4.0):
    t   = vehicle.get_transform()
    fwd = t.get_forward_vector()
    loc = t.location
    world.get_spectator().set_transform(carla.Transform(
        carla.Location(
            x=loc.x - fwd.x * distance,
            y=loc.y - fwd.y * distance,
            z=loc.z + height,
        ),
        carla.Rotation(pitch=-15, yaw=t.rotation.yaw, roll=0),
    ))
 
# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    # pygame.init()
    # screen = pygame.display.set_mode((WIDTH, HEIGHT))
    # pygame.display.set_caption("CARLA Scenario")
    # clock  = pygame.time.Clock()
    # font   = pygame.font.SysFont("Arial", FONT_SIZE, bold=True)

    print("[Phase7] Connecting to CARLA ...")
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)

    world = client.load_world(MAP_NAME)
    world_map = world.get_map()

    # ego = spawn_ego(world, client)
    # ego.set_autopilot(False)

    ego = find_ego_vehicle(world)
    success = precise_respawn(ego, carla.Location(x=-182, y=-84.5, z=0.5), yaw=None)


    print("[Phase7] Spawning queue vehicles ...")
    queue_vehicles = spawn_queue_vehicles(world, client)
    spawned = sum(1 for v in queue_vehicles if v is not None)
    print(f"[Phase7] {spawned}/3 queue vehicles spawned.")

    print("[Phase7] Spawning illegal turn vehicles ...")
    bpl             = world.get_blueprint_library()
    illegal_agents  = []
    illegal_actors  = []
    spawn_points    = world.get_map().get_spawn_points()

    for cfg in ILLEGAL_CONFIGS:
        sp = spawn_points[cfg["idx"]]
        tf = carla.Transform(
            carla.Location(x=sp.location.x, y=sp.location.y, z=sp.location.z + 0.5),
            carla.Rotation(yaw=cfg["yaw"])
        )
        batch   = [carla.command.SpawnActor(bpl.find(cfg["bp"]), tf)]
        results = client.apply_batch_sync(batch, True)
        if results and not results[0].error:
            actor = world.get_actor(results[0].actor_id)
            # Park with hand brake — stays still until GREEN 3
            actor.apply_control(
                carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
            )
            agent = IllegalTurnVehicle(actor, world_map, cfg)
            illegal_actors.append(actor)
            illegal_agents.append(agent)
            print(f"[Phase7] Illegal vehicle '{cfg['bp']}' parked at "
                  f"({actor.get_location().x:.1f}, {actor.get_location().y:.1f})")
        else:
            err = results[0].error if results else "no result"
            print(f"[Phase7] WARNING: Illegal vehicle spawn failed — {err}")

    print("[Phase7] Spawning pedestrians ...")
    from pedestrian_manager import PedestrianManager
    ped_manager = PedestrianManager(world, client)
    ped_manager.spawn_all()

    print("[Phase7] Populating world with background traffic ...")
    from world_population import WorldPopulator
    world_pop = WorldPopulator(world, client)
    world_pop.populate()

    print("[Phase7] Locating traffic lights ...")
    lights = find_traffic_lights_by_position(world, TL_POSITIONS, TL_MATCH_RADIUS)
    if not lights:
        print("[Phase7] ERROR: No traffic lights found. Aborting.")
        ego.destroy()
        return

    horn_player    = HornPlayer(HORN_WAV)
    horn_scheduler = HornScheduler(horn_player)
    tl_ctrl        = TrafficLightController(lights)
    # Keep approach lights green so ego isn't stopped before scenario junction
    approach_tl = ApproachLightController(world)
    queue_ctrl     = QueueController(queue_vehicles, world_map)

       # HUD
    from hud import HUD
    hud = HUD(world)
    # hud.start_tts()

    # green_count_last = 0   # track when GREEN 3 fires to activate illegal turn

    print("""
[Phase7] Ready. Full scenario sequence:
  RED 1    : all hold, Q3 right indicator
  YELLOW 1 : Q1 departs, Q2+Q3 creep forward
  GREEN 1  : all hold, horn starts
  YELLOW 2 : Q2 departs, Q3 creeps forward
  GREEN 2  : Q3 holds, ego has room, horn continues
  GREEN 3  : Q3 departs right turn
             Illegal turn vehicle activates
             Pedestrian crosses at t=300s

  Ctrl+C to stop.
""")
    print(f"{'t (s)':>8}  {'Phase':>8}  {'Remaining':>10}  {'Speed':>8}")
    print("-" * 44)
    last_nav_draw = 0.0
    green_count   = 0

    try:
        set_spectator_behind_vehicle(world, ego)
        world.tick()
        time.sleep(0.5)
 
        # ── Countdown ─────────────────────────────────────────────────────
        print("[Phase7] Starting countdown ...")
        show_countdown(world)
        # hud.countdown(ego)

        t_start       = time.time()
        last_print    = -1.0
        triggered       = set()   # track which messages have been shown

        while True:
            t = time.time() - t_start

            tl_ctrl.update(t)
            approach_tl.update()
            queue_ctrl.update(tl_ctrl.phase, t)
            horn_scheduler.update(t)
            ped_manager.update(t)

            # Activate illegal turn vehicles on YELLOW 3
            if tl_ctrl.phase == "YELLOW" and queue_ctrl._yellow_count >= 3:
                for agent in illegal_agents:
                    if agent._green3_t is None:
                        agent.activate(t)

            for agent in illegal_agents:
                agent.run_step(t)

            # Check each timed message
            for trigger_sec, message in TIMED_MESSAGES:
                if trigger_sec not in triggered and t >= trigger_sec:
                    triggered.add(trigger_sec)
                    print(f"[INFO] Showing message at {trigger_sec}s: '{message}'")
                    show_message(world, None, message)
                    # Reset scenario clock offset so loop continues cleanly
                    break

            hud.update(t, ego, tl_ctrl.phase, tl_ctrl.time_remaining(t))
            update_spectator(world, ego)

            # Clean up any crashed background vehicles every 5s
            if int(t) % 5 == 0 and int(t) != int(last_print):
                world_pop.monitor()
            draw_phase_label(world, tl_ctrl.phase, tl_ctrl.time_remaining(t))

            if int(t) != int(last_print):
                spd = get_speed_kmh(ego)
                print(f"{t:>8.1f}  {tl_ctrl.phase:>8}  "
                      f"{tl_ctrl.time_remaining(t):>9.0f}s  "
                      f"{spd:>7.1f} km/h")
                last_print = t

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[Phase7] Stopped.")

    finally:
        world_pop.destroy_all()
        tl_ctrl.unfreeze()
        ego.destroy()
        for v in queue_vehicles:
            if v and v.is_alive:
                v.destroy()
        for v in illegal_actors:
            if v and v.is_alive:
                v.destroy()
        ped_manager.destroy_all()
        print("[Phase7] Done.")


if __name__ == "__main__":
    main()