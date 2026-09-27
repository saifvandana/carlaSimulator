"""
Scenario 1 — Full Script (FIXED: Cut-in vehicles join queue)
==============================================================
FIX: Congestion cut-in vehicles (C1, C2, C3) properly join the queue
     and follow it through recovery and exit phases.
"""

import carla, time, math, random
from enum import Enum

CARLA_HOST  = "localhost"
CARLA_PORT  = 2000
FDS         = 0.05
TM_PORT     = 8001
ROAD_LIMIT  = 120.0

SPEED_CRUISE    = 125.0
SPEED_CATCHUP   = 110.0
SPEED_SLOW      = 70.0

# Congestion speed profile
SPEED_CONG_MIN  = 5.0
SPEED_CONG_MAX  = 10.0
SPEED_RECOVERY_MIN = 10.0
SPEED_RECOVERY_MAX = 50.0

CUT_AHEAD_M     = 5.0
SPAWN_BEFORE_S  = 10.0
DESTROY_M       = 300.0
NUM_CONG_VEHS   = 8

SPEED_LOW  = 100.0; SPEED_HIGH = 105.0
WARN_LOW   = 90.0;  WARN_DELAY = 20.0

COLOR_OK     = carla.Color(0, 220, 0)
COLOR_WARN   = carla.Color(255, 180, 0)
COLOR_DANGER = carla.Color(255, 40, 40)
COLOR_INFO   = carla.Color(100, 200, 255)
COLOR_RATING = carla.Color(255, 255, 0)

class DecelType(Enum):
    RAPID   = 3.0
    GRADUAL = 4.5

# ── Utilities ─────────────────────────────────────────────────────────────────
def pct(kmh): return ((ROAD_LIMIT - kmh) / ROAD_LIMIT) * 100.0
def is_alive(a):
    try:    return a is not None and a.is_alive
    except: return False
def safe_destroy(a):
    try:
        if is_alive(a): a.set_autopilot(False); a.destroy()
    except: pass
def kmh_of(a):
    if not is_alive(a): return 0.0
    try:
        v = a.get_velocity()
        return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
    except: return 0.0
def dist_ahead(ego, npc):
    if not is_alive(ego) or not is_alive(npc): return 0.0
    try:
        tf  = ego.get_transform(); fwd = tf.get_forward_vector()
        d   = npc.get_transform().location - tf.location
        return d.x*fwd.x + d.y*fwd.y
    except: return 0.0

def get_ego_wp(world, ego):
    return world.get_map().get_waypoint(
        ego.get_transform().location, project_to_road=True,
        lane_type=carla.LaneType.Driving)

def wp_in_lane(wp, target_lane_id):
    cur = wp
    for _ in range(6):
        if cur is None: return None
        if cur.lane_id == target_lane_id: return cur
        if abs(target_lane_id) > abs(cur.lane_id):
            nxt = cur.get_right_lane()
        else:
            nxt = cur.get_left_lane()
        if nxt and nxt.lane_type == carla.LaneType.Driving:
            cur = nxt
        else:
            break
    return cur if cur and cur.lane_id == target_lane_id else None

def spawn_at_wp(world, wp, model=None):
    blib = world.get_blueprint_library()
    bp   = None
    if model:
        found = blib.filter(model)
        if found:
            bp = found[0]
        else:
            print(f"[Spawn] Model '{model}' not found — using fallback")
    if not bp:
        cars = [b for b in blib.filter("vehicle.*")
                if "dreyevr" not in b.id
                and int(b.get_attribute("number_of_wheels")) == 4]
        bp = random.choice(cars) if cars else blib.find("vehicle.tesla.model3")
    bp.set_attribute("role_name", "npc")
    tf = carla.Transform(
        carla.Location(x=wp.transform.location.x,
                       y=wp.transform.location.y,
                       z=wp.transform.location.z + 0.3),
        wp.transform.rotation)
    actor = world.try_spawn_actor(bp, tf)
    return actor

def spawn_behind_in_lane(world, ego, lane_id, behind_m, model=None):
    ego_wp = get_ego_wp(world, ego)
    if not ego_wp: return None
    ego_lane_wp = wp_in_lane(ego_wp, lane_id)
    if not ego_lane_wp: return None
    prev = ego_lane_wp.previous(behind_m)
    if not prev: return None
    spawn_wp = prev[0]
    spawn_wp = wp_in_lane(spawn_wp, lane_id) or spawn_wp
    npc = spawn_at_wp(world, spawn_wp, model)
    if npc:
        npc.set_autopilot(False)
        npc.set_simulate_physics(True)
    return npc

def find_gap_in_lane(world, reference_actor, lane_id, min_gap_m=1.0,
                     search_ahead_m=60.0):
    if not is_alive(reference_actor): return False, 0.0, 0.0
    try:
        ref_loc = reference_actor.get_transform().location
        ref_fwd = reference_actor.get_transform().get_forward_vector()
        actors  = world.get_actors().filter("vehicle.*")

        in_lane = []
        for a in actors:
            if not is_alive(a) or a.id == reference_actor.id: continue
            wp = world.get_map().get_waypoint(
                a.get_transform().location, project_to_road=True,
                lane_type=carla.LaneType.Driving)
            if not wp or wp.lane_id != lane_id: continue
            d = a.get_transform().location - ref_loc
            dist = d.x*ref_fwd.x + d.y*ref_fwd.y
            if -20.0 < dist < search_ahead_m:
                in_lane.append((dist, a))

        if not in_lane:
            return True, 999.0, 0.0

        in_lane.sort(key=lambda x: x[0])
        prev_dist = -5.0
        max_gap   = 0.0
        gap_start = 0.0
        for dist, _ in in_lane:
            gap = dist - prev_dist
            if gap > max_gap:
                max_gap   = gap
                gap_start = prev_dist
            prev_dist = dist

        return max_gap >= min_gap_m, max_gap, gap_start
    except Exception as e:
        print(f"[GapCheck] {e}")
        return False, 0.0, 0.0

# ── Background traffic ──────────────────────────────────────────────────────
class BackgroundController:
    def __init__(self, world, npc, target_lane=-1):
        self.world = world
        self.npc = npc
        self.target_lane = target_lane
        self.target_speed = random.uniform(95.0, 115.0)
        self.controller = NPCController(world, npc, target_lane)
        
    def update(self):
        if not is_alive(self.npc):
            return
        self.controller.tick(self.target_speed)

def spawn_background_custom(world, ego, num_vehicles=4):
    ego_wp = get_ego_wp(world, ego)
    if not ego_wp: return [], []

    lane1_wp = wp_in_lane(ego_wp, -1)
    if not lane1_wp:
        print("[BGTraffic] Could not find lane -1")
        return [], []

    spawned = []
    controllers = []
    blib = world.get_blueprint_library()

    for i in range(num_vehicles):
        offset_m = (i - num_vehicles // 2) * 120.0
        
        if offset_m >= 0:
            wps = lane1_wp.next(offset_m + 40.0)
        else:
            wps = lane1_wp.previous(abs(offset_m))
            
        if not wps: continue
        wp = wps[0]
        wp = wp_in_lane(wp, -1) or wp

        model = random.choice([
            "vehicle.audi.tt", "vehicle.chevrolet.impala",
            "vehicle.dodge.charger_2020", "vehicle.ford.mustang",
            "vehicle.lincoln.mkz_2017", "vehicle.toyota.prius"
        ])
        cars = blib.filter(model)
        bp = cars[0] if cars else random.choice(
            [b for b in blib.filter("vehicle.*")
             if "dreyevr" not in b.id
             and int(b.get_attribute("number_of_wheels")) == 4])
        bp.set_attribute("role_name", "background")

        tf = carla.Transform(
            carla.Location(x=wp.transform.location.x,
                           y=wp.transform.location.y,
                           z=wp.transform.location.z + 0.3),
            wp.transform.rotation)

        npc = world.try_spawn_actor(bp, tf)
        if not npc:
            continue

        npc.set_autopilot(False)
        npc.set_simulate_physics(True)
        
        controller = BackgroundController(world, npc, target_lane=-1)
        controllers.append(controller)
        spawned.append(npc)

    print(f"[BGTraffic] {len(spawned)} background vehicles in lane -1")
    return spawned, controllers

# ── Connect ───────────────────────────────────────────────────────────────────
def connect_and_load(host, port, map_name="Town04"):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world  = client.load_world(map_name)
    world.set_weather(carla.WeatherParameters(
        cloudiness=10, precipitation=0,
        sun_altitude_angle=45, sun_azimuth_angle=200,
        fog_density=0, wetness=0))
    return client, world

# ── Ego ───────────────────────────────────────────────────────────────────────
def find_ego_vehicle(world):
    v = list(world.get_actors().filter("harplab.dreyevr_vehicle.*"))
    if v: return v[0]
    bp = world.get_blueprint_library().find("harplab.dreyevr_vehicle.model3")
    return world.spawn_actor(bp, world.get_map().get_spawn_points()[0])

def precise_respawn(vehicle, location, yaw=None, pitch=0.0, roll=0.0):
    try:
        if not isinstance(location, carla.Location):
            location = carla.Location(*location)
        
        if yaw is None:
            waypoint = vehicle.get_world().get_map().get_waypoint(
                location, project_to_road=True)
            yaw = waypoint.transform.rotation.yaw if waypoint else 0.0
        
        transform = carla.Transform(
            location,
            carla.Rotation(pitch=float(pitch), yaw=float(yaw), roll=float(roll))
        )
        
        vehicle.set_simulate_physics(False)
        vehicle.set_transform(transform)
        
        time.sleep(0.1)
        current_yaw = vehicle.get_transform().rotation.yaw
        yaw_diff = abs((current_yaw - yaw + 180) % 360 - 180)
        
        if yaw_diff > 5.0:
            correction = carla.Transform(
                location,
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
            )
            vehicle.set_transform(correction)
            time.sleep(0.1)
        
        vehicle.set_simulate_physics(True)
        return True
    except Exception as e:
        print(f"Respawn failed: {str(e)}")
        try:
            vehicle.set_simulate_physics(True)
        except:
            pass
        return False

# ── Indicator ─────────────────────────────────────────────────────────────────
def show_indicator(world, npc, duration_s=2.5, side="left"):
    if not is_alive(npc): return
    try:
        tf = npc.get_transform(); fwd = tf.get_forward_vector()
        sign = 1.0 if side == "left" else -1.0
        tip = carla.Location(x=tf.location.x + fwd.y*3.5*sign,
                             y=tf.location.y - fwd.x*3.5*sign,
                             z=tf.location.z + 1.5)
        world.debug.draw_arrow(
            tf.location + carla.Location(z=1.0), tip,
            thickness=0.15, arrow_size=0.1,
            color=carla.Color(205, 165, 0),
            life_time=duration_s, persistent_lines=False)
    except: pass

# ── NPC Controller ────────────────────────────────────────────────────────────
class NPCController:
    MIN_LOOKAHEAD = 3.0
    MAX_LOOKAHEAD = 10.0
    KP_STEER = 1.2
    MAX_STEER = 0.6
 
    def __init__(self, world, npc, target_lane):
        self.world = world
        self.npc = npc
        self.target_lane = target_lane
        self.follow_leader = None
 
    def change_lane(self, lane_id):
        print(f"[NPC {self.npc.id}] Lane target → {lane_id}")
        self.target_lane = lane_id
 
    def in_target_lane(self):
        if not is_alive(self.npc): return False
        try:
            wp = self.world.get_map().get_waypoint(
                self.npc.get_transform().location, project_to_road=True,
                lane_type=carla.LaneType.Driving)
            return wp is not None and wp.lane_id == self.target_lane
        except: return False
 
    def _lookahead_for_speed(self, speed_kmh):
        t = max(0.0, min(1.0, speed_kmh / 100.0))
        return self.MIN_LOOKAHEAD + t * (self.MAX_LOOKAHEAD - self.MIN_LOOKAHEAD)
 
    def _closest_vehicle_ahead(self, max_dist=30.0):
        if not is_alive(self.npc): return None, None
        try:
            npc_tf = self.npc.get_transform()
            npc_loc = npc_tf.location
            fwd = npc_tf.get_forward_vector()
            town_map = self.world.get_map()
            npc_wp = town_map.get_waypoint(npc_loc, project_to_road=True,
                                           lane_type=carla.LaneType.Driving)
            if not npc_wp: return None, None
 
            closest_dist = max_dist
            closest_actor = None
 
            for actor in self.world.get_actors().filter("vehicle.*"):
                if not is_alive(actor) or actor.id == self.npc.id:
                    continue
                a_loc = actor.get_transform().location
                d = a_loc - npc_loc
                dot = d.x*fwd.x + d.y*fwd.y
                if dot <= 0 or dot > max_dist:
                    continue
                a_wp = town_map.get_waypoint(a_loc, project_to_road=True,
                                             lane_type=carla.LaneType.Driving)
                if a_wp and a_wp.lane_id == self.target_lane:
                    if dot < closest_dist:
                        closest_dist = dot
                        closest_actor = actor
 
            return (closest_dist, closest_actor) if closest_actor else (None, None)
        except:
            return None, None
 
    def tick(self, target_kmh):
        if not is_alive(self.npc): return
        try:
            npc_tf = self.npc.get_transform()
            loc = npc_tf.location
            fwd = npc_tf.get_forward_vector()
            spd = kmh_of(self.npc)
 
            # Following distance logic
            MIN_FOLLOW_DIST = 4.0
            SAFE_FOLLOW_DIST = 10.0
            dist_ahead, leader = self._closest_vehicle_ahead(max_dist=30.0)
            
            if dist_ahead is not None:
                if dist_ahead < MIN_FOLLOW_DIST:
                    target_kmh = max(0.0, target_kmh * 0.2)
                elif dist_ahead < SAFE_FOLLOW_DIST:
                    ratio = (dist_ahead - MIN_FOLLOW_DIST) / (SAFE_FOLLOW_DIST - MIN_FOLLOW_DIST)
                    leader_spd = kmh_of(leader) if leader else 0.0
                    target_kmh = max(leader_spd * ratio, 2.0)
 
            lookahead = self._lookahead_for_speed(spd)
            look = carla.Location(x=loc.x+fwd.x*lookahead,
                                  y=loc.y+fwd.y*lookahead, z=loc.z)
 
            town_map = self.world.get_map()
            wp = town_map.get_waypoint(look, project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
            if not wp:
                wp = town_map.get_waypoint(loc, project_to_road=True,
                                           lane_type=carla.LaneType.Driving)
            if not wp: return
 
            cur_wp = town_map.get_waypoint(loc, project_to_road=True,
                                           lane_type=carla.LaneType.Driving)
            at_junction = cur_wp and cur_wp.is_junction
 
            if at_junction:
                nexts = cur_wp.next(lookahead)
                tgt = nexts[0] if nexts else cur_wp
            else:
                tgt = wp_in_lane(wp, self.target_lane) or wp
 
            to = carla.Vector3D(tgt.transform.location.x-loc.x,
                                tgt.transform.location.y-loc.y, 0.0)
            mag = math.sqrt(to.x**2+to.y**2)
            if mag > 0.001:
                to.x /= mag
                to.y /= mag
            cross = fwd.x*to.y - fwd.y*to.x
            steer = max(-self.MAX_STEER, min(self.MAX_STEER, self.KP_STEER*cross))
 
            err = target_kmh - spd
            if target_kmh <= 0.0:
                th = 0.0
                br = 0.8
            elif err > 0:
                th = min(1.0, err/20.0)
                br = 0.0
            else:
                th = 0.0
                br = min(1.0, abs(err)/10.0)
 
            self.npc.apply_control(carla.VehicleControl(
                throttle=float(th), steer=float(steer),
                brake=float(br), hand_brake=False))
        except Exception as e:
            print(f"[NPCCtrl] {e}")

# ── Queue Manager ─────────────────────────────────────────────────────────────
class QueueManager:
    """
    Manages all vehicles in the congestion queue including cut-in vehicles.
    Ensures all vehicles follow the queue through all phases.
    """
    def __init__(self):
        self.vehicles = []  # List of CongestionQueueVehicle objects
        self.cutin_vehicles = []  # Track which vehicles are cut-ins
        
    def add_vehicle(self, npc, world, target_lane=-4, is_cutin=False):
        """Add a vehicle to the queue."""
        queue_vehicle = CongestionQueueVehicle(npc, world, target_lane)
        queue_vehicle.is_cutin = is_cutin
        self.vehicles.append(queue_vehicle)
        return queue_vehicle
        
    def add_cutin_vehicle(self, npc, world, target_lane=-4):
        """Add a cut-in vehicle to the queue."""
        return self.add_vehicle(npc, world, target_lane, is_cutin=True)
        
    def update_all(self, target_speed):
        """Update all vehicles in the queue with the target speed."""
        for qv in self.vehicles:
            if qv.alive:
                qv.update(target_speed)
                
    def remove_at_exit(self):
        """Remove vehicles that have reached the exit."""
        removed = []
        for qv in self.vehicles[:]:
            if qv.alive and qv.is_at_exit():
                print(f"[Queue] Vehicle {qv.npc.id} reached exit, removing...")
                safe_destroy(qv.npc)
                qv.alive = False
                removed.append(qv)
                self.vehicles.remove(qv)
        return removed
        
    def get_alive_count(self):
        return sum(1 for qv in self.vehicles if qv.alive)
        
    def get_average_speed(self):
        speeds = [kmh_of(qv.npc) for qv in self.vehicles if qv.alive]
        return sum(speeds) / len(speeds) if speeds else 0.0

class CongestionQueueVehicle:
    def __init__(self, npc, world, target_lane=-4, position_in_queue=0):
        self.npc = npc
        self.controller = NPCController(world, npc, target_lane)
        self.alive = True
        self.position = position_in_queue
        self.is_cutin = False  # Set by QueueManager
        
    def update(self, target_speed):
        if not is_alive(self.npc):
            self.alive = False
            return
            
        _, leader = self.controller._closest_vehicle_ahead(max_dist=30.0)
        
        if leader:
            leader_speed = kmh_of(leader)
            dist = dist_ahead(self.npc, leader)
            
            if dist < 3.0:
                speed_target = max(0.0, leader_speed - 3.0)
            elif dist < 6.0:
                speed_target = min(leader_speed, target_speed)
            else:
                speed_target = target_speed
        else:
            speed_target = target_speed
        
        self.controller.tick(speed_target)
        
    def is_at_exit(self):
        if not is_alive(self.npc):
            return True
        try:
            wp = self.npc.get_world().get_map().get_waypoint(
                self.npc.get_transform().location, project_to_road=True,
                lane_type=carla.LaneType.Driving)
            return wp is not None and wp.road_id == 275
        except:
            return False

# ── A-Event (A1-A4) ───────────────────────────────────────────────────────────
class AEvent:
    def __init__(self, label, indicator, decel_type, start_t, model):
        self.label = label
        self.indicator = indicator
        self.decel_type = decel_type
        self.start_t = start_t
        self.model = model
        self.npc = None
        self.ctrl = None
        self.state = "waiting"
        self._state_t = 0.0
        self._ind_done = False
        self._spawn_t = start_t - SPAWN_BEFORE_S
        print(f"[{label}] lane-3→lane-2  ind={'ON' if indicator else 'OFF'}  "
              f"decel={decel_type.name}  spawn@{self._spawn_t:.0f}s")

    def _set_state(self, s, t):
        print(f"[{self.label}] {self.state} -> {s}  T={t:.1f}s")
        self.state = s
        self._state_t = t

    def update(self, world, ego, t):
        try:
            return self._update(world, ego, t)
        except Exception as e:
            print(f"[{self.label}] err: {e}")
            return ""

    def _update(self, world, ego, t):
        ego_spd = kmh_of(ego) if is_alive(ego) else SPEED_CRUISE

        if self.state == "waiting":
            if t >= self._spawn_t:
                npc = spawn_behind_in_lane(world, ego, lane_id=-3,
                                           behind_m=8.0, model=self.model)
                if npc:
                    self.npc = npc
                    self.ctrl = NPCController(world, npc, target_lane=-3)
                    self._set_state("tailing", t)
                else:
                    print(f"[{self.label}] spawn failed — retry")
            return ""

        if not is_alive(self.npc):
            return f"{self.label} (gone)"
        ahead = dist_ahead(ego, self.npc)

        if self.state == "tailing":
            self.ctrl.tick(max(ego_spd, 110.0))
            if t >= self.start_t:
                self._set_state("overtake", t)
            return f"{self.label} — tailing"

        elif self.state == "overtake":
            ramp = min((t-self._state_t)/5.0, 1.0)
            spd = max(ego_spd, ramp*(SPEED_CATCHUP + 10))
            self.ctrl.tick(spd)
            if ahead >= CUT_AHEAD_M:
                if not self._ind_done:
                    if self.indicator:
                        show_indicator(world, self.npc, 2.5)
                    self._ind_done = True
                self.ctrl.change_lane(-2)
                self._set_state("cutin", t)
            return f"{self.label} — overtaking"

        elif self.state == "cutin":
            self.ctrl.tick(SPEED_CRUISE)
            if self.ctrl.in_target_lane():
                self._set_state("decel", t)
            return f"{self.label} — cutting in"

        elif self.state == "decel":
            progress = min((t-self._state_t)/self.decel_type.value, 1.0)
            spd = SPEED_SLOW + progress*(SPEED_SLOW-SPEED_CRUISE)
            self.ctrl.tick(spd)
            if progress >= 1.0:
                self._set_state("tail_ego", t)
            return f"{self.label} — decel"

        elif self.state == "tail_ego":
            self.ctrl.tick(SPEED_SLOW)
            if t-self._state_t >= 5.0:
                self._set_state("reaccel", t)
            return f"{self.label} — tail"

        elif self.state == "reaccel":
            progress = min((t-self._state_t)/self.decel_type.value, 1.0)
            spd = SPEED_SLOW + progress*(SPEED_CRUISE-SPEED_SLOW) + 20
            self.ctrl.tick(spd)
            if progress >= 1.0:
                self._set_state("done", t)
            return f"{self.label} — reaccel"

        elif self.state == "done":
            self.ctrl.tick(SPEED_CRUISE)
            if ahead > DESTROY_M or ahead < -50.0:
                safe_destroy(self.npc)
                self.npc = None
                return f"{self.label} — done"
            return f"{self.label} — cruising"

        return ""

# ── Congestion cut-in (FIXED: joins queue permanently) ──────────────────────
class CongestionCutIn:
    """
    NPC spawns in lane -2, cuts RIGHT into lane -4 (queue),
    then joins the queue permanently.
    """
    def __init__(self, label, trigger_t, queue_manager, solid_line=False,
                 model="vehicle.audi.a2"):
        self.label = label
        self.trigger_t = trigger_t
        self.queue_manager = queue_manager
        self.solid_line = solid_line
        self.model = model
        self.npc = None
        self.ctrl = None
        self.state = "waiting"
        self._state_t = 0.0
        self._ind_done = False
        self._joined_queue = False
        print(f"[{label}] lane-2→lane-4 (joins queue) @ {trigger_t}s  "
              f"solid={'YES' if solid_line else 'no'}")

    def _set_state(self, s, t):
        print(f"[{self.label}] {self.state} -> {s}  T={t:.1f}s")
        self.state = s
        self._state_t = t

    def update(self, world, ego, t):
        try:
            return self._update(world, ego, t)
        except Exception as e:
            print(f"[{self.label}] err: {e}")
            return ""

    def _update(self, world, ego, t):
        if self.state == "waiting":
            if t >= self.trigger_t:
                # Spawn in lane -2 (flowing traffic)
                npc = spawn_behind_in_lane(world, ego, lane_id=-3,
                                           behind_m=15.0, model=self.model)
                if npc:
                    self.npc = npc
                    self.ctrl = NPCController(world, npc, target_lane=-3)
                    self._set_state("overtake", t)
                else:
                    print(f"[{self.label}] spawn failed")
            return ""

        if not is_alive(self.npc):
            return f"{self.label} (gone)"
            
        ahead = dist_ahead(ego, self.npc)

        if self.state == "overtake":
            # Drive in lane -3 at speed to catch up to queue
            ramp = min((t-self._state_t)/5.0, 1.0)
            spd = SPEED_CONG_MAX + ramp*(SPEED_CATCHUP-SPEED_SLOW-10)
            self.ctrl.tick(spd)
            
            # Check if there's a gap in the queue (lane -4)
            has_gap, gap_size, _ = find_gap_in_lane(
                world, self.npc, lane_id=-4, min_gap_m=10.0,
                search_ahead_m=40.0)
                
            if has_gap and ahead >= 1.0:
                if not self._ind_done:
                    if not self.solid_line:
                        show_indicator(world, self.npc, 2.5, side="right")
                        print(f"[{self.label}] Indicator ON — merging into queue")
                    else:
                        print(f"[{self.label}] No indicator (solid line) — illegal merge")
                    self._ind_done = True
                    
                # Cut RIGHT into lane -4 (queue lane)
                self.ctrl.change_lane(-4)
                self._set_state("merge", t)
                return f"{self.label} — merging into queue (gap={gap_size:.0f}m)"
            else:
                return f"{self.label} — pacing queue  gap={gap_size:.0f}m"

        elif self.state == "merge":
            self.ctrl.tick(SPEED_CONG_MIN)
            if self.ctrl.in_target_lane():
                print(f"[{self.label}] ✓ MERGED into queue!")
                self._set_state("queue", t)
            return f"{self.label} — merging"

        elif self.state == "queue":
            # JOIN THE QUEUE PERMANENTLY
            if not self._joined_queue:
                print(f"[{self.label}] Adding to queue manager...")
                self.queue_manager.add_cutin_vehicle(self.npc, world, target_lane=-4)
                self._joined_queue = True
                self._set_state("following", t)
            return f"{self.label} — joined queue"

        elif self.state == "following":
            # The queue manager will handle updates
            return f"{self.label} — following queue"

        return ""

# ── Exit cut-in ───────────────────────────────────────────────────────────────
class ExitCutIn:
    def __init__(self):
        self.npc = None
        self.ctrl = None
        self.state = "waiting"
        self._state_t = 0.0
        print("[Exit] Ready @ 810s")

    def _set_state(self, s, t):
        print(f"[Exit] {self.state} -> {s}  T={t:.1f}s")
        self.state = s
        self._state_t = t

    def update(self, world, ego, t):
        try:
            return self._update(world, ego, t)
        except Exception as e:
            print(f"[Exit] err: {e}")
            return ""

    def _update(self, world, ego, t):
        ego_spd = kmh_of(ego) if is_alive(ego) else SPEED_CRUISE

        if self.state == "waiting":
            if t >= 810.0:
                npc = spawn_behind_in_lane(world, ego, lane_id=-3,
                                           behind_m=8.0,
                                           model="vehicle.mercedes.coupe")
                if npc:
                    self.npc = npc
                    self.ctrl = NPCController(world, npc, -3)
                    self._set_state("overtake", t)
            return ""

        if not is_alive(self.npc):
            return "Exit gone"
        ahead = dist_ahead(ego, self.npc)

        if self.state == "overtake":
            ramp = min((t-self._state_t)/4.0, 1.0)
            spd = ego_spd + ramp*(SPEED_CATCHUP-ego_spd)
            self.ctrl.tick(spd)
            if ahead >= 1.0:
                print("[Exit] Sudden cut-in — NO indicator!")
                self.ctrl.change_lane(-4)
                self._set_state("cutin", t)
            return f"Exit — overtaking"

        elif self.state == "cutin":
            self.ctrl.tick(SPEED_CONG_MAX)
            if self.ctrl.in_target_lane():
                self._set_state("hard_brake", t)
            return "Exit — cutting in"

        elif self.state == "hard_brake":
            elapsed = t - self._state_t
            if elapsed < 4.0:
                self.npc.apply_control(carla.VehicleControl(
                    throttle=0.0, brake=0.9, hand_brake=False))
                return f"Exit — HARD BRAKE"
            self._set_state("cruise", t)
            return "Exit — braking"

        elif self.state == "cruise":
            self.ctrl.tick(40.0)
            if ahead > DESTROY_M or ahead < -50.0:
                safe_destroy(self.npc)
                self.npc = None
                return "Exit — done"
            return f"Exit — slow cruise"

        return ""

# ── Rating prompt ─────────────────────────────────────────────────────────────
def show_rating(world, ego, msg, duration_s=15.0):
    if not is_alive(ego): return
    try:
        loc = ego.get_transform().location
        world.debug.draw_string(loc+carla.Location(z=8.0), msg,
            draw_shadow=True, color=COLOR_RATING,
            life_time=duration_s, persistent_lines=False)
        print(f"\n[RATING] {msg}\n")
    except: pass

# ── HUD & Speed Monitor ───────────────────────────────────────────────────────
class HUD:
    def __init__(self, world, ego):
        self.world = world
        self.ego = ego
    def update(self, spd, warning, t, label=""):
        if not is_alive(self.ego): return
        try:
            loc = self.ego.get_transform().location
        except:
            return
        col = (COLOR_OK if SPEED_LOW <= spd <= SPEED_HIGH else
               COLOR_DANGER if spd < WARN_LOW or spd > SPEED_HIGH else COLOR_WARN)
        lt = FDS*2
        self.world.debug.draw_string(loc+carla.Location(z=5.0),
            f"Speed: {spd:.1f} km/h   T+{t:.0f}s",
            draw_shadow=True, color=col, life_time=lt, persistent_lines=False)
        if label:
            self.world.debug.draw_string(loc+carla.Location(z=6.5), label,
                draw_shadow=True, color=COLOR_INFO, life_time=lt, persistent_lines=False)
        if warning:
            self.world.debug.draw_string(loc+carla.Location(z=3.8), warning,
                draw_shadow=True, color=COLOR_DANGER, life_time=lt, persistent_lines=False)

class SpeedMonitor:
    def __init__(self, world, ego):
        self.world = world
        self.ego = ego
        self.hud = HUD(world, ego)
        self.scenario_time = 0.0
        self.slow_since = None
        self.warning = ""
    def ego_kmh(self):
        if not is_alive(self.ego): return 0.0
        try:
            v = self.ego.get_velocity()
            return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        except:
            return 0.0
    def _warn(self, spd):
        if spd > SPEED_HIGH:
            self.slow_since = None
            return "Please drive at 100-105 km/h. Your current speed is too fast."
        if spd < WARN_LOW:
            if self.slow_since is None:
                self.slow_since = self.scenario_time
            elif self.scenario_time - self.slow_since >= WARN_DELAY:
                return ("Please drive at 100-105 km/h. "
                        "Your current speed is too low. "
                        "Please accelerate to the target speed.")
        else:
            self.slow_since = None
        if SPEED_LOW <= spd <= SPEED_HIGH:
            return ""
        return self.warning
    def tick(self, dt, label=""):
        self.scenario_time += dt
        spd = self.ego_kmh()
        self.warning = self._warn(spd)
        self.hud.update(spd, self.warning, self.scenario_time, label)
        return self.scenario_time, spd

def beep(world, loc):
    world.debug.draw_point(loc+carla.Location(z=2.5),
        size=0.8, color=carla.Color(255,255,255), life_time=0.5)
    print("[BEEP] *** t=0 ***")

# ── Traffic Light Control ─────────────────────────────────────────────────────
def set_all_traffic_lights_green(world):
    """
    Set all traffic lights in the world to green and disable autopilot control.
    """
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    for traffic_light in traffic_lights:
        try:
            # Set the light to green
            traffic_light.set_state(carla.TrafficLightState.Green)
            # Disable automatic control so it stays green
            traffic_light.set_green_time(9999.0)
            traffic_light.set_red_time(0.0)
            traffic_light.set_yellow_time(0.0)
        except Exception as e:
            print(f"[TrafficLight] Error setting light: {e}")

def keep_traffic_lights_green(world):
    """
    Periodically check and ensure all traffic lights remain green.
    Call this in the main loop.
    """
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    for traffic_light in traffic_lights:
        try:
            current_state = traffic_light.get_state()
            if current_state != carla.TrafficLightState.Green:
                traffic_light.set_state(carla.TrafficLightState.Green)
                # Reset timers to keep it green
                traffic_light.set_green_time(9999.0)
                traffic_light.set_red_time(0.0)
                traffic_light.set_yellow_time(0.0)
        except Exception as e:
            pass  # Silently continue if there's an error

class SimpleCrashDetector:
    """
    Simple detector that only removes vehicles that are clearly crashed.
    """
    def __init__(self, queue_manager):
        self.queue_manager = queue_manager
        
    def update(self, world, t):
        """Check for crashed vehicles and remove them."""
        vehicles = world.get_actors().filter('vehicle.*')
        removed = []
        
        for vehicle in vehicles:
            if not is_alive(vehicle):
                continue
                
            # Check for obvious crash conditions
            if self._is_obviously_crashed(vehicle):
                print(f"[CrashDetect] 🚗 Removing crashed vehicle {vehicle.id}")
                safe_destroy(vehicle)
                removed.append(vehicle.id)
                
                # Remove from queue if it was in the queue
                for qv in self.queue_manager.vehicles[:]:
                    if qv.alive and qv.npc and qv.npc.id == vehicle.id:
                        qv.alive = False
                        self.queue_manager.vehicles.remove(qv)
                        break
        
        return removed
    
    def _is_obviously_crashed(self, vehicle):
        """
        Check for obvious crash indicators.
        """
        try:
            transform = vehicle.get_transform()
            rotation = transform.rotation
            
            # Vehicle is flipped or on its side
            if abs(rotation.roll) > 70 or abs(rotation.pitch) > 70:
                return True
            
            # Vehicle is upside down (forward vector pointing down)
            forward = transform.get_forward_vector()
            if forward.z < -0.5:
                return True
            
            # Extreme angular velocity (spinning out)
            angular = vehicle.get_angular_velocity()
            speed = kmh_of(vehicle)
            if speed < 2.0 and abs(angular.z) > 6.0:
                return True
                
        except:
            pass
            
        return False

import subprocess
import sys

# ── TTS ───────────────────────────────────────────────────────────────────────
# ── Rating prompts ────────────────────────────────────────────────────────────
# (trigger_t, display_s, title, line1, line2, spoken_text)
 
def _speak(text: str):
    """Speak text in a separate process — no audio device conflicts."""
    script = (
        "import pyttsx3; e=pyttsx3.init(); "
        "e.setProperty('rate',155); e.setProperty('volume',1.0); "
        f"e.say({repr(text)}); e.runAndWait()"
    )
    subprocess.Popen(
        [sys.executable, "-c", script],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    client, world = connect_and_load(CARLA_HOST, CARLA_PORT)
    all_npcs = []
    bg_controllers = []
    
    # Create queue manager
    queue_manager = QueueManager()

    # ── SET ALL TRAFFIC LIGHTS TO GREEN ─────────────────────────────
    print("[TrafficLights] Setting all traffic lights to green...")
    set_all_traffic_lights_green(world)

     # ── ADD THIS: Create crash detector ──
    crash_detector = SimpleCrashDetector(queue_manager)

    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(False)
    tm.set_random_device_seed(42)
    tm.set_global_distance_to_leading_vehicle(8.0)
    # tm.ignore_lights_percentage(100.0)  # Make TM ignore traffic lights

    try:
        ego = find_ego_vehicle(world)
        success = precise_respawn(ego, carla.Location(x=-502, y=69, z=0.23), yaw=None)

        
        # beep(world, ego.get_transform().location)

        # Background traffic with custom controller
        bg_npcs, bg_controllers = spawn_background_custom(world, ego, num_vehicles=3)
        world.tick()
        all_npcs.extend(bg_npcs)
        world.tick()

        # A1-A4 events
        a_events = [
            AEvent("A1", True,  DecelType.RAPID,    60.0, "vehicle.audi.tt"),
            AEvent("A2", True,  DecelType.GRADUAL, 170.0, "vehicle.bmw.grandtourer"),
            AEvent("A3", False, DecelType.RAPID,   280.0, "vehicle.mercedes.coupe"),
            AEvent("A4", False, DecelType.GRADUAL, 390.0, "vehicle.dodge.charger_2020"),
        ]

        # Congestion cut-ins (pass queue_manager to each)
        c_events = [
            CongestionCutIn("C1", 545.0, queue_manager, solid_line=False, model="vehicle.dodge.charger_2020"),
            CongestionCutIn("C2", 615.0, queue_manager, solid_line=False, model="vehicle.audi.tt"),
            CongestionCutIn("C3", 685.0, queue_manager, solid_line=False, model="vehicle.ford.mustang"),
        ]

        exit_event = ExitCutIn()

        rating_schedule = {
            120.0: "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
            230.0: "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
            320.0: "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
            440.0: "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
            545.0: "How much anger or frustration do you feel\ndue to the current traffic congestion?",
            570.0: "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
            635.0: "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
            705.0: "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
            805.0: "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
        }
        prompted = set()
        congestion_spawned = False
        recovery_started = False
        exit_road_reached = False

        ORIGINAL_EGO_SPAWN = carla.Location(x=80.25, y=-348.11, z=0.23)

        print(f"[Congestion] Pre-spawning queue at exit location...")

        spawn_wp = world.get_map().get_waypoint(
            ORIGINAL_EGO_SPAWN, project_to_road=True,
            lane_type=carla.LaneType.Driving)

        if spawn_wp:
            lane4_wp = wp_in_lane(spawn_wp, -4) or spawn_wp
            original_road = lane4_wp.road_id

            test_prev = lane4_wp.previous(20.0)
            test_next = lane4_wp.next(20.0)

            if test_next and test_next[0].road_id == original_road:
                step_func_name = "next"
            elif test_prev and test_prev[0].road_id == original_road:
                step_func_name = "previous"
            else:
                step_func_name = "previous"

            print(f"[Congestion] Highway direction: {step_func_name}()")

            if step_func_name == "next":
                close = lane4_wp.previous(2.0)
                cur_wp = close[0] if close else lane4_wp
            else:
                close = lane4_wp.next(2.0)
                cur_wp = close[0] if close else lane4_wp

            cur_wp = wp_in_lane(cur_wp, -4) or cur_wp

        monitor = SpeedMonitor(world, ego)
        status_every = int(2.0 / FDS)
        tick_count = 0

        print("\n[Scenario 1] Running. Ctrl+C to stop.\n")

        while True:
            world.tick()
            if not is_alive(ego):
                print("Ego lost.")
                break

            t, ego_spd = monitor.tick(FDS)
            label = "Adaptation — drive 100-105 km/h" if t < 60.0 else ""

            # Update background vehicles
            for ctrl in bg_controllers:
                ctrl.update()

            # ── ENSURE TRAFFIC LIGHTS STAY GREEN ────────────────────────────
            # Check every few ticks to reduce performance impact
            if tick_count % 10 == 0:  # Check every 10 ticks
                keep_traffic_lights_green(world)

            # ── A1-A4 (0-500s) ──────────────────────────────────────────
            if t < 500.0:
                for ev in a_events:
                    lbl = ev.update(world, ego, t)
                    if lbl:
                        label = lbl
                    if ev.npc and ev.npc not in all_npcs and is_alive(ev.npc):
                        all_npcs.append(ev.npc)

            # ── Spawn congestion queue at t=450 ─────────────────────────
            if not congestion_spawned and t >= 450.0:
                for i in range(NUM_CONG_VEHS):
                    if cur_wp is None:
                        print(f"[Congestion] cur_wp is None at vehicle {i}")
                        break
                    print(f"[Congestion] Spawning vehicle {i}: "
                          f"lane={cur_wp.lane_id}  "
                          f"x={cur_wp.transform.location.x:.1f}  "
                          f"y={cur_wp.transform.location.y:.1f}")
                    npc = spawn_at_wp(world, cur_wp)
                    if npc:
                        npc.set_autopilot(False)
                        npc.set_simulate_physics(True)
                        queue_manager.add_vehicle(npc, world, target_lane=-4)
                        all_npcs.append(npc)
                        
                    if step_func_name == "next":
                        nxt = cur_wp.next(14.0)
                    else:
                        nxt = cur_wp.previous(14.0)
                    cur_wp = nxt[0] if nxt else None
                    
                print(f"[Congestion] {queue_manager.get_alive_count()} queue vehicles spawned")
                congestion_spawned = True

            # ── 500-750s: CONGESTION (VERY SLOW) ────────────────────────
            if 520.0 <= t < 750.0:
                label = f"🚗 CONGESTION — 5-10 km/h (lane-4)"
                
                # Very slow speed: 5-10 km/h
                target_speed = random.uniform(SPEED_CONG_MIN, SPEED_CONG_MAX)
                
                # Update ALL queue vehicles (including cut-ins)
                queue_manager.update_all(target_speed)
                
                # Check if any vehicles reached exit
                # removed = queue_manager.remove_at_exit()
                # if removed:
                #     exit_road_reached = True
                
                # Update congestion cut-ins (they will join queue)
                for ev in c_events:
                    lbl = ev.update(world, ego, t)
                    if lbl and "following" not in lbl:  # Don't spam "following" messages
                        label = lbl
                    if ev.npc and ev.npc not in all_npcs and is_alive(ev.npc):
                        all_npcs.append(ev.npc)

            # ── 750-800s: RECOVERY (GRADUAL ACCELERATION) ──────────────
            elif 750.0 <= t < 790.0:
                if not recovery_started:
                    print("[Congestion] 🚦 RECOVERY STARTING — Gradual acceleration...")
                    recovery_started = True
                
                progress = (t - 750.0) / 50.0
                eased_progress = progress * progress * (3 - 2 * progress)
                recovery_spd = SPEED_RECOVERY_MIN + eased_progress * (SPEED_RECOVERY_MAX - SPEED_RECOVERY_MIN)
                
                label = f"🚦 RECOVERY — {recovery_spd:.0f} km/h"
                
                # Update ALL queue vehicles with recovery speed
                queue_manager.update_all(recovery_spd)
                
                # Remove vehicles at exit
                # removed = queue_manager.remove_at_exit()
                # if removed:
                #     exit_road_reached = True
                
                # Continue updating cut-in events (any that haven't joined yet)
                for ev in c_events:
                    lbl = ev.update(world, ego, t)
                    if lbl:
                        label = lbl
                    if ev.npc and ev.npc not in all_npcs and is_alive(ev.npc):
                        all_npcs.append(ev.npc)

            # # ── 800-810s: pre-exit ──────────────────────────────────────
            # elif 800.0 <= t < 810.0:
            #     label = "Pre-exit transition"
            #     queue_manager.update_all(40.0)

            # ── 810-890s: exit cut-in ──────────────────────────────────
            elif 790.0 <= t < 890.0:
                lbl = exit_event.update(world, ego, t)
                if lbl:
                    label = lbl
                if (exit_event.npc and exit_event.npc not in all_npcs
                        and is_alive(exit_event.npc)):
                    all_npcs.append(exit_event.npc)
                
                queue_manager.update_all(40.0)

            elif t >= 890.0:
                label = "Cooldown"
                removed = queue_manager.remove_at_exit()
                if removed:
                    print(f"[Cooldown] Removed {len(removed)} vehicles at exit")

            # ── Rating prompts ──────────────────────────────────────────
            for pt, msg in rating_schedule.items():
                if t >= pt and pt not in prompted:
                    show_rating(world, ego, msg, duration_s=15.0)
                    _speak(msg)
                    prompted.add(pt)

            # ── Crash detection ─────────────────────────────────────────
            # ── ADD THIS: Check for crashed vehicles ──
            if tick_count % 10 == 0:  # Check every 10 ticks
                removed = crash_detector.update(world, t)
                if removed:
                    print(f"[CrashDetect] Removed {len(removed)} crashed vehicles")

            monitor.hud.update(ego_spd, monitor.warning, t, label)
            tick_count += 1
            if tick_count % status_every == 0:
                alive_count = queue_manager.get_alive_count()
                avg_speed = queue_manager.get_average_speed()
                bg_count = len([c for c in bg_controllers if is_alive(c.npc)])
                print(f"  T={t:>6.1f}s  ego={ego_spd:>5.1f} km/h  "
                      f"queue={alive_count}  avg_spd={avg_speed:>4.1f} km/h  "
                      f"bg={bg_count}  [{label[:30]}]")

            if t >= 900.0:
                print("\n[Scenario 1] Complete at T=900s.")
                break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[Scenario 1] Stopped.")
    finally:
        for npc in all_npcs:
            safe_destroy(npc)
        try:
            s = world.get_settings()
            s.synchronous_mode = False
            s.fixed_delta_seconds = None
            world.apply_settings(s)
        except:
            pass
        print("[Scenario 1] Done.")

if __name__ == "__main__":
    main()                           