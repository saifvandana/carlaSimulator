"""
Scenario 2 — Fixed Implementation
=====================================
Merge Failure + Congestion + Shoulder Violations + Blocking Vehicle

Timeline:
  0-100s:   Adaptation (ramp driving)
  100-160s: Merge blocked (real congestion queue across lanes -1/-2/-3)
  160-180s: Gap window — queue opens a real gap in lane -3, ego can merge
  180-220s: Stabilization
  220-440s: Congestion + shoulder events (vehicles cycle, never go stale)
  440-460s: Recovery
  460+s:    Blocking vehicle

FIXES APPLIED (vs. previous version):
  1. Merge success is now detected from ego's actual road/lane via
     ego_has_merged(), not a hardcoded "t >= 170" timer.
  2. The single MergeBlocker car is replaced by a real congestion queue
     (lanes -1/-2/-3) that ego genuinely cannot merge into until the
     160-180s gap window, when two lane-3 vehicles nearest ego are
     slowed to open a real, geometrically-checkable gap.
  3. All spawning is anchored to a known highway waypoint
     (HIGHWAY_ENTRANCE on MERGE_ROAD) and then walked with .previous()/
     .next() across the actual road graph — not derived by lane-walking
     from ego's current position, which fails while ego is still on the
     ramp (different road than the highway).
  4. Congestion/shoulder vehicles are now cycled: any vehicle that drifts
     too far ahead or behind ego is respawned at a sane anchor position
     instead of being spawned once and abandoned.
"""

import carla
import time
import math
import random
from enum import Enum
import pyttsx3
import threading

CARLA_HOST = "localhost"
CARLA_PORT = 2000
FDS = 0.05
TM_PORT = 8001
ROAD_LIMIT = 120.0

# ── Map anchors (verify these against your map before running) ──────────────
# These come from Scenario 1 / Step-3 debugging: road 36 is the short merge
# wedge, road 35 is the upstream highway lanes -1/-2/-3 feed from.
MERGE_ROAD = 36
HIGHWAY_ROAD_IDS = {35, 36}          # any road id ego could be on post-merge
HIGHWAY_ENTRANCE = carla.Location(x=381.9, y=-197.2, z=0.3)
RAMP_SPAWN = carla.Location(x=65.25, y=-338.11, z=0.23)

# ── Speed Parameters ──────────────────────────────────────────────────────────
SPEED_RAMP = 80.0
SPEED_MERGE_TARGET = 100.0
SPEED_CONGESTION_MIN = 10.0
SPEED_CONGESTION_MAX = 20.0
SPEED_SHOULDER = 55.0
SPEED_BLOCKING = 60.0
SPEED_CRUISE = 100.0
SPEED_QUEUE_MIN = 5.0
SPEED_QUEUE_MAX = 10.0
GAP_STOP_SPEED = 3.0

# ── Colors ────────────────────────────────────────────────────────────────────
COLOR_OK = carla.Color(0, 220, 0)
COLOR_WARN = carla.Color(255, 180, 0)
COLOR_DANGER = carla.Color(255, 40, 40)
COLOR_INFO = carla.Color(100, 200, 255)
COLOR_RATING = carla.Color(255, 255, 0)
COLOR_COUNTDOWN = carla.Color(255, 100, 0)
COLOR_SHOULDER = carla.Color(255, 165, 0)

class ScenarioPhase(Enum):
    ADAPTATION = "Adaptation"
    MERGE_BLOCKED = "Merge Blocked"
    GAP_WINDOW = "Gap Window"
    STABILIZATION = "Stabilization"
    CONGESTION = "Congestion"
    RECOVERY = "Recovery"
    BLOCKING = "Blocking Vehicle"

# ── Text-to-Speech System ────────────────────────────────────────────────────
class VoiceSystem:
    def __init__(self):
        self.engine = None
        self.initialized = False
        self._lock = threading.Lock()
        self.is_speaking = False

    def initialize(self):
        try:
            self.engine = pyttsx3.init()
            self.engine.setProperty('rate', 150)
            self.engine.setProperty('volume', 0.9)
            voices = self.engine.getProperty('voices')
            if voices:
                for voice in voices:
                    if 'female' in voice.name.lower():
                        self.engine.setProperty('voice', voice.id)
                        break
                else:
                    self.engine.setProperty('voice', voices[0].id)
            self.initialized = True
            print("[Voice] OK text-to-speech initialized")
            return True
        except Exception as e:
            print(f"[Voice] FAILED to initialize TTS: {e}")
            return False

    def speak(self, text, wait=False):
        if not self.initialized:
            print(f"[Voice] (Would speak): {text}")
            return

        def _speak():
            with self._lock:
                try:
                    self.is_speaking = True
                    self.engine.say(text)
                    self.engine.runAndWait()
                    self.is_speaking = False
                except Exception as e:
                    print(f"[Voice] Error speaking: {e}")
                    self.is_speaking = False

        if wait:
            _speak()
        else:
            threading.Thread(target=_speak, daemon=True).start()

    def speak_with_beep(self, text, world, location):
        self._play_beep(world, location)
        time.sleep(0.3)
        self.speak(text, wait=False)

    def _play_beep(self, world, location):
        try:
            world.debug.draw_point(
                location + carla.Location(z=2.5), size=0.8,
                color=carla.Color(255, 255, 255), life_time=0.5)
            world.debug.draw_string(
                location + carla.Location(z=4.0), "BEEP",
                draw_shadow=True, color=carla.Color(255, 255, 0),
                life_time=0.5, persistent_lines=False)
            print("[BEEP] *** Synchronization signal ***")
        except: pass

# ── Utilities ─────────────────────────────────────────────────────────────────
def pct(kmh): return ((ROAD_LIMIT - kmh) / ROAD_LIMIT) * 100.0

def is_alive(a):
    try: return a is not None and a.is_alive
    except: return False

def safe_destroy(a):
    try:
        if is_alive(a):
            a.set_autopilot(False)
            a.destroy()
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
        tf = ego.get_transform()
        fwd = tf.get_forward_vector()
        d = npc.get_transform().location - tf.location
        return d.x*fwd.x + d.y*fwd.y
    except: return 0.0

def get_wp(world, actor):
    if not is_alive(actor): return None
    return world.get_map().get_waypoint(
        actor.get_transform().location, project_to_road=True,
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
    bp = None
    if model:
        found = blib.filter(model)
        if found: bp = found[0]
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
    return world.try_spawn_actor(bp, tf)

# ── FIX #3: anchored spawn geometry ──────────────────────────────────────────
# These never depend on ego's current waypoint, so they work even while ego
# is still on the ramp (a different road than the highway lanes).

_HIGHWAY_ANCHOR_CACHE = {}

def get_highway_anchor(world, lane_id):
    """
    Returns a waypoint on the upstream highway road (NOT the short merge
    wedge) for the given lane, by anchoring on HIGHWAY_ENTRANCE / MERGE_ROAD
    and stepping back once onto the road that actually feeds it.
    Cached per lane since the map doesn't change mid-run.
    """
    if lane_id in _HIGHWAY_ANCHOR_CACHE:
        return _HIGHWAY_ANCHOR_CACHE[lane_id]

    town_map = world.get_map()
    all_wps = town_map.generate_waypoints(5.0)
    entry_candidates = [wp for wp in all_wps
                         if wp.road_id == MERGE_ROAD and wp.lane_id == lane_id]
    if not entry_candidates:
        print(f"[Anchor] No entry wp for lane {lane_id} on road {MERGE_ROAD}")
        return None
    entry_candidates.sort(key=lambda w: w.s)
    entry_wp = entry_candidates[0]

    prevs = entry_wp.previous(5.0)
    anchor = prevs[0] if prevs else entry_wp
    _HIGHWAY_ANCHOR_CACHE[lane_id] = anchor
    print(f"[Anchor] lane {lane_id} -> road {anchor.road_id} "
          f"lane {anchor.lane_id} s={anchor.s:.1f}")
    return anchor

def spawn_on_highway(world, lane_id, offset_from_anchor_m, model=None):
    """
    Spawn a vehicle on the real highway lane at a distance upstream
    (positive = further back from the merge point) of the anchor.
    """
    anchor = get_highway_anchor(world, lane_id)
    if not anchor: return None
    if offset_from_anchor_m > 0:
        wps = anchor.previous(offset_from_anchor_m)
    elif offset_from_anchor_m < 0:
        wps = anchor.next(-offset_from_anchor_m)
    else:
        wps = [anchor]
    if not wps: return None
    npc = spawn_at_wp(world, wps[0], model)
    if npc:
        npc.set_autopilot(False)
        npc.set_simulate_physics(True)
    return npc

def walk_highway(world, lane_id, start_offset_m, step_m, count, model=None):
    """Spawn `count` vehicles spaced step_m apart, starting start_offset_m
    upstream of the anchor, all on the real highway road for that lane."""
    anchor = get_highway_anchor(world, lane_id)
    if not anchor: return []
    start = anchor.previous(start_offset_m)
    cur = start[0] if start else anchor
    out = []
    for i in range(count):
        if i > 0:
            nxt = cur.next(step_m)
            if not nxt: break
            cur = nxt[0]
        npc = spawn_at_wp(world, cur, model)
        if npc:
            npc.set_autopilot(False)
            npc.set_simulate_physics(True)
            out.append(npc)
    return out

def find_gap_in_lane(world, reference_actor, lane_id, min_gap_m=10.0,
                      search_ahead_m=60.0):
    if not is_alive(reference_actor): return False, 0.0
    try:
        ref_loc = reference_actor.get_transform().location
        ref_fwd = reference_actor.get_transform().get_forward_vector()
        actors = world.get_actors().filter("vehicle.*")
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
                in_lane.append(dist)
        if not in_lane:
            return True, 999.0
        in_lane.sort()
        prev = -5.0
        max_gap = 0.0
        for d in in_lane:
            max_gap = max(max_gap, d - prev)
            prev = d
        return max_gap >= min_gap_m, max_gap
    except Exception as e:
        print(f"[GapCheck] {e}")
        return False, 0.0

def show_indicator(world, npc, duration_s=2.5, side="left"):
    if not is_alive(npc): return
    try:
        tf = npc.get_transform()
        fwd = tf.get_forward_vector()
        sign = 1.0 if side == "left" else -1.0
        tip = carla.Location(
            x=tf.location.x + fwd.y * 3.5 * sign,
            y=tf.location.y - fwd.x * 3.5 * sign,
            z=tf.location.z + 1.5)
        world.debug.draw_arrow(
            tf.location + carla.Location(z=1.0), tip, thickness=0.15,
            arrow_size=0.3, color=carla.Color(255, 165, 0),
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

    def change_lane(self, lane_id):
        self.target_lane = lane_id

    def in_target_lane(self):
        wp = get_wp(self.world, self.npc)
        return wp is not None and wp.lane_id == self.target_lane

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
                a_wp = town_map.get_waypoint(
                    a_loc, project_to_road=True,
                    lane_type=carla.LaneType.Driving)
                if a_wp and a_wp.lane_id == self.target_lane and dot < closest_dist:
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

            MIN_FOLLOW_DIST = 4.0
            SAFE_FOLLOW_DIST = 10.0
            dist_a, leader = self._closest_vehicle_ahead(max_dist=30.0)
            if dist_a is not None:
                if dist_a < MIN_FOLLOW_DIST:
                    target_kmh = max(0.0, target_kmh * 0.2)
                elif dist_a < SAFE_FOLLOW_DIST:
                    ratio = (dist_a - MIN_FOLLOW_DIST) / (SAFE_FOLLOW_DIST - MIN_FOLLOW_DIST)
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
                to.x /= mag; to.y /= mag
            cross = fwd.x*to.y - fwd.y*to.x
            steer = max(-self.MAX_STEER, min(self.MAX_STEER, self.KP_STEER*cross))

            err = target_kmh - spd
            if target_kmh <= 0.0:
                th, br = 0.0, 0.8
            elif err > 0:
                th, br = min(1.0, err/20.0), 0.0
            else:
                th, br = 0.0, min(1.0, abs(err)/10.0)

            self.npc.apply_control(carla.VehicleControl(
                throttle=float(th), steer=float(steer), brake=float(br)))
        except Exception as e:
            print(f"[NPCCtrl] {e}")

# ── FIX #2 + #4: real queue with blocking + gap + cycling ───────────────────
class QueueVehicle:
    def __init__(self, world, npc, lane_id):
        self.world = world
        self.npc = npc
        self.lane_id = lane_id
        self.ctrl = NPCController(world, npc, lane_id)
        self.alive = True

    def update(self, target_speed):
        if not is_alive(self.npc):
            self.alive = False
            return
        self.ctrl.tick(target_speed)

    def destroy(self):
        safe_destroy(self.npc)
        self.alive = False


class HighwayQueue:
    """
    Manages the persistent congestion queue across lanes -1/-2/-3.
    Used both as the merge-blocking wall (100-160s) and as the ongoing
    congestion (220-440s). Vehicles that drift too far ahead/behind ego
    are respawned (FIX #4) instead of being abandoned.
    """
    def __init__(self, world, ego, lanes=(-1, -2, -3),
                 per_lane=12, spacing_m=12.0, start_offset_m=130.0,
                 model_pool=None):
        self.world = world
        self.ego = ego
        self.lanes = lanes
        self.per_lane = per_lane
        self.spacing_m = spacing_m
        self.start_offset_m = start_offset_m
        self.models = model_pool or [
            "vehicle.audi.tt", "vehicle.chevrolet.impala",
            "vehicle.ford.mustang", "vehicle.lincoln.mkz_2017",
            "vehicle.toyota.prius", "vehicle.dodge.charger_2020",
            "vehicle.mini.cooper_s", "vehicle.seat.leon",
        ]
        self.vehicles = []
        self.gap_lane_held = set()  # npc ids currently being held for a gap

    def spawn(self):
        total = 0
        for lane in self.lanes:
            npcs = walk_highway(self.world, lane, self.start_offset_m,
                                 self.spacing_m, self.per_lane,
                                 model=random.choice(self.models))
            for npc in npcs:
                self.vehicles.append(QueueVehicle(self.world, npc, lane))
            total += len(npcs)
            print(f"[Queue] Lane {lane}: {len(npcs)}/{self.per_lane} spawned")
        print(f"[Queue] Total: {total} vehicles")

    def update(self, speed_min=SPEED_QUEUE_MIN, speed_max=SPEED_QUEUE_MAX):
        for qv in self.vehicles:
            if not qv.alive:
                continue
            if qv.npc.id in self.gap_lane_held:
                continue  # held open for the gap window
            qv.update(random.uniform(speed_min, speed_max))

    def open_gap_in_lane(self, lane_id, t):
        """FIX #2: instead of one static blocker, pick the two real
        lane vehicles geometrically ahead of ego and stop them to create
        a checkable gap. Returns True if a gap was opened."""
        candidates = []
        for qv in self.vehicles:
            if qv.lane_id != lane_id or not qv.alive:
                continue
            d = dist_ahead(self.ego, qv.npc)
            if 0.0 < d < 60.0:
                candidates.append((d, qv))
        candidates.sort(key=lambda x: x[0])

        print(f"[Gap] T={t:.1f}s lane {lane_id} candidates ahead of ego: "
              f"{[(round(d,1), qv.npc.id) for d, qv in candidates]}")

        if not candidates:
            print(f"[Gap] WARNING: no lane {lane_id} vehicles ahead of ego "
                  f"within 60m — cannot open gap")
            return False

        chosen = candidates[:2]
        for d, qv in chosen:
            qv.update(GAP_STOP_SPEED)
            self.gap_lane_held.add(qv.npc.id)
            print(f"[Gap]   slowing npc {qv.npc.id} (dist_ahead={d:.1f}m)")
        return True

    def release_gap(self):
        self.gap_lane_held.clear()

    def cycle(self, max_ahead=180.0, max_behind=-60.0):
        """FIX #4: respawn any vehicle that has drifted out of useful
        range relative to ego, keeping the queue populated indefinitely."""
        for qv in list(self.vehicles):
            if not qv.alive:
                self.vehicles.remove(qv)
                continue
            d = dist_ahead(self.ego, qv.npc)
            if d > max_ahead or d < max_behind:
                lane = qv.lane_id
                safe_destroy(qv.npc)
                qv.alive = False
                self.vehicles.remove(qv)
                new_npc = spawn_on_highway(self.world, lane, self.start_offset_m,
                                            model=random.choice(self.models))
                if new_npc:
                    self.vehicles.append(QueueVehicle(self.world, new_npc, lane))

    def destroy_all(self):
        for qv in self.vehicles:
            qv.destroy()
        self.vehicles.clear()


# ── FIX #1: real merge detection ─────────────────────────────────────────────
def ego_has_merged(world, ego):
    """True once ego is on the actual highway road in a real driving lane.
    Replaces the old hardcoded 't >= 170' timer."""
    wp = get_wp(world, ego)
    if wp is None:
        return False
    return wp.road_id in HIGHWAY_ROAD_IDS and wp.lane_id in (-1, -2, -3)

def ego_lane(world, ego):
    wp = get_wp(world, ego)
    return wp.lane_id if wp else None


# ── Shoulder Vehicle (spawn geometry fixed) ──────────────────────────────────
class ShoulderVehicle:
    def __init__(self, world, ego, label, event_type):
        self.world = world
        self.ego = ego
        self.label = label
        self.event_type = event_type
        self.npc = None
        self.ctrl = None
        self.active = False
        self.has_cut_in = False

    def spawn(self, t):
        # FIX #3: anchored spawn, not ego-relative lane-walk
        self.npc = spawn_on_highway(self.world, -3, 20.0, model="vehicle.audi.a2")
        if self.npc:
            self.ctrl = NPCController(self.world, self.npc, -3)
            self.active = True
            print(f"[Shoulder {self.label}] Spawned at {t:.1f}s")
            self.ctrl.change_lane(-4)
            return True
        return False

    def update(self, t):
        if not self.active or not is_alive(self.npc):
            return False
        self.ctrl.tick(SPEED_SHOULDER)
        dist = dist_ahead(self.ego, self.npc)
        if self.event_type == 'cutin' and dist > 5.0 and not self.has_cut_in:
            self.ctrl.change_lane(-3)
            self.has_cut_in = True
            print(f"[Shoulder {self.label}] Cut in front of ego!")
        return True

    def destroy(self):
        safe_destroy(self.npc)
        self.active = False
        print(f"[Shoulder {self.label}] Destroyed")


# ── Blocking Vehicle (spawn geometry fixed) ──────────────────────────────────
class BlockingVehicle:
    def __init__(self, world, ego):
        self.world = world
        self.ego = ego
        self.npc = None
        self.ctrl = None
        self.active = False

    def spawn(self, t):
        self.npc = spawn_on_highway(self.world, -1, -30.0,
                                     model="vehicle.chevrolet.impala")
        if self.npc:
            self.ctrl = NPCController(self.world, self.npc, -1)
            self.active = True
            print(f"[Blocking] Spawned at {t:.1f}s in lane -1")
            try: self.npc.set_color(carla.Color(255, 165, 0))
            except: pass
            return True
        return False

    def update(self, t):
        if not self.active or not is_alive(self.npc):
            return False
        self.ctrl.tick(SPEED_BLOCKING)
        if is_alive(self.ego):
            loc = self.ego.get_transform().location
            self.world.debug.draw_string(
                loc + carla.Location(z=7.5),
                "Blocking vehicle ahead: 60 km/h",
                draw_shadow=True, color=COLOR_DANGER,
                life_time=FDS*2, persistent_lines=False)
        return True

    def destroy(self):
        safe_destroy(self.npc)
        self.active = False
        print("[Blocking] Destroyed")


# ── Horn System ──────────────────────────────────────────────────────────────
class HornSystem:
    def __init__(self, world, ego):
        self.world = world
        self.ego = ego
        self.vehicles_behind = []
        self.last_horn_time = 0
        self.horn_interval = 5.0
        self.active = False

    def add_vehicle_behind(self):
        npc = spawn_on_highway(self.world, -2, -10.0, model="vehicle.audi.a2")
        if npc:
            self.vehicles_behind.append(npc)
            print("[Horn] Vehicle behind added")
        return npc

    def update(self, t, ego_speed):
        if not self.active:
            return
        if not self.vehicles_behind:
            self.add_vehicle_behind()
            return
        if t >= 120.0 and t - self.last_horn_time >= self.horn_interval:
            for vehicle in self.vehicles_behind:
                if is_alive(vehicle):
                    try:
                        vehicle.horn()
                        print(f"[HORN] Honking at {t:.1f}s")
                    except: pass
            self.last_horn_time = t
        for vehicle in self.vehicles_behind:
            if is_alive(vehicle):
                NPCController(self.world, vehicle, -2).tick(max(ego_speed - 5, 20.0))

    def cleanup(self):
        for vehicle in self.vehicles_behind:
            safe_destroy(vehicle)
        self.vehicles_behind.clear()


# ── Main Scenario Class ──────────────────────────────────────────────────────
class Scenario2:
    def __init__(self):
        self.client = None
        self.world = None
        self.ego = None
        self.voice = None
        self.all_npcs = []
        self.running = True

        self.timings = {
            'ADAPTATION_END': 100.0,
            'BLOCKED_END': 160.0,
            'GAP_END': 180.0,
            'STABILIZATION_END': 220.0,
            'CONGESTION_END': 440.0,
            'RECOVERY_END': 460.0,
            'SCENARIO_END': 800.0
        }

        self.current_phase = ScenarioPhase.ADAPTATION
        self.merged = False
        self.gap_opened = False
        self.recovery_started = False

        self.merge_queue = None        # HighwayQueue used 100-220s
        self.congestion_queue = None   # HighwayQueue used 220-440s
        self.shoulder_vehicles = []
        self.blocking_vehicle = None
        self.horn_system = None

        self.rating_schedule = {
            50.0: "How much anger or frustration do you feel due to the current traffic condition?",
            185.0: "How much anger or frustration did you feel due to the merging refusal situation?",
            270.0: "How much anger or frustration did you feel due to the vehicle merging from the shoulder?",
            335.0: "How much anger or frustration did you feel due to vehicles driving on the shoulder?",
            390.0: "How much anger or frustration did you feel due to the vehicle merging from the shoulder?",
            445.0: "How much anger or frustration did you feel due to vehicles driving on the shoulder?",
            745.0: "How much anger or frustration did you feel due to the slow vehicle blocking your lane?"
        }
        self.prompted = set()

    def connect(self):
        print("=" * 60)
        print("SCENARIO 2 (fixed): Merge Block + Congestion + Shoulder + Blocking")
        print("=" * 60)
        try:
            self.client, self.world = connect_and_load(CARLA_HOST, CARLA_PORT)
            print("Connected to CARLA")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def setup_ego(self):
        try:
            self.ego = find_ego_vehicle(self.world)
            if not self.ego:
                print("Failed to find/spawn ego vehicle")
                return False
            precise_respawn(self.ego, RAMP_SPAWN)
            self.ego.set_autopilot(False)
            print("Ego positioned on ramp")
            return True
        except Exception as e:
            print(f"Error setting up ego: {e}")
            return False

    def setup_voice(self):
        self.voice = VoiceSystem()
        return self.voice.initialize()

    def display_countdown(self):
        if not is_alive(self.ego): return
        loc = self.ego.get_transform().location
        for number in [3, 2, 1]:
            self.world.debug.draw_string(
                loc + carla.Location(z=3.0), f"  {number}  ",
                draw_shadow=True, color=COLOR_COUNTDOWN,
                life_time=1.5, persistent_lines=False, text_size=3.0)
            if self.voice:
                self.voice._play_beep(self.world, loc)
                time.sleep(0.2)
                self.voice.speak(str(number), wait=False)
            time.sleep(1.0)
        self.world.debug.draw_string(
            loc + carla.Location(z=3.0), "  GO!  ",
            draw_shadow=True, color=carla.Color(0, 255, 0),
            life_time=2.0, persistent_lines=False, text_size=3.0)
        if self.voice:
            self.voice.speak("GO!", wait=False)

    def show_once(self, attr, t, trigger_t, msg, z=2.5, speak=None, color=None):
        if not is_alive(self.ego): return
        if t >= trigger_t and not getattr(self, attr, False):
            loc = self.ego.get_transform().location
            self.world.debug.draw_string(
                loc + carla.Location(z=z), msg, draw_shadow=True,
                color=color or carla.Color(0, 255, 255),
                life_time=10.0, persistent_lines=False, text_size=1.0)
            if self.voice and speak:
                self.voice.speak(speak, wait=False)
            setattr(self, attr, True)
            print(f"[MESSAGE] {msg}")

    def show_rating(self, msg, t):
        if not is_alive(self.ego): return
        loc = self.ego.get_transform().location
        self.world.debug.draw_string(
            loc + carla.Location(z=8.0), msg, draw_shadow=True,
            color=COLOR_RATING, life_time=15.0, persistent_lines=False)
        print(f"\n[RATING] {msg}\n")
        if self.voice:
            self.voice._play_beep(self.world, loc)
            time.sleep(0.3)
            self.voice.speak(msg, wait=False)

    # ── Phase Logic ──────────────────────────────────────────────────────────

    def update_adaptation(self, t):
        if is_alive(self.ego):
            self.ego.apply_control(carla.VehicleControl(throttle=0.4))

    def update_merge_blocked(self, t):
        """100-160s: real congestion queue blocks all three lanes."""
        if self.merge_queue is None:
            self.merge_queue = HighwayQueue(self.world, self.ego,
                                             lanes=(-1, -2, -3),
                                             per_lane=14, spacing_m=12.0,
                                             start_offset_m=130.0)
            self.merge_queue.spawn()
            self.all_npcs.extend(qv.npc for qv in self.merge_queue.vehicles)

        if not self.horn_system and t >= 120.0:
            self.horn_system = HornSystem(self.world, self.ego)
            self.horn_system.active = True
        if self.horn_system:
            self.horn_system.update(t, kmh_of(self.ego))

        self.merge_queue.update()
        self.merge_queue.cycle()

        if ego_has_merged(self.world, self.ego) and not self.merged:
            self.merged = True
            print(f"[Merge] Ego merged early at {t:.1f}s, lane="
                  f"{ego_lane(self.world, self.ego)}")

    def update_gap_window(self, t):
        """160-180s: open a real gap in lane -3."""
        if self.merge_queue is None:
            self.update_merge_blocked(t)
            return

        if not self.gap_opened:
            opened = self.merge_queue.open_gap_in_lane(-3, t)
            self.gap_opened = True
            if not opened:
                print("[Gap] No candidates found — gap window will retry "
                      "every tick until merge or window ends")

        if not ego_has_merged(self.world, self.ego):
            # keep retrying in case the queue moved since the first attempt
            self.merge_queue.open_gap_in_lane(-3, t)

        self.merge_queue.update()
        self.merge_queue.cycle()

        if self.horn_system:
            self.horn_system.update(t, kmh_of(self.ego))

        if ego_has_merged(self.world, self.ego) and not self.merged:
            self.merged = True
            print(f"[Merge] Ego merged at {t:.1f}s, lane="
                  f"{ego_lane(self.world, self.ego)}")

    def update_stabilization(self, t):
        if self.merge_queue:
            self.merge_queue.release_gap()
            self.merge_queue.destroy_all()
            self.merge_queue = None
        if self.horn_system:
            self.horn_system.cleanup()
            self.horn_system = None
        if is_alive(self.ego):
            self.ego.apply_control(carla.VehicleControl(throttle=0.3))

    def spawn_shoulder_schedule(self):
        if self.shoulder_vehicles:
            return
        for label, spawn_time, event_type in [
            ('B1', 240.0, 'pass'), ('B2', 300.0, 'cutin'),
            ('B3', 360.0, 'pass'), ('B4', 400.0, 'cutin')
        ]:
            sv = ShoulderVehicle(self.world, self.ego, label, event_type)
            sv.scheduled_time = spawn_time
            self.shoulder_vehicles.append(sv)
            print(f"[Shoulder] Scheduled {label} at {spawn_time}s ({event_type})")

    def update_congestion(self, t):
        if self.congestion_queue is None:
            self.congestion_queue = HighwayQueue(self.world, self.ego,
                                                  lanes=(-1, -2, -3),
                                                  per_lane=12, spacing_m=15.0,
                                                  start_offset_m=100.0)
            self.congestion_queue.spawn()
            self.all_npcs.extend(qv.npc for qv in self.congestion_queue.vehicles)
            self.spawn_shoulder_schedule()

        self.congestion_queue.update(SPEED_CONGESTION_MIN, SPEED_CONGESTION_MAX)
        self.congestion_queue.cycle()  # FIX #4: keeps lanes populated for full 220s

        for sv in self.shoulder_vehicles:
            if not sv.active and t >= sv.scheduled_time:
                sv.spawn(t)
                self.all_npcs.append(sv.npc)
            elif sv.active:
                sv.update(t)

        if is_alive(self.ego):
            ego_speed = kmh_of(self.ego)
            if ego_speed > SPEED_CONGESTION_MAX + 10:
                self.ego.apply_control(carla.VehicleControl(brake=0.3))
            elif ego_speed < SPEED_CONGESTION_MIN - 5:
                self.ego.apply_control(carla.VehicleControl(throttle=0.3))

    def update_recovery(self, t):
        if not self.recovery_started:
            print(f"[Recovery] Starting at {t:.1f}s")
            self.recovery_started = True
            if self.congestion_queue:
                self.congestion_queue.destroy_all()
                self.congestion_queue = None
            for sv in self.shoulder_vehicles:
                sv.destroy()
            self.shoulder_vehicles.clear()

        progress = (t - 440.0) / 20.0
        recovery_speed = SPEED_CONGESTION_MAX + progress * (SPEED_CRUISE - SPEED_CONGESTION_MAX)
        if is_alive(self.ego):
            ego_speed = kmh_of(self.ego)
            if ego_speed < recovery_speed:
                self.ego.apply_control(carla.VehicleControl(
                    throttle=min(1.0, (recovery_speed - ego_speed) / 20.0)))
            loc = self.ego.get_transform().location
            self.world.debug.draw_string(
                loc + carla.Location(z=7.5), f"RECOVERY: {recovery_speed:.0f} km/h",
                draw_shadow=True, color=carla.Color(0, 255, 255),
                life_time=FDS*2, persistent_lines=False)

    def update_blocking(self, t):
        if not self.blocking_vehicle:
            self.blocking_vehicle = BlockingVehicle(self.world, self.ego)
            if self.blocking_vehicle.spawn(t):
                self.all_npcs.append(self.blocking_vehicle.npc)

        if self.blocking_vehicle:
            self.blocking_vehicle.update(t)

        if is_alive(self.ego):
            ego_speed = kmh_of(self.ego)
            wp = get_wp(self.world, self.ego)
            if wp and wp.lane_id == -1:
                if ego_speed > SPEED_BLOCKING + 5:
                    self.ego.apply_control(carla.VehicleControl(brake=0.2))
                elif ego_speed < SPEED_BLOCKING - 5:
                    self.ego.apply_control(carla.VehicleControl(throttle=0.3))

    # ── HUD ──────────────────────────────────────────────────────────────────
    def update_hud(self, t, ego_speed):
        if not is_alive(self.ego): return
        loc = self.ego.get_transform().location
        phase_colors = {
            ScenarioPhase.ADAPTATION: COLOR_INFO,
            ScenarioPhase.MERGE_BLOCKED: COLOR_DANGER,
            ScenarioPhase.GAP_WINDOW: COLOR_WARN,
            ScenarioPhase.STABILIZATION: COLOR_OK,
            ScenarioPhase.CONGESTION: COLOR_DANGER,
            ScenarioPhase.RECOVERY: COLOR_INFO,
            ScenarioPhase.BLOCKING: COLOR_WARN
        }
        color = phase_colors.get(self.current_phase, COLOR_INFO)
        self.world.debug.draw_string(
            loc + carla.Location(z=5.0), f"Speed: {ego_speed:.1f} km/h   T+{t:.0f}s",
            draw_shadow=True, color=color, life_time=FDS*2, persistent_lines=False)
        self.world.debug.draw_string(
            loc + carla.Location(z=6.5), f"[{self.current_phase.value}]",
            draw_shadow=True, color=COLOR_INFO, life_time=FDS*2, persistent_lines=False)
        if self.current_phase == ScenarioPhase.CONGESTION:
            if any(sv.active for sv in self.shoulder_vehicles):
                self.world.debug.draw_string(
                    loc + carla.Location(z=8.0), "Shoulder Violation Detected",
                    draw_shadow=True, color=COLOR_SHOULDER,
                    life_time=FDS*2, persistent_lines=False)
        if self.blocking_vehicle and self.blocking_vehicle.active:
            self.world.debug.draw_string(
                loc + carla.Location(z=8.5), "Blocking Vehicle: 60 km/h",
                draw_shadow=True, color=COLOR_DANGER,
                life_time=FDS*2, persistent_lines=False)

    # ── Cleanup ──────────────────────────────────────────────────────────────
    def cleanup(self):
        print("\n--- Cleaning Up ---")
        if self.merge_queue: self.merge_queue.destroy_all()
        if self.congestion_queue: self.congestion_queue.destroy_all()
        for sv in self.shoulder_vehicles: sv.destroy()
        self.shoulder_vehicles.clear()
        if self.blocking_vehicle: self.blocking_vehicle.destroy()
        if self.horn_system: self.horn_system.cleanup()
        for npc in self.all_npcs: safe_destroy(npc)
        self.all_npcs.clear()
        if self.ego: safe_destroy(self.ego)
        print("Cleanup complete")

    # ── Main Run Loop ────────────────────────────────────────────────────────
    def run(self):
        if not self.connect(): return
        if not self.setup_ego(): return
        self.setup_voice()

        print("\nSCENARIO STARTING IN 3...")
        # self.display_countdown()

        self.world.tick()
        self.start_time = self.world.get_snapshot().timestamp.elapsed_seconds

        if self.voice:
            self.voice.speak(
                "Scenario 2 starting. Drive on the ramp, then merge onto "
                "the highway after one hundred seconds.", wait=False)

        tick_count = 0
        status_every = int(2.0 / FDS)

        print("\n[Scenario 2] Running. Ctrl+C to stop.\n")
        print("Timeline:")
        print("  0-100s:    Adaptation")
        print("  100-160s:  Merge blocked (real queue, no gap)")
        print("  160-180s:  Gap window — merge here")
        print("  180-220s:  Stabilization")
        print("  220-440s:  Congestion + shoulder events")
        print("  440-460s:  Recovery")
        print("  460+s:     Blocking vehicle\n")

        try:
            while self.running:
                self.world.tick()
                if not is_alive(self.ego):
                    print("Ego lost.")
                    break

                current_time = (self.world.get_snapshot().timestamp.elapsed_seconds
                                 - self.start_time)
                ego_speed = kmh_of(self.ego)

                if current_time < self.timings['ADAPTATION_END']:
                    self.current_phase = ScenarioPhase.ADAPTATION
                    self.update_adaptation(current_time)
                elif current_time < self.timings['BLOCKED_END']:
                    self.current_phase = ScenarioPhase.MERGE_BLOCKED
                    self.update_merge_blocked(current_time)
                elif current_time < self.timings['GAP_END']:
                    self.current_phase = ScenarioPhase.GAP_WINDOW
                    self.update_gap_window(current_time)
                elif current_time < self.timings['STABILIZATION_END']:
                    self.current_phase = ScenarioPhase.STABILIZATION
                    self.update_stabilization(current_time)
                elif current_time < self.timings['CONGESTION_END']:
                    self.current_phase = ScenarioPhase.CONGESTION
                    self.update_congestion(current_time)
                elif current_time < self.timings['RECOVERY_END']:
                    self.current_phase = ScenarioPhase.RECOVERY
                    self.update_recovery(current_time)
                else:
                    self.current_phase = ScenarioPhase.BLOCKING
                    self.update_blocking(current_time)

                # self.show_once('start_message_shown', current_time, 2.0,
                #                 "Drive on ramp - Merge at 100s")
                # self.show_once('merge_instruction_shown', current_time, 65.0,
                #                 "Turn on the left signal and merge into the main lane",
                #                 z=6.5, speak="Turn on left signal and merge into the main lane",
                #                 color=carla.Color(255, 255, 0))
                # self.show_once('adaptation_message_shown', current_time, 220.0,
                #                 "Maintain safe distance and follow traffic speed",
                #                 z=6.5, speak="Maintain a safe distance and follow the traffic speed")

                for prompt_time, msg in self.rating_schedule.items():
                    if current_time >= prompt_time and prompt_time not in self.prompted:
                        self.show_rating(msg, current_time)
                        self.prompted.add(prompt_time)

                self.update_hud(current_time, ego_speed)

                tick_count += 1
                if tick_count % status_every == 0:
                    shoulder_count = sum(1 for sv in self.shoulder_vehicles if sv.active)
                    qcount = (len(self.merge_queue.vehicles) if self.merge_queue else
                              len(self.congestion_queue.vehicles) if self.congestion_queue else 0)
                    print(f"  T={current_time:>6.1f}s  speed={ego_speed:>5.1f} km/h  "
                          f"phase={self.current_phase.value[:14]}  lane={ego_lane(self.world, self.ego)}  "
                          f"shoulder={shoulder_count}  queue={qcount}")

                if current_time >= self.timings['SCENARIO_END']:
                    print("\nSCENARIO 2 COMPLETE")
                    if self.voice:
                        self.voice.speak("Scenario 2 complete. Thank you for participating.",
                                          wait=False)
                    break

                time.sleep(0.001)

        except KeyboardInterrupt:
            print("\n[Scenario 2] Stopped by user.")
        finally:
            self.cleanup()


# ── Connect / Ego helpers ─────────────────────────────────────────────────────
def connect_and_load(host, port, map_name="Town04"):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.load_world(map_name)
    world.set_weather(carla.WeatherParameters(
        cloudiness=10, precipitation=0, sun_altitude_angle=45,
        sun_azimuth_angle=200, fog_density=0, wetness=0))
    return client, world

def find_ego_vehicle(world):
    v = list(world.get_actors().filter("harplab.dreyevr_vehicle.*"))
    if v: return v[0]
    bp = world.get_blueprint_library().find("harplab.dreyevr_vehicle.model3")
    spawn_points = world.get_map().get_spawn_points()
    for sp in spawn_points:
        if 50 < sp.location.x < 150:
            return world.spawn_actor(bp, sp)
    return world.spawn_actor(bp, spawn_points[0])

def precise_respawn(vehicle, location, yaw=None):
    try:
        if not isinstance(location, carla.Location):
            location = carla.Location(*location)
        if yaw is None:
            wp = vehicle.get_world().get_map().get_waypoint(location, project_to_road=True)
            yaw = wp.transform.rotation.yaw if wp else 0.0
        vehicle.set_simulate_physics(False)
        vehicle.set_transform(carla.Transform(location, carla.Rotation(yaw=float(yaw))))
        time.sleep(0.1)
        vehicle.set_simulate_physics(True)
        return True
    except Exception as e:
        print(f"Respawn failed: {e}")
        return False


if __name__ == "__main__":
    print("\nCARLA SCENARIO 2 (fixed)")
    print("Make sure CARLA is running, then press Enter to start...")
    input()
    Scenario2().run()