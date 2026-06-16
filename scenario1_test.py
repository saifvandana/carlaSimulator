"""
Scenario 1 — Full Script
========================
Lane mapping (Road 36, negative direction):
  Lane -1 = Lane 1 (leftmost / fast lane)
  Lane -2 = Lane 2 (EGO lane for A1-A4)
  Lane -3 = Lane 3 (rightmost / NPC lane for A1-A4, EGO lane for congestion)

A1-A4  (0-500s):   NPCs spawn in lane -3, cut LEFT into lane -2 (ego lane)
Congestion (500s+): Ego moves to lane -3. Queue in lane -3 ahead of ego.
                    Cut-ins go from lane -2 RIGHT into lane -3 (ego's new lane)
"""

import carla, time, math, random
from enum import Enum

CARLA_HOST  = "localhost"
CARLA_PORT  = 2000
FDS         = 0.05
TM_PORT     = 8001
ROAD_LIMIT  = 120.0

SPEED_CRUISE    = 100.0
SPEED_CATCHUP   = 110.0
SPEED_SLOW      = 70.0
SPEED_CONG_MIN  = 5.0
SPEED_CONG_MAX  = 10.0
CUT_AHEAD_M     = 10.0
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
    GRADUAL = 6.0

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
    """Get ego's current waypoint."""
    return world.get_map().get_waypoint(
        ego.get_transform().location, project_to_road=True,
        lane_type=carla.LaneType.Driving)


def wp_in_lane(wp, target_lane_id):
    """
    Walk from wp to target_lane_id using get_left_lane / get_right_lane.
    Returns waypoint in target lane or None.
    In CARLA negative lanes: -1 is leftmost, -3 is rightmost.
    get_left_lane()  from -2 → -1 (more left, lower abs value)
    get_right_lane() from -2 → -3 (more right, higher abs value)
    """
    cur = wp
    for _ in range(6):
        if cur is None: return None
        if cur.lane_id == target_lane_id: return cur
        # Decide direction: negative lanes, higher abs = more right
        if abs(target_lane_id) > abs(cur.lane_id):
            nxt = cur.get_right_lane()   # going right (e.g. -2 → -3)
        else:
            nxt = cur.get_left_lane()    # going left  (e.g. -3 → -2)
        if nxt and nxt.lane_type == carla.LaneType.Driving:
            cur = nxt
        else:
            break
    return cur if cur and cur.lane_id == target_lane_id else None


def spawn_at_wp(world, wp, model=None):
    """Spawn vehicle at waypoint. Returns actor or None."""
    blib = world.get_blueprint_library()
    bp   = None
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


def spawn_behind_in_lane(world, ego, lane_id, behind_m, model=None):
    """
    Spawn vehicle behind ego by behind_m metres in lane_id.
    Returns actor or None.
    """
    ego_wp = get_ego_wp(world, ego)
    if not ego_wp: return None
    # Go to ego's lane first then walk to target lane
    ego_lane_wp = wp_in_lane(ego_wp, lane_id)
    if not ego_lane_wp: return None
    prev = ego_lane_wp.previous(behind_m)
    if not prev: return None
    spawn_wp = prev[0]
    # Ensure we are in target lane after going back
    spawn_wp = wp_in_lane(spawn_wp, lane_id) or spawn_wp
    npc = spawn_at_wp(world, spawn_wp, model)
    if npc:
        npc.set_autopilot(False)
        npc.set_simulate_physics(True)
        print(f"[Spawn] id={npc.id}  lane={spawn_wp.lane_id}  {behind_m:.0f}m behind")
    return npc


def spawn_ahead_in_lane(world, ego, lane_id, ahead_m, model=None):
    """
    Spawn vehicle ahead of ego by ahead_m metres in lane_id.
    Returns actor or None.
    """
    ego_wp = get_ego_wp(world, ego)
    if not ego_wp: return None
    ego_lane_wp = wp_in_lane(ego_wp, lane_id)
    if not ego_lane_wp: return None
    nexts = ego_lane_wp.next(ahead_m)
    if not nexts: return None
    spawn_wp = nexts[0]
    spawn_wp = wp_in_lane(spawn_wp, lane_id) or spawn_wp
    npc = spawn_at_wp(world, spawn_wp, model)
    if npc:
        npc.set_autopilot(False)
        npc.set_simulate_physics(True)
        print(f"[Spawn] id={npc.id}  lane={spawn_wp.lane_id}  {ahead_m:.0f}m ahead")
    return npc


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


# ── Indicator ─────────────────────────────────────────────────────────────────
def show_indicator(world, npc, duration_s=2.5):
    if not is_alive(npc): return
    try:
        tf  = npc.get_transform(); fwd = tf.get_forward_vector()
        left = carla.Location(x=tf.location.x + fwd.y*3.5,
                               y=tf.location.y - fwd.x*3.5,
                               z=tf.location.z + 1.5)
        world.debug.draw_arrow(
            tf.location + carla.Location(z=1.0), left,
            thickness=0.15, arrow_size=0.3,
            color=carla.Color(255, 165, 0),
            life_time=duration_s, persistent_lines=False)
    except: pass


# ── NPC Controller ────────────────────────────────────────────────────────────
class NPCController:
    """Physics-ON, apply_control each tick. Steers toward target_lane waypoint."""
    LOOKAHEAD = 8.0; KP_STEER = 1.2; MAX_STEER = 0.6

    def __init__(self, world, npc, target_lane):
        self.world=world; self.npc=npc; self.target_lane=target_lane

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

    def tick(self, target_kmh):
        if not is_alive(self.npc): return
        try:
            npc_tf = self.npc.get_transform()
            loc    = npc_tf.location
            fwd    = npc_tf.get_forward_vector()
            look   = carla.Location(x=loc.x+fwd.x*self.LOOKAHEAD,
                                    y=loc.y+fwd.y*self.LOOKAHEAD, z=loc.z)
            town_map = self.world.get_map()
            wp = town_map.get_waypoint(look, project_to_road=True,
                                        lane_type=carla.LaneType.Driving)
            if not wp:
                wp = town_map.get_waypoint(loc, project_to_road=True,
                                            lane_type=carla.LaneType.Driving)
            if not wp: return
            tgt = wp_in_lane(wp, self.target_lane) or wp
            to  = carla.Vector3D(tgt.transform.location.x-loc.x,
                                  tgt.transform.location.y-loc.y, 0.0)
            mag = math.sqrt(to.x**2+to.y**2)
            if mag>0.001: to.x/=mag; to.y/=mag
            cross = fwd.x*to.y - fwd.y*to.x
            steer = max(-self.MAX_STEER, min(self.MAX_STEER, self.KP_STEER*cross))
            spd   = kmh_of(self.npc); err = target_kmh-spd
            if err>0: th=min(1.0,err/20.0); br=0.0
            else:     th=0.0; br=min(1.0,abs(err)/10.0)
            self.npc.apply_control(carla.VehicleControl(
                throttle=float(th), steer=float(steer),
                brake=float(br), hand_brake=False))
        except Exception as e:
            print(f"[NPCCtrl] {e}")


# ── A-Event (A1-A4): lane -3 → lane -2 ───────────────────────────────────────
class AEvent:
    """
    NPC spawns in lane -3 (right of ego), 8m behind.
    Tails ego → overtakes → cuts LEFT into lane -2 (ego lane) → decelerates.
    """
    def __init__(self, label, indicator, decel_type, start_t, model):
        self.label     = label
        self.indicator = indicator
        self.decel_type= decel_type
        self.start_t   = start_t
        self.model     = model
        self.npc       = None
        self.ctrl      = None
        self.state     = "waiting"
        self._state_t  = 0.0
        self._ind_done = False
        self._spawn_t  = start_t - SPAWN_BEFORE_S
        print(f"[{label}] lane-3→lane-2  ind={'ON' if indicator else 'OFF'}  "
              f"decel={decel_type.name}  spawn@{self._spawn_t:.0f}s")

    def _set_state(self, s, t):
        print(f"[{self.label}] {self.state} -> {s}  T={t:.1f}s")
        self.state=s; self._state_t=t

    def update(self, world, ego, t):
        try: return self._update(world, ego, t)
        except Exception as e:
            print(f"[{self.label}] err: {e}"); return ""

    def _update(self, world, ego, t):
        ego_spd = kmh_of(ego) if is_alive(ego) else SPEED_CRUISE

        if self.state == "waiting":
            if t >= self._spawn_t:
                # Spawn in lane -3 (right of ego), 8m behind
                npc = spawn_behind_in_lane(world, ego, lane_id=-3,
                                           behind_m=8.0, model=self.model)
                if npc:
                    self.npc  = npc
                    self.ctrl = NPCController(world, npc, target_lane=-3)
                    self._set_state("tailing", t)
                else:
                    print(f"[{self.label}] spawn failed — retry next tick")
            return ""

        if not is_alive(self.npc): return f"{self.label} (gone)"
        ahead = dist_ahead(ego, self.npc)

        if self.state == "tailing":
            self.ctrl.tick(max(ego_spd, 20.0))
            if t >= self.start_t: self._set_state("overtake", t)
            return f"{self.label} — tailing lane-3  {kmh_of(self.npc):.0f} km/h"

        elif self.state == "overtake":
            ramp = min((t-self._state_t)/5.0, 1.0)
            spd  = ego_spd + ramp*(SPEED_CATCHUP-ego_spd)
            self.ctrl.tick(spd)
            if ahead >= CUT_AHEAD_M:
                if not self._ind_done:
                    if self.indicator: show_indicator(world, self.npc, 2.5)
                    self._ind_done = True
                # Cut LEFT: lane -3 → lane -2
                self.ctrl.change_lane(-2)
                self._set_state("cutin", t)
            return (f"{self.label} — overtaking  "
                    f"{kmh_of(self.npc):.0f} km/h  ahead={ahead:.0f}m")

        elif self.state == "cutin":
            self.ctrl.tick(SPEED_CRUISE)
            if self.ctrl.in_target_lane(): self._set_state("decel", t)
            return f"{self.label} — cutting in lane-3→lane-2"

        elif self.state == "decel":
            progress = min((t-self._state_t)/self.decel_type.value, 1.0)
            spd = SPEED_CRUISE + progress*(SPEED_SLOW-SPEED_CRUISE)
            self.ctrl.tick(spd)
            if progress >= 1.0: self._set_state("tail_ego", t)
            return (f"{self.label} — decel ({self.decel_type.name})  "
                    f"{kmh_of(self.npc):.0f} km/h")

        elif self.state == "tail_ego":
            self.ctrl.tick(SPEED_SLOW)
            if t-self._state_t >= 5.0: self._set_state("reaccel", t)
            return f"{self.label} — tail  {kmh_of(self.npc):.0f} km/h"

        elif self.state == "reaccel":
            progress = min((t-self._state_t)/20.0, 1.0)
            spd = SPEED_SLOW + progress*(SPEED_CRUISE-SPEED_SLOW)
            self.ctrl.tick(spd)
            if progress >= 1.0: self._set_state("done", t)
            return f"{self.label} — reaccel  {kmh_of(self.npc):.0f} km/h"

        elif self.state == "done":
            self.ctrl.tick(SPEED_CRUISE)
            if ahead > DESTROY_M or ahead < -50.0:
                safe_destroy(self.npc); self.npc=None
                return f"{self.label} — done"
            return f"{self.label} — cruising"

        return ""


# ── Congestion cut-in: lane -2 → lane -3 (ego's congestion lane) ─────────────
class CongestionCutIn:
    """
    Ego is in lane -3 (congestion queue).
    NPC spawns in lane -2 (flowing), overtakes ego,
    then cuts RIGHT into lane -3 (ego's lane) and brakes.
    solid_line=True = no indicator.
    """
    def __init__(self, label, trigger_t, solid_line=False):
        self.label      = label
        self.trigger_t  = trigger_t
        self.solid_line = solid_line
        self.npc        = None
        self.ctrl       = None
        self.state      = "waiting"
        self._state_t   = 0.0
        self._ind_done  = False
        print(f"[{label}] lane-2→lane-3  @ {trigger_t}s  "
              f"solid={'YES' if solid_line else 'no'}")

    def _set_state(self, s, t):
        print(f"[{self.label}] {self.state} -> {s}  T={t:.1f}s")
        self.state=s; self._state_t=t

    def update(self, world, ego, t):
        try: return self._update(world, ego, t)
        except Exception as e:
            print(f"[{self.label}] err: {e}"); return ""

    def _update(self, world, ego, t):
        if self.state == "waiting":
            if t >= self.trigger_t:
                # Spawn in lane -2 (flowing), 15m behind ego
                npc = spawn_behind_in_lane(world, ego, lane_id=-2,
                                           behind_m=15.0, model="vehicle.audi.a2")
                if npc:
                    self.npc  = npc
                    self.ctrl = NPCController(world, npc, target_lane=-2)
                    self._set_state("overtake", t)
            return ""

        if not is_alive(self.npc): return f"{self.label} (gone)"
        ahead = dist_ahead(ego, self.npc)

        if self.state == "overtake":
            # NPC in lane -2, drive at cruise speed (flowing traffic)
            ramp = min((t-self._state_t)/5.0, 1.0)
            spd  = SPEED_CONG_MAX + ramp*(SPEED_CRUISE-SPEED_CONG_MAX)
            self.ctrl.tick(spd)
            if ahead >= CUT_AHEAD_M:
                if not self._ind_done:
                    # Dashed = indicator ON (right blinker for lane -2→-3)
                    if not self.solid_line:
                        show_indicator(world, self.npc, 2.5)
                        print(f"[{self.label}] Indicator ON (dashed line)")
                    else:
                        print(f"[{self.label}] No indicator (solid line violation)")
                    self._ind_done = True
                # Cut RIGHT: lane -2 → lane -3
                self.ctrl.change_lane(-3)
                self._set_state("cutin", t)
            return f"{self.label} — lane-2 flowing  ahead={ahead:.0f}m"

        elif self.state == "cutin":
            self.ctrl.tick(SPEED_CRUISE)
            if self.ctrl.in_target_lane(): self._set_state("decel", t)
            return f"{self.label} — cutting in lane-2→lane-3"

        elif self.state == "decel":
            progress = min((t-self._state_t)/3.0, 1.0)
            spd = SPEED_CRUISE + progress*(SPEED_SLOW-SPEED_CRUISE)
            self.ctrl.tick(spd)
            if progress >= 1.0: self._set_state("cruise", t)
            return f"{self.label} — decel  {kmh_of(self.npc):.0f} km/h"

        elif self.state == "cruise":
            self.ctrl.tick(SPEED_SLOW)
            if ahead > DESTROY_M or ahead < -50.0:
                safe_destroy(self.npc); self.npc=None
                return f"{self.label} — done"
            return f"{self.label} — cruising  {kmh_of(self.npc):.0f} km/h"

        return ""


# ── Exit cut-in (810-890s) ────────────────────────────────────────────────────
class ExitCutIn:
    """Sudden cut-in without indicator + hard brake near exit (810-890s)."""
    def __init__(self):
        self.npc=None; self.ctrl=None
        self.state="waiting"; self._state_t=0.0
        print("[Exit] Ready @ 810s")

    def _set_state(self, s, t):
        print(f"[Exit] {self.state} -> {s}  T={t:.1f}s")
        self.state=s; self._state_t=t

    def update(self, world, ego, t):
        try: return self._update(world, ego, t)
        except Exception as e:
            print(f"[Exit] err: {e}"); return ""

    def _update(self, world, ego, t):
        ego_spd = kmh_of(ego) if is_alive(ego) else SPEED_CRUISE

        if self.state == "waiting":
            if t >= 810.0:
                # Spawn in lane -3 (ego's exit lane), 8m behind
                npc = spawn_behind_in_lane(world, ego, lane_id=-3,
                                           behind_m=8.0,
                                           model="vehicle.mercedes.coupe")
                if npc:
                    self.npc  = npc
                    self.ctrl = NPCController(world, npc, -3)
                    self._set_state("overtake", t)
            return ""

        if not is_alive(self.npc): return "Exit gone"
        ahead = dist_ahead(ego, self.npc)

        if self.state == "overtake":
            ramp = min((t-self._state_t)/4.0, 1.0)
            spd  = ego_spd + ramp*(SPEED_CATCHUP-ego_spd)
            self.ctrl.tick(spd)
            if ahead >= CUT_AHEAD_M:
                print("[Exit] Sudden cut-in — NO indicator!")
                # Cut LEFT into ego's lane (-3 → -2 near exit)
                self.ctrl.change_lane(-2)
                self._set_state("cutin", t)
            return f"Exit — overtaking  ahead={ahead:.0f}m"

        elif self.state == "cutin":
            self.ctrl.tick(SPEED_CRUISE)
            if self.ctrl.in_target_lane(): self._set_state("hard_brake", t)
            return "Exit — cutting in"

        elif self.state == "hard_brake":
            elapsed = t - self._state_t
            if elapsed < 4.0:
                self.npc.apply_control(carla.VehicleControl(
                    throttle=0.0, brake=0.9, hand_brake=False))
                return f"Exit — HARD BRAKE  {kmh_of(self.npc):.0f} km/h"
            self._set_state("cruise", t)
            return "Exit — braking"

        elif self.state == "cruise":
            self.ctrl.tick(40.0)
            if ahead > DESTROY_M or ahead < -50.0:
                safe_destroy(self.npc); self.npc=None
                return "Exit — done"
            return f"Exit — slow cruise  {kmh_of(self.npc):.0f} km/h"

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
    def __init__(self, world, ego): self.world=world; self.ego=ego
    def update(self, spd, warning, t, label=""):
        if not is_alive(self.ego): return
        try: loc=self.ego.get_transform().location
        except: return
        col=(COLOR_OK if SPEED_LOW<=spd<=SPEED_HIGH else
             COLOR_DANGER if spd<WARN_LOW or spd>SPEED_HIGH else COLOR_WARN)
        lt=FDS*2
        self.world.debug.draw_string(loc+carla.Location(z=5.0),
            f"Speed: {spd:.1f} km/h   T+{t:.0f}s",
            draw_shadow=True,color=col,life_time=lt,persistent_lines=False)
        if label:
            self.world.debug.draw_string(loc+carla.Location(z=6.5),label,
                draw_shadow=True,color=COLOR_INFO,life_time=lt,persistent_lines=False)
        if warning:
            self.world.debug.draw_string(loc+carla.Location(z=3.8),warning,
                draw_shadow=True,color=COLOR_DANGER,life_time=lt,persistent_lines=False)

class SpeedMonitor:
    def __init__(self, world, ego):
        self.world=world; self.ego=ego; self.hud=HUD(world,ego)
        self.scenario_time=0.0; self.slow_since=None; self.warning=""
    def ego_kmh(self):
        if not is_alive(self.ego): return 0.0
        try:
            v=self.ego.get_velocity()
            return 3.6*math.sqrt(v.x**2+v.y**2+v.z**2)
        except: return 0.0
    def _warn(self, spd):
        if spd>SPEED_HIGH:
            self.slow_since=None
            return "Please drive at 100-105 km/h. Your current speed is too fast."
        if spd<WARN_LOW:
            if self.slow_since is None: self.slow_since=self.scenario_time
            elif self.scenario_time-self.slow_since>=WARN_DELAY:
                return ("Please drive at 100-105 km/h. "
                        "Your current speed is too low. "
                        "Please accelerate to the target speed.")
        else: self.slow_since=None
        if SPEED_LOW<=spd<=SPEED_HIGH: return ""
        return self.warning
    def tick(self, dt, label=""):
        self.scenario_time+=dt
        spd=self.ego_kmh(); self.warning=self._warn(spd)
        self.hud.update(spd,self.warning,self.scenario_time,label)
        return self.scenario_time, spd


def beep(world, loc):
    world.debug.draw_point(loc+carla.Location(z=2.5),
        size=0.8,color=carla.Color(255,255,255),life_time=0.5)
    print("[BEEP] *** t=0 ***")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    client, world = connect_and_load(CARLA_HOST, CARLA_PORT)
    all_npcs        = []
    congestion_npcs = []

    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(False)
    tm.set_random_device_seed(42)
    tm.set_global_distance_to_leading_vehicle(3.0)

    try:
        ego = find_ego_vehicle(world)
        world.tick()
        beep(world, ego.get_transform().location)

        # A1-A4: NPCs in lane -3, cut left into lane -2
        a_events = [
            AEvent("A1", True,  DecelType.RAPID,    60.0, "vehicle.audi.a2"),
            AEvent("A2", True,  DecelType.GRADUAL, 170.0, "vehicle.bmw.grandtourer"),
            AEvent("A3", False, DecelType.RAPID,   280.0, "vehicle.mercedes.coupe"),
            AEvent("A4", False, DecelType.GRADUAL, 390.0, "vehicle.nissan.micra"),
        ]

        # Congestion cut-ins: NPCs in lane -2, cut right into lane -3
        c_events = [
            CongestionCutIn("C1", 530.0, solid_line=False),
            CongestionCutIn("C2", 595.0, solid_line=False),
            CongestionCutIn("C3", 665.0, solid_line=False),
            CongestionCutIn("C4", 755.0, solid_line=True),
        ]

        exit_event = ExitCutIn()

        rating_schedule = {
            120.0: "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
            210.0: "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
            300.0: "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
            385.0: "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
            505.0: "How much anger or frustration do you feel\ndue to the current traffic congestion?",
            570.0: "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
            635.0: "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
            705.0: "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
            795.0: "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
        }
        prompted          = set()
        congestion_spawned= False
        recovery_started  = False

        monitor      = SpeedMonitor(world, ego)
        status_every = int(2.0 / FDS)
        tick_count   = 0

        print("\n[Scenario 1] Running. Ctrl+C to stop.\n")

        while True:
            world.tick()
            if not is_alive(ego):
                print("Ego lost."); break

            t, ego_spd = monitor.tick(FDS)
            label = "Adaptation — drive 100-105 km/h" if t < 60.0 else ""

            # ── A1-A4 (0-500s): ego in lane -2 ───────────────────────
            if t < 500.0:
                for ev in a_events:
                    lbl = ev.update(world, ego, t)
                    if lbl: label = lbl
                    if ev.npc and ev.npc not in all_npcs and is_alive(ev.npc):
                        all_npcs.append(ev.npc)

            # ── Spawn congestion queue at t=500 AHEAD in lane -3 ──────
            if not congestion_spawned and t >= 500.0:
                print("\n[Congestion] Spawning queue in lane -3 AHEAD of ego...")
                beep(world, ego.get_transform().location)
                ego_wp = get_ego_wp(world, ego)
                if ego_wp:
                    lane3_wp = wp_in_lane(ego_wp, -3)
                    if lane3_wp:
                        for i in range(NUM_CONG_VEHS):
                            ahead_m = 15.0 + i * 10.0
                            nexts   = lane3_wp.next(ahead_m)
                            if not nexts: continue
                            wp = nexts[0]
                            npc = spawn_at_wp(world, wp)
                            if npc:
                                npc.set_autopilot(True, TM_PORT)
                                tm.auto_lane_change(npc, False)
                                tm.ignore_lights_percentage(npc, 100)
                                tm.vehicle_percentage_speed_difference(
                                    npc, pct(random.uniform(
                                        SPEED_CONG_MIN, SPEED_CONG_MAX)))
                                congestion_npcs.append(npc)
                                all_npcs.append(npc)
                print(f"[Congestion] {len(congestion_npcs)} vehicles in lane -3")
                congestion_spawned = True

            # ── 500-750s: congestion ──────────────────────────────────
            if 500.0 <= t < 750.0:
                label = (f"Congestion — ego in lane-3  "
                         f"{SPEED_CONG_MIN:.0f}-{SPEED_CONG_MAX:.0f} km/h")
                # Maintain slow speed in lane -3
                for npc in congestion_npcs:
                    if is_alive(npc):
                        tm.vehicle_percentage_speed_difference(
                            npc, pct(random.uniform(SPEED_CONG_MIN, SPEED_CONG_MAX)))
                # Cut-ins from lane -2 into lane -3
                for ev in c_events:
                    lbl = ev.update(world, ego, t)
                    if lbl: label = lbl
                    if ev.npc and ev.npc not in all_npcs and is_alive(ev.npc):
                        all_npcs.append(ev.npc)

            # ── 750-800s: recovery ────────────────────────────────────
            elif 750.0 <= t < 800.0:
                if not recovery_started:
                    print("[Congestion] Recovery starting...")
                    recovery_started = True
                progress     = (t-750.0)/50.0
                recovery_spd = SPEED_CONG_MAX + progress*(SPEED_CRUISE-SPEED_CONG_MAX)
                label        = f"Recovery — lane-3 → {recovery_spd:.0f} km/h"
                for npc in congestion_npcs:
                    if is_alive(npc):
                        tm.vehicle_percentage_speed_difference(npc, pct(recovery_spd))

            # ── 800-810s: pre-exit ────────────────────────────────────
            elif 800.0 <= t < 810.0:
                label = "Pre-exit transition"

            # ── 810-890s: exit cut-in ─────────────────────────────────
            elif 810.0 <= t < 890.0:
                lbl = exit_event.update(world, ego, t)
                if lbl: label = lbl
                if (exit_event.npc and exit_event.npc not in all_npcs
                        and is_alive(exit_event.npc)):
                    all_npcs.append(exit_event.npc)

            elif t >= 890.0:
                label = "Cooldown"

            # ── Rating prompts ────────────────────────────────────────
            for pt, msg in rating_schedule.items():
                if t >= pt and pt not in prompted:
                    show_rating(world, ego, msg, duration_s=15.0)
                    prompted.add(pt)

            monitor.hud.update(ego_spd, monitor.warning, t, label)
            tick_count += 1
            if tick_count % status_every == 0:
                print(f"  T={t:>6.1f}s  ego={ego_spd:>5.1f} km/h  [{label}]")

            if t >= 900.0:
                print("\n[Scenario 1] Complete at T=900s."); break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[Scenario 1] Stopped.")
    finally:
        for npc in all_npcs: safe_destroy(npc)
        try:
            s=world.get_settings(); s.synchronous_mode=False
            s.fixed_delta_seconds=None; world.apply_settings(s)
        except: pass
        print("[Scenario 1] Done.")

if __name__ == "__main__":
    main()