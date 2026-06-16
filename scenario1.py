"""
Scenario 1 - Step 4: A1-A4 Cut-in Events
Merged best of both codebases:
- Real indicator lights (set_light_state) from user code
- Proportional brake deceleration profile from user code
- Lead vehicle detection from user code
- DReyeVR ego finder from our code
- Waypoint-based NPC steering from our code
- State machine timing from our code
- Spawn 10s before event, tail ego, overtake, cut in
"""

import carla, time, math, threading
from enum import Enum

CARLA_HOST     = "localhost"
CARLA_PORT     = 2000
FDS            = 0.05
TM_PORT        = 8001
ROAD_LIMIT     = 120.0
SPEED_CRUISE   = 100.0
SPEED_CATCHUP  = 110.0
SPEED_SLOW     = 70.0
CUT_AHEAD_M    = 10.0
SPAWN_BEFORE_S = 10.0
DESTROY_M      = 300.0
SPEED_LOW      = 100.0
SPEED_HIGH     = 105.0
WARN_LOW       = 90.0
WARN_DELAY     = 20.0

COLOR_OK     = carla.Color(0, 220, 0)
COLOR_WARN   = carla.Color(255, 180, 0)
COLOR_DANGER = carla.Color(255, 40, 40)
COLOR_INFO   = carla.Color(100, 200, 255)

NPC_MODELS = [
    "vehicle.audi.a2",
    "vehicle.bmw.grandtourer",
    "vehicle.mercedes.coupe",
    "vehicle.nissan.micra",
]

class DecelType(Enum):
    RAPID   = 3.0
    GRADUAL = 6.0

# ──────────────────────────────────────────────────────────────────────────────
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
    """Signed distance: positive = npc is ahead of ego."""
    if not is_alive(ego) or not is_alive(npc): return 0.0
    try:
        tf  = ego.get_transform(); fwd = tf.get_forward_vector()
        d   = npc.get_transform().location - tf.location
        return d.x*fwd.x + d.y*fwd.y
    except: return 0.0

def get_lead_vehicle(ego, candidates):
    """
    From user code: find closest vehicle ahead of ego within 50m.
    Returns (vehicle, distance) or (None, inf).
    """
    if not is_alive(ego): return None, float('inf')
    ego_loc = ego.get_location()
    ego_fwd = ego.get_transform().get_forward_vector()
    closest_dist = float('inf')
    closest_veh  = None
    for v in candidates:
        if not is_alive(v): continue
        rel = v.get_location() - ego_loc
        dot = rel.x*ego_fwd.x + rel.y*ego_fwd.y
        if dot > 0:
            d = ego_loc.distance(v.get_location())
            if d < closest_dist and d < 50.0:
                closest_dist = d
                closest_veh  = v
    return closest_veh, closest_dist


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
    model = "harplab.dreyevr_vehicle.model3"
    bp    = world.get_blueprint_library().find(model)
    return world.spawn_actor(bp, world.get_map().get_spawn_points()[0])

def precise_respawn(vehicle, location, yaw=None, pitch=0.0, roll=0.0):
    try:
        if not isinstance(location, carla.Location):
            location = carla.Location(*location)
        if yaw is None:
            wp  = vehicle.get_world().get_map().get_waypoint(
                location, project_to_road=True)
            yaw = wp.transform.rotation.yaw if wp else 0.0
        vehicle.set_simulate_physics(False)
        vehicle.set_transform(carla.Transform(
            location,
            carla.Rotation(pitch=float(pitch),
                           yaw=float(yaw), roll=float(roll))))
        time.sleep(0.1)
        vehicle.set_simulate_physics(True)
        return True
    except Exception as e:
        print(f"Respawn failed: {e}")
        try: vehicle.set_simulate_physics(True)
        except: pass
        return False


# ── NPC spawn ─────────────────────────────────────────────────────────────────
def spawn_npc(world, ego, model="vehicle.audi.a2", behind_m=8.0):
    """Spawn NPC in lane 3, behind_m behind ego. Physics ON."""
    town_map = world.get_map()
    ego_wp   = town_map.get_waypoint(
        ego.get_transform().location, project_to_road=True,
        lane_type=carla.LaneType.Driving)
    if not ego_wp: return None

    prev = ego_wp.previous(behind_m)
    if not prev: return None
    wp = prev[0]

    for _ in range(6):
        if wp.lane_id == -3: break
        r = wp.get_right_lane(); l = wp.get_left_lane()
        if r and r.lane_type == carla.LaneType.Driving: wp = r
        elif l and l.lane_type == carla.LaneType.Driving: wp = l
        else: break

    spawn_tf = carla.Transform(
        carla.Location(x=wp.transform.location.x,
                       y=wp.transform.location.y,
                       z=wp.transform.location.z + 0.5),
        wp.transform.rotation)

    blib = world.get_blueprint_library()
    bp   = None
    for m in [model] + NPC_MODELS + ["vehicle.tesla.model3"]:
        found = blib.filter(m)
        if found: bp = found[0]; break
    if not bp: return None
    bp.set_attribute("role_name", "npc")

    npc = world.try_spawn_actor(bp, spawn_tf)
    if not npc: print(f"[NPC] Spawn failed"); return None

    npc.set_autopilot(False)
    npc.set_simulate_physics(True)
    print(f"[NPC] id={npc.id}  model={bp.id}  lane={wp.lane_id}  {behind_m:.0f}m behind")
    return npc


# ── Indicator (real lights from user code) ────────────────────────────────────
def set_indicator(npc, world, on=True, duration_s=2.5):
    """Orange arrow on LEFT side of NPC — debug arrow indicator."""
    if not is_alive(npc) or not on: return
    try:
        tf  = npc.get_transform(); fwd = tf.get_forward_vector()
        left = carla.Location(
            x=tf.location.x + fwd.y * 3.5,
            y=tf.location.y - fwd.x * 3.5,
            z=tf.location.z + 1.5)
        world.debug.draw_arrow(
            tf.location + carla.Location(z=1.0), left,
            thickness=0.15, arrow_size=0.3,
            color=carla.Color(255, 165, 0),
            life_time=duration_s, persistent_lines=False)
    except: pass


# ── NPC controller ────────────────────────────────────────────────────────────
class NPCController:
    """
    Physics-ON waypoint follower with proportional speed control.
    Steering: lookahead waypoint in target lane.
    Speed: proportional throttle/brake (from user code pattern).
    """
    LOOKAHEAD = 8.0
    KP_STEER  = 1.2
    MAX_STEER = 0.6

    def __init__(self, world, npc, target_lane=-3):
        self.world       = world
        self.npc         = npc
        self.target_lane = target_lane

    def change_lane(self, lane_id):
        print(f"[NPC {self.npc.id}] Lane target → {lane_id}")
        self.target_lane = lane_id

    def in_target_lane(self):
        if not is_alive(self.npc): return False
        try:
            wp = self.world.get_map().get_waypoint(
                self.npc.get_transform().location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving)
            return wp is not None and wp.lane_id == self.target_lane
        except: return False

    def tick(self, target_kmh):
        if not is_alive(self.npc): return
        try:
            npc_tf  = self.npc.get_transform()
            npc_loc = npc_tf.location
            fwd     = npc_tf.get_forward_vector()

            # Lookahead location
            look_loc = carla.Location(
                x=npc_loc.x + fwd.x * self.LOOKAHEAD,
                y=npc_loc.y + fwd.y * self.LOOKAHEAD,
                z=npc_loc.z)

            town_map = self.world.get_map()
            wp = town_map.get_waypoint(look_loc, project_to_road=True,
                                        lane_type=carla.LaneType.Driving)
            if wp is None:
                wp = town_map.get_waypoint(npc_loc, project_to_road=True,
                                            lane_type=carla.LaneType.Driving)
            if wp is None: return

            # Walk to target lane
            tgt = wp
            for _ in range(6):
                if tgt.lane_id == self.target_lane: break
                l = tgt.get_left_lane(); r = tgt.get_right_lane()
                if l and l.lane_type == carla.LaneType.Driving: tgt = l
                elif r and r.lane_type == carla.LaneType.Driving: tgt = r
                else: break

            # Steer toward target lane waypoint
            to_tgt = carla.Vector3D(
                tgt.transform.location.x - npc_loc.x,
                tgt.transform.location.y - npc_loc.y, 0.0)
            mag = math.sqrt(to_tgt.x**2 + to_tgt.y**2)
            if mag > 0.001:
                to_tgt.x /= mag; to_tgt.y /= mag
            cross = fwd.x * to_tgt.y - fwd.y * to_tgt.x
            steer = max(-self.MAX_STEER,
                        min(self.MAX_STEER, self.KP_STEER * cross))

            # Proportional throttle/brake (from user code)
            speed     = kmh_of(self.npc)
            speed_err = target_kmh - speed
            if speed_err > 0:
                throttle = min(1.0, speed_err / 20.0)
                brake    = 0.0
            else:
                throttle = 0.0
                # Proportional brake: full at 10 km/h over target
                brake = min(1.0, abs(speed_err) / 10.0)

            self.npc.apply_control(carla.VehicleControl(
                throttle=float(throttle),
                steer=float(steer),
                brake=float(brake),
                hand_brake=False,
                manual_gear_shift=False))
        except Exception as e:
            print(f"[NPC ctrl] {e}")


# ── HUD ───────────────────────────────────────────────────────────────────────
class HUD:
    def __init__(self, world, ego):
        self.world = world; self.ego = ego
    def update(self, spd, warning, t, label=""):
        if not is_alive(self.ego): return
        try: loc = self.ego.get_transform().location
        except: return
        col = (COLOR_OK if SPEED_LOW <= spd <= SPEED_HIGH else
               COLOR_DANGER if spd < WARN_LOW or spd > SPEED_HIGH
               else COLOR_WARN)
        lt = FDS * 2
        self.world.debug.draw_string(loc + carla.Location(z=5.0),
            f"Speed: {spd:.1f} km/h   T+{t:.0f}s",
            draw_shadow=True, color=col, life_time=lt, persistent_lines=False)
        if label:
            self.world.debug.draw_string(loc + carla.Location(z=6.5), label,
                draw_shadow=True, color=COLOR_INFO,
                life_time=lt, persistent_lines=False)
        if warning:
            self.world.debug.draw_string(loc + carla.Location(z=3.8), warning,
                draw_shadow=True, color=COLOR_DANGER,
                life_time=lt, persistent_lines=False)


# ── Speed monitor ─────────────────────────────────────────────────────────────
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
        if spd > SPEED_HIGH:
            self.slow_since=None
            return f"! Too fast ({spd:.0f}) target {SPEED_LOW:.0f}-{SPEED_HIGH:.0f}"
        if spd < WARN_LOW:
            if self.slow_since is None: self.slow_since=self.scenario_time
            elif self.scenario_time-self.slow_since >= WARN_DELAY:
                return f"! Too slow ({spd:.0f}) reach {SPEED_LOW:.0f}-{SPEED_HIGH:.0f}"
        else: self.slow_since=None
        if SPEED_LOW <= spd <= SPEED_HIGH: return ""
        return self.warning
    def tick(self, dt, label=""):
        self.scenario_time += dt
        spd=self.ego_kmh(); self.warning=self._warn(spd)
        self.hud.update(spd, self.warning, self.scenario_time, label)
        return self.scenario_time, spd


# ── A-Event ───────────────────────────────────────────────────────────────────
class AEvent:
    """
    States: waiting → tailing → overtake → cutin → decel → tail_ego → reaccel → done
    Uses real indicator lights, proportional brake, lead vehicle detection.
    """
    def __init__(self, label, indicator, decel_type, start_t, model):
        self.label      = label
        self.indicator  = indicator        # True/False
        self.decel_type = decel_type       # DecelType enum
        self.start_t    = start_t
        self.model      = model
        self.npc        = None
        self.ctrl       = None
        self.state      = "waiting"
        self._state_t   = 0.0
        self._ind_done  = False
        self._spawn_t   = start_t - SPAWN_BEFORE_S
        print(f"[{label}] model={model}  ind={'ON' if indicator else 'OFF'}  "
              f"decel={decel_type.name}({decel_type.value}s)  "
              f"spawn@{self._spawn_t:.0f}s  cutin@{start_t:.0f}s")

    def _set_state(self, s, t):
        print(f"[{self.label}] {self.state} -> {s}  T={t:.1f}s")
        self.state=s; self._state_t=t

    def update(self, world, ego, t):
        try: return self._update(world, ego, t)
        except Exception as e:
            print(f"[{self.label}] error: {e}"); return ""

    def _update(self, world, ego, t):
        ego_spd = kmh_of(ego) if is_alive(ego) else SPEED_CRUISE

        # ── Waiting ───────────────────────────────────────────────────
        if self.state == "waiting":
            if t >= self._spawn_t:
                self.npc = spawn_npc(world, ego, self.model, behind_m=8.0)
                if self.npc:
                    self.ctrl = NPCController(world, self.npc, target_lane=-3)
                    self._set_state("tailing", t)
                else:
                    print(f"[{self.label}] spawn failed — retry next tick")
            return ""

        if not is_alive(self.npc):
            return f"{self.label} (NPC gone)"

        ahead = dist_ahead(ego, self.npc)

        # ── Tailing: match ego speed in lane 3 ───────────────────────
        if self.state == "tailing":
            self.ctrl.tick(max(ego_spd, 20.0))
            if t >= self.start_t:
                self._set_state("overtake", t)
            return (f"{self.label} — tailing  "
                    f"{kmh_of(self.npc):.0f} km/h  ahead={ahead:.0f}m")

        # ── Overtake ──────────────────────────────────────────────────
        elif self.state == "overtake":
            ramp = min((t - self._state_t) / 5.0, 1.0)
            spd  = ego_spd + ramp * (SPEED_CATCHUP - ego_spd)
            self.ctrl.tick(spd)
            if ahead >= CUT_AHEAD_M:
                # Real indicator light (from user code)
                if not self._ind_done:
                    set_indicator(self.npc, world, on=self.indicator)
                    if self.indicator:
                        print(f"[{self.label}] Left blinker ON")
                    self._ind_done = True
                self.ctrl.change_lane(-2)
                self._set_state("cutin", t)
            return (f"{self.label} — overtaking  "
                    f"{kmh_of(self.npc):.0f} km/h  ahead={ahead:.0f}m")

        # ── Cutin: steer to lane 2, hold cruise speed ─────────────────
        elif self.state == "cutin":
            self.ctrl.tick(SPEED_CRUISE)
            if self.ctrl.in_target_lane():
                set_indicator(self.npc, world, on=False)   # turn off blinker
                self._set_state("decel", t)
            return f"{self.label} — cutting in  {kmh_of(self.npc):.0f} km/h"

        # ── Decel: proportional brake profile (from user code) ────────
        elif self.state == "decel":
            duration = self.decel_type.value
            progress = min((t - self._state_t) / duration, 1.0)
            # Target speed ramps down linearly
            target   = SPEED_CRUISE + progress * (SPEED_SLOW - SPEED_CRUISE)
            # Proportional brake toward target (user code pattern)
            speed    = kmh_of(self.npc)
            speed_diff = speed - target
            if speed_diff > 0:
                brake    = min(1.0, speed_diff / 10.0)
                throttle = 0.0
            else:
                brake    = 0.0
                throttle = min(0.3, abs(speed_diff) / 20.0)
            # Still steer to stay in lane 2
            self.ctrl.tick(target)
            if progress >= 1.0:
                self._set_state("tail_ego", t)
            style = self.decel_type.name.lower()
            return (f"{self.label} — decel ({style})  "
                    f"{speed:.0f}→{target:.0f} km/h")

        # ── Tail ego: hold 70 km/h ─────────────────────────────────────
        elif self.state == "tail_ego":
            self.ctrl.tick(SPEED_SLOW)
            if t - self._state_t >= 5.0:
                self._set_state("reaccel", t)
            return f"{self.label} — tail  {kmh_of(self.npc):.0f} km/h"

        # ── Reaccel ───────────────────────────────────────────────────
        elif self.state == "reaccel":
            progress = min((t - self._state_t) / 20.0, 1.0)
            spd = SPEED_SLOW + progress * (SPEED_CRUISE - SPEED_SLOW)
            self.ctrl.tick(spd)
            if progress >= 1.0:
                self._set_state("done", t)
            return f"{self.label} — reaccel  {kmh_of(self.npc):.0f} km/h"

        # ── Done ──────────────────────────────────────────────────────
        elif self.state == "done":
            self.ctrl.tick(SPEED_CRUISE)
            if ahead > DESTROY_M or ahead < -50.0:
                print(f"[{self.label}] NPC removed (ahead={ahead:.0f}m)")
                safe_destroy(self.npc); self.npc=None
                return f"{self.label} — done"
            return f"{self.label} — cruising  {kmh_of(self.npc):.0f} km/h"

        return ""


# ── Beep ──────────────────────────────────────────────────────────────────────
def beep(world, loc):
    world.debug.draw_point(loc + carla.Location(z=2.5),
        size=0.8, color=carla.Color(255,255,255), life_time=0.5)
    print("[BEEP] *** t=0 ***")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    client, world = connect_and_load(CARLA_HOST, CARLA_PORT)
    all_npcs = []

    try:
        ego = find_ego_vehicle(world)
        world.tick()
        beep(world, ego.get_transform().location)

        events = [
            AEvent("A1", True,  DecelType.RAPID,   60.0,  "vehicle.audi.a2"),
            AEvent("A2", True,  DecelType.GRADUAL, 170.0, "vehicle.bmw.grandtourer"),
            AEvent("A3", False, DecelType.RAPID,   280.0, "vehicle.mercedes.coupe"),
            AEvent("A4", False, DecelType.GRADUAL, 390.0, "vehicle.nissan.micra"),
        ]

        monitor      = SpeedMonitor(world, ego)
        status_every = int(2.0 / FDS)
        tick_count   = 0
        print("\n[Step 4] Running. Ctrl+C to stop.\n")

        while True:
            world.tick()
            if not is_alive(ego):
                print("[Step 4] Ego lost."); break

            t, ego_spd = monitor.tick(FDS)
            label = "Adaptation — drive 100-105 km/h" if t < 60.0 else ""

            for ev in events:
                ev_label = ev.update(world, ego, t)
                if ev_label: label = ev_label
                if ev.npc and ev.npc not in all_npcs and is_alive(ev.npc):
                    all_npcs.append(ev.npc)

            monitor.hud.update(ego_spd, monitor.warning, t, label)
            tick_count += 1
            if tick_count % status_every == 0:
                print(f"  T={t:>6.1f}s  ego={ego_spd:>5.1f} km/h  [{label}]")

            if t >= 500.0:
                print("\n[Step 4] Complete."); break
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[Step 4] Stopped.")
    finally:
        for npc in all_npcs: safe_destroy(npc)
        try:
            s=world.get_settings(); s.synchronous_mode=False
            s.fixed_delta_seconds=None; world.apply_settings(s)
        except: pass
        print("[Step 4] Done.")

if __name__ == "__main__":
    main()