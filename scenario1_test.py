"""
Scenario 1 - Step 4: A1-A4 Cut-in Events
Fixes:
- NPC moves forward AND laterally simultaneously during lane change (no stopping)
- Speed properly ramped at each phase
- NPC spawns 10s before event in lane 3, tails ego, then overtakes and cuts in
- 4 different vehicle models
"""

from typing import Optional
import carla, time, math

CARLA_HOST          = "localhost"
CARLA_PORT          = 2000
FIXED_DELTA_SECONDS = 0.05
TM_PORT             = 8001

ROAD_SPEED_LIMIT    = 120.0
SPEED_CRUISE        = 50.0
SPEED_CATCHUP       = 110.0
SPEED_SLOW          = 70.0
CUT_AHEAD_M         = 10.0
SPAWN_BEFORE_S      = 10.0
DESTROY_M           = 300.0

SPEED_TARGET_LOW    = 100.0
SPEED_TARGET_HIGH   = 105.0
SPEED_WARN_LOW      = 90.0
SLOW_WARN_DELAY_S   = 20.0

COLOR_OK     = carla.Color(r=0,   g=220, b=0)
COLOR_WARN   = carla.Color(r=255, g=180, b=0)
COLOR_DANGER = carla.Color(r=255, g=40,  b=40)
COLOR_INFO   = carla.Color(r=100, g=200, b=255)

NPC_MODELS = [
    "vehicle.audi.a2",
    "vehicle.bmw.grandtourer",
    "vehicle.mercedes.coupe",
    "vehicle.nissan.micra",
]

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
    if not is_alive(ego) or not is_alive(npc): return 0.0
    try:
        tf  = ego.get_transform(); fwd = tf.get_forward_vector()
        d   = npc.get_transform().location - tf.location
        return d.x*fwd.x + d.y*fwd.y
    except: return 0.0


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
    ego_vehicles = list(world.get_actors().filter("harplab.dreyevr_vehicle.*"))
    if ego_vehicles: return ego_vehicles[0]
    model = "harplab.dreyevr_vehicle.model3"
    print(f'No EgoVehicle found, spawning: "{model}"')
    bp = world.get_blueprint_library().find(model)
    return world.spawn_actor(bp, world.get_map().get_spawn_points()[0])

def precise_respawn(vehicle, location, yaw=None, pitch=0.0, roll=0.0):
    try:
        if not isinstance(location, carla.Location):
            location = carla.Location(*location)
        if yaw is None:
            wp  = vehicle.get_world().get_map().get_waypoint(location, project_to_road=True)
            yaw = wp.transform.rotation.yaw if wp else 0.0
        vehicle.set_simulate_physics(False)
        vehicle.set_transform(carla.Transform(
            location, carla.Rotation(pitch=float(pitch), yaw=float(yaw), roll=float(roll))))
        time.sleep(0.1)
        vehicle.set_simulate_physics(True)
        return True
    except Exception as e:
        print(f"Respawn failed: {e}")
        try: vehicle.set_simulate_physics(True)
        except: pass
        return False


# ── NPC spawn ─────────────────────────────────────────────────────────────────
def spawn_npc_lane3(world, ego, behind_m=8.0, model="vehicle.audi.a2"):
    """Spawn NPC in lane 3, behind_m behind ego. Physics ON, no autopilot."""
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

    tf = carla.Transform(
        carla.Location(x=wp.transform.location.x,
                       y=wp.transform.location.y,
                       z=wp.transform.location.z + 0.3),
        wp.transform.rotation)

    blib = world.get_blueprint_library()
    bp   = None
    for m in [model] + NPC_MODELS + ["vehicle.tesla.model3"]:
        found = blib.filter(m)
        if found: bp = found[0]; break
    if not bp: return None
    bp.set_attribute("role_name", "npc")

    npc = world.try_spawn_actor(bp, tf)
    if not npc: print(f"[NPC] Spawn failed"); return None

    # Physics ON, no autopilot — we control via set_transform each tick
    npc.set_autopilot(False)
    print(f"[NPC] id={npc.id}  model={bp.id}  lane={wp.lane_id}  {behind_m:.0f}m behind")
    return npc


# ── Kinematic driver ──────────────────────────────────────────────────────────
class KinematicDriver:
    """
    Drives NPC kinematically using waypoints + set_transform each tick.
    Physics OFF so no crashes, but position/rotation follows road naturally.
    Supports smooth lateral lane change while continuing forward motion.
    """
    def __init__(self, world, npc, lane_id=-3):
        self.world   = world
        self.npc     = npc
        self.lane_id = lane_id
        self._wp     = self._find_wp(npc.get_transform().location, lane_id)
        self._lateral_offset = 0.0      # current lateral offset in metres
        self._lateral_target = 0.0      # target lateral offset
        self._lateral_speed  = 1.5      # metres per second lateral shift
        npc.set_simulate_physics(False)

    def _find_wp(self, loc, lane_id):
        town_map = self.world.get_map()
        wp = town_map.get_waypoint(loc, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        if not wp: return None
        for _ in range(6):
            if wp.lane_id == lane_id: return wp
            l = wp.get_left_lane(); r = wp.get_right_lane()
            if l and l.lane_type == carla.LaneType.Driving: wp = l
            elif r and r.lane_type == carla.LaneType.Driving: wp = r
            else: break
        return wp

    def change_lane(self, target_lane_id):
        """
        Compute lateral offset needed to reach target lane centre.
        NPC will drift laterally each tick while still moving forward.
        """
        if not self._wp: return
        cur_wp = self._wp
        tgt_wp = cur_wp
        for _ in range(6):
            if tgt_wp.lane_id == target_lane_id: break
            l = tgt_wp.get_left_lane(); r = tgt_wp.get_right_lane()
            if l and l.lane_type == carla.LaneType.Driving: tgt_wp = l
            elif r and r.lane_type == carla.LaneType.Driving: tgt_wp = r
            else: break

        # Compute lateral distance between lane centres
        cur_loc = cur_wp.transform.location
        tgt_loc = tgt_wp.transform.location
        # Right vector of road
        right = cur_wp.transform.get_right_vector()
        dx = tgt_loc.x - cur_loc.x
        dy = tgt_loc.y - cur_loc.y
        # Project onto right vector to get signed lateral distance
        lat = dx * right.x + dy * right.y
        self._lateral_target = lat
        self.lane_id = target_lane_id
        print(f"[KD] id={self.npc.id} lane change → {target_lane_id}  "
              f"lateral_dist={lat:.2f}m")

    def in_target_lane(self):
        """Returns True when lateral offset has reached target."""
        return abs(self._lateral_offset - self._lateral_target) < 0.1

    def tick(self, speed_kmh):
        """Advance NPC forward at speed_kmh, drift laterally if lane changing."""
        if not is_alive(self.npc) or not self._wp:
            return

        dt   = FIXED_DELTA_SECONDS
        dist = (speed_kmh / 3.6) * dt

        # Advance waypoint forward
        nexts = self._wp.next(dist)
        if nexts:
            self._wp = nexts[0]

        # Update lateral offset toward target
        lat_diff = self._lateral_target - self._lateral_offset
        lat_step = self._lateral_speed * dt
        if abs(lat_diff) < lat_step:
            self._lateral_offset = self._lateral_target
        else:
            self._lateral_offset += math.copysign(lat_step, lat_diff)

        # Build final transform: road waypoint + lateral offset
        wp_tf    = self._wp.transform
        right    = wp_tf.get_right_vector()
        offset_x = right.x * self._lateral_offset
        offset_y = right.y * self._lateral_offset

        final_loc = carla.Location(
            x=wp_tf.location.x + offset_x,
            y=wp_tf.location.y + offset_y,
            z=wp_tf.location.z + 0.05)

        self.npc.set_transform(carla.Transform(final_loc, wp_tf.rotation))


# ── Indicator ─────────────────────────────────────────────────────────────────
def flash_indicator(world, npc, duration_s=2.5):
    if not is_alive(npc): return
    try:
        tf = npc.get_transform(); fwd = tf.get_forward_vector()
        left = carla.Location(x=tf.location.x + fwd.y*3.5,
                               y=tf.location.y - fwd.x*3.5,
                               z=tf.location.z + 1.5)
        world.debug.draw_arrow(
            tf.location + carla.Location(z=1.0), left,
            thickness=0.15, arrow_size=0.3,
            color=carla.Color(255, 165, 0),
            life_time=duration_s, persistent_lines=False)
    except: pass


# ── HUD ───────────────────────────────────────────────────────────────────────
class HUD:
    def __init__(self, world, ego):
        self.world = world; self.ego = ego
    def update(self, spd, warning, t, label=""):
        if not is_alive(self.ego): return
        try: loc = self.ego.get_transform().location
        except: return
        col = (COLOR_OK if SPEED_TARGET_LOW <= spd <= SPEED_TARGET_HIGH else
               COLOR_DANGER if spd < SPEED_WARN_LOW or spd > SPEED_TARGET_HIGH
               else COLOR_WARN)
        lt = FIXED_DELTA_SECONDS * 2
        self.world.debug.draw_string(loc + carla.Location(z=5.0),
            f"Speed: {spd:.1f} km/h   T+{t:.0f}s",
            draw_shadow=True, color=col, life_time=lt, persistent_lines=False)
        if label:
            self.world.debug.draw_string(loc + carla.Location(z=6.5), label,
                draw_shadow=True, color=COLOR_INFO, life_time=lt, persistent_lines=False)
        if warning:
            self.world.debug.draw_string(loc + carla.Location(z=3.8), warning,
                draw_shadow=True, color=COLOR_DANGER, life_time=lt, persistent_lines=False)


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
        if spd > SPEED_TARGET_HIGH:
            self.slow_since=None
            return f"! Too fast ({spd:.0f}) target {SPEED_TARGET_LOW:.0f}-{SPEED_TARGET_HIGH:.0f}"
        if spd < SPEED_WARN_LOW:
            if self.slow_since is None: self.slow_since=self.scenario_time
            elif self.scenario_time-self.slow_since >= SLOW_WARN_DELAY_S:
                return f"! Too slow ({spd:.0f}) reach {SPEED_TARGET_LOW:.0f}-{SPEED_TARGET_HIGH:.0f}"
        else: self.slow_since=None
        if SPEED_TARGET_LOW <= spd <= SPEED_TARGET_HIGH: return ""
        return self.warning
    def tick(self, dt, label=""):
        self.scenario_time += dt
        spd=self.ego_kmh(); self.warning=self._warn(spd)
        self.hud.update(spd, self.warning, self.scenario_time, label)
        return self.scenario_time, spd


# ── A-Event ───────────────────────────────────────────────────────────────────
class AEvent:
    """
    States:
      waiting   : before spawn time (start_t - SPAWN_BEFORE_S)
      tailing   : NPC in lane 3, matching ego speed (10s)
      overtake  : NPC accelerates to CUT_AHEAD_M ahead
      cutin     : lane change triggered, drifting to lane 2
      decel     : 100->70 over decel_time_s
      tail_ego  : hold 70 km/h
      reaccel   : 70->100 over 20s
      done      : cruise 100 km/h
    """
    def __init__(self, label, indicator, decel_time_s, start_t,
                 vehicle_model="vehicle.audi.a2"):
        self.label         = label
        self.indicator     = indicator
        self.decel_time_s  = decel_time_s
        self.start_t       = start_t
        self.vehicle_model = vehicle_model
        self.npc           = None
        self.driver        = None
        self.state         = "waiting"
        self._state_t      = 0.0
        self._ind_done     = False
        self._spawn_t      = start_t - SPAWN_BEFORE_S
        print(f"[{label}] model={vehicle_model}  "
              f"indicator={'ON' if indicator else 'OFF'}  "
              f"decel={decel_time_s}s  spawn@{self._spawn_t:.0f}s  cutin@{start_t:.0f}s")

    def _set_state(self, s, t):
        print(f"[{self.label}] {self.state} -> {s}  T={t:.1f}s")
        self.state=s; self._state_t=t

    def update(self, world, ego, t):
        try: return self._update(world, ego, t)
        except Exception as e:
            print(f"[{self.label}] error: {e}"); return ""

    def _update(self, world, ego, t):
        ego_spd = kmh_of(ego) if is_alive(ego) else SPEED_CRUISE

        # ── Waiting: spawn NPC 10s before event ──────────────────────
        if self.state == "waiting":
            if t >= self._spawn_t:
                self.npc = spawn_npc_lane3(
                    world, ego,
                    behind_m=8.0,
                    model=self.vehicle_model)
                if self.npc:
                    self.driver = KinematicDriver(world, self.npc, lane_id=-3)
                    self._set_state("tailing", t)
                else:
                    print(f"[{self.label}] spawn failed")
            return ""

        if not is_alive(self.npc):
            return f"{self.label} (NPC gone)"

        ahead = dist_ahead(ego, self.npc)

        # ── Tailing: match ego speed in lane 3 ───────────────────────
        if self.state == "tailing":
            spd = max(ego_spd, 20.0)
            self.driver.tick(spd)
            if t >= self.start_t:
                self._set_state("overtake", t)
            return f"{self.label} — tailing  {spd:.0f} km/h"

        # ── Overtake: accelerate to get CUT_AHEAD_M ahead ────────────
        elif self.state == "overtake":
            # Ramp from ego speed to CATCHUP over 5s
            ramp = min((t - self._state_t) / 5.0, 1.0)
            spd  = ego_spd + ramp * (SPEED_CATCHUP - ego_spd)
            self.driver.tick(spd)
            if ahead >= CUT_AHEAD_M:
                if not self._ind_done and self.indicator:
                    flash_indicator(world, self.npc, duration_s=2.5)
                    print(f"[{self.label}] Indicator ON")
                self._ind_done = True
                self.driver.change_lane(-2)
                self._set_state("cutin", t)
            return f"{self.label} — overtaking  {spd:.0f} km/h  ahead={ahead:.0f}m"

        # ── Cutin: continue forward at cruise while drifting to lane 2
        elif self.state == "cutin":
            self.driver.tick(SPEED_CRUISE)
            if self.driver.in_target_lane():
                self._set_state("decel", t)
            return f"{self.label} — cutting in..."

        # ── Decel: 100->70 over decel_time_s ─────────────────────────
        elif self.state == "decel":
            progress = min((t - self._state_t) / self.decel_time_s, 1.0)
            spd = SPEED_CRUISE + progress * (SPEED_SLOW - SPEED_CRUISE)
            self.driver.tick(spd)
            if progress >= 1.0: self._set_state("tail_ego", t)
            style = "rapid" if self.decel_time_s == 3 else "gradual"
            return f"{self.label} — decel ({style})  {spd:.0f} km/h"

        # ── Tail ego: hold 70 km/h ────────────────────────────────────
        elif self.state == "tail_ego":
            self.driver.tick(SPEED_SLOW)
            if t - self._state_t >= 5.0: self._set_state("reaccel", t)
            return f"{self.label} — tail  {SPEED_SLOW:.0f} km/h"

        # ── Reaccel: 70->100 over 20s ────────────────────────────────
        elif self.state == "reaccel":
            progress = min((t - self._state_t) / 20.0, 1.0)
            spd = SPEED_SLOW + progress * (SPEED_CRUISE - SPEED_SLOW)
            self.driver.tick(spd)
            if progress >= 1.0: self._set_state("done", t)
            return f"{self.label} — reaccel  {spd:.0f} km/h"

        # ── Done: cruise at 100 ───────────────────────────────────────
        elif self.state == "done":
            self.driver.tick(SPEED_CRUISE)
            if ahead > DESTROY_M or ahead < -50.0:
                print(f"[{self.label}] NPC removed")
                safe_destroy(self.npc); self.npc=None
                return f"{self.label} — done"
            return f"{self.label} — cruising"

        return ""


# ── Beep ──────────────────────────────────────────────────────────────────────
def beep(world, location):
    world.debug.draw_point(location + carla.Location(z=2.5),
                           size=0.8, color=carla.Color(255, 255, 255),
                           life_time=0.5)
    print("[BEEP] *** t=0 ***")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    client, world = connect_and_load(CARLA_HOST, CARLA_PORT)
    all_npcs      = []

    try:
        ego = find_ego_vehicle(world)
        world.tick()
        beep(world, ego.get_transform().location)

        events = [
            AEvent("A1", True,  3.0, 60.0,  "vehicle.audi.a2"),
            AEvent("A2", True,  6.0, 170.0, "vehicle.bmw.grandtourer"),
            AEvent("A3", False, 3.0, 280.0, "vehicle.mercedes.coupe"),
            AEvent("A4", False, 6.0, 390.0, "vehicle.nissan.micra"),
        ]

        monitor      = SpeedMonitor(world, ego)
        status_every = int(2.0 / FIXED_DELTA_SECONDS)
        tick_count   = 0
        print("\n[Step 4] Running. Ctrl+C to stop.\n")

        while True:
            world.tick()
            if not is_alive(ego):
                print("[Step 4] Ego lost."); break

            t, ego_spd = monitor.tick(FIXED_DELTA_SECONDS)
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