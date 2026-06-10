"""
Scenario 1 - Step 4: A1-A4 Cut-in Events
"""

from typing import Optional
import carla
import time
import math

CARLA_HOST          = "localhost"
CARLA_PORT          = 2000
FIXED_DELTA_SECONDS = 0.05
TM_PORT             = 8001

EGO_SPAWN_INDEX     = 28
ROAD_SPEED_LIMIT    = 120.0

SPEED_CRUISE        = 100.0
SPEED_CATCHUP       = 120.0
SPEED_SLOW          = 70.0
AHEAD_THRESHOLD_M   = 25.0

SPEED_TARGET_LOW    = 100.0
SPEED_TARGET_HIGH   = 105.0
SPEED_WARN_LOW      = 90.0
SLOW_WARN_DELAY_S   = 20.0

COLOR_OK     = carla.Color(r=0,   g=220, b=0)
COLOR_WARN   = carla.Color(r=255, g=180, b=0)
COLOR_DANGER = carla.Color(r=255, g=40,  b=40)
COLOR_INFO   = carla.Color(r=100, g=200, b=255)

# ──────────────────────────────────────────────────────────────────────────────

def pct(kmh):
    return ((ROAD_SPEED_LIMIT - kmh) / ROAD_SPEED_LIMIT) * 100.0

def is_alive(actor):
    try:    return actor is not None and actor.is_alive
    except: return False

def safe_destroy(actor):
    try:
        if is_alive(actor):
            actor.set_autopilot(False)
            actor.destroy()
    except: pass

def kmh_of(actor):
    if not is_alive(actor): return 0.0
    try:
        v = actor.get_velocity()
        return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
    except: return 0.0

def dist_ahead(ego, npc):
    if not is_alive(ego) or not is_alive(npc): return 0.0
    try:
        ego_tf = ego.get_transform()
        fwd    = ego_tf.get_forward_vector()
        delta  = npc.get_transform().location - ego_tf.location
        return delta.x * fwd.x + delta.y * fwd.y
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
    DReyeVR_vehicle = None
    ego_vehicles = list(world.get_actors().filter("harplab.dreyevr_vehicle.*"))
    if len(ego_vehicles) >= 1:
        DReyeVR_vehicle = ego_vehicles[0]
    else:
        model = "harplab.dreyevr_vehicle.model3"
        print(f'No EgoVehicle found, spawning one: "{model}"')
        bp = world.get_blueprint_library().find(model)
        transform = world.get_map().get_spawn_points()[0]
        DReyeVR_vehicle = world.spawn_actor(bp, transform)
    return DReyeVR_vehicle

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
            carla.Rotation(pitch=float(pitch), yaw=float(yaw), roll=float(roll)))
        vehicle.set_simulate_physics(False)
        vehicle.set_transform(transform)
        time.sleep(0.1)
        vehicle.set_simulate_physics(True)
        return True
    except Exception as e:
        print(f"Respawn failed: {e}")
        try: vehicle.set_simulate_physics(True)
        except: pass
        return False


# ── NPC spawn ─────────────────────────────────────────────────────────────────
def spawn_npc_behind(world, ego, tm, behind_m=60.0, color="255,50,50"):
    town_map = world.get_map()
    ego_wp   = town_map.get_waypoint(
        ego.get_transform().location, project_to_road=True,
        lane_type=carla.LaneType.Driving)
    if ego_wp is None:
        print("[NPC] No ego waypoint."); return None

    behind_wps = ego_wp.previous(behind_m)
    if not behind_wps:
        print("[NPC] No behind waypoint."); return None
    wp = behind_wps[0]

    # Walk to lane -3
    for _ in range(6):
        if wp.lane_id == -3: break
        r = wp.get_right_lane()
        l = wp.get_left_lane()
        if r and r.lane_type == carla.LaneType.Driving: wp = r
        elif l and l.lane_type == carla.LaneType.Driving: wp = l
        else: break

    spawn_tf = wp.transform
    spawn_tf.location.z += 0.3

    blib = world.get_blueprint_library()
    bp   = blib.find("vehicle.tesla.model3")
    bp.set_attribute("role_name", "npc")
    if bp.has_attribute("color"):
        bp.set_attribute("color", color)

    npc = world.try_spawn_actor(bp, spawn_tf)
    if npc is None:
        print("[NPC] Spawn failed."); return None

    # No autopilot — fully manual waypoint control from start
    npc.set_autopilot(False)
    npc.set_simulate_physics(False)   # kinematic from spawn

    # Snap to correct waypoint
    spawn_wp = world.get_map().get_waypoint(
        spawn_tf.location, project_to_road=True,
        lane_type=carla.LaneType.Driving)
    if spawn_wp:
        npc.set_transform(spawn_wp.transform)

    print(f"[NPC] id={npc.id}  lane={wp.lane_id}  {behind_m:.0f}m behind")
    return npc


# ── Manual lane change ────────────────────────────────────────────────────────
def manual_lane_change(world, npc, target_lane=-2,
                       _state={}, duration_s=2.5):
    """
    Smoothly interpolate NPC laterally into target_lane over duration_s.
    Uses set_transform with physics disabled during the move,
    re-enables physics when done.
    Returns True once complete.
    """
    if not is_alive(npc):
        return True

    town_map = world.get_map()
    npc_id   = npc.id

    # Initialise state on first call
    if npc_id not in _state:
        # Find target lane waypoint
        npc_loc = npc.get_transform().location
        cur_wp  = town_map.get_waypoint(npc_loc, project_to_road=True,
                                         lane_type=carla.LaneType.Driving)
        if cur_wp is None:
            return False

        tgt_wp = cur_wp
        for _ in range(6):
            if tgt_wp.lane_id == target_lane: break
            l = tgt_wp.get_left_lane()
            r = tgt_wp.get_right_lane()
            if l and l.lane_type == carla.LaneType.Driving: tgt_wp = l
            elif r and r.lane_type == carla.LaneType.Driving: tgt_wp = r
            else: break

        if tgt_wp.lane_id != target_lane:
            print(f"[LC] Cannot find lane {target_lane}"); return False

        # Disable physics for smooth lateral move
        npc.set_simulate_physics(False)

        _state[npc_id] = {
            'start_loc': npc_loc,
            'start_rot': npc.get_transform().rotation,
            'tgt_y':     tgt_wp.transform.location.y,
            'tgt_x':     tgt_wp.transform.location.x,
            'elapsed':   0.0,
            'duration':  duration_s,
        }
        print(f"[LC] Starting smooth lane change → lane {target_lane}  "
              f"src_y={npc_loc.y:.1f}  tgt_y={tgt_wp.transform.location.y:.1f}")

    s = _state[npc_id]
    s['elapsed'] += FIXED_DELTA_SECONDS
    progress = min(s['elapsed'] / s['duration'], 1.0)

    # Smooth step interpolation (ease in/out)
    t = progress * progress * (3 - 2 * progress)

    # Get current forward position (NPC keeps moving forward)
    cur_tf  = npc.get_transform()
    # Interpolate only the lateral (Y) component
    new_y = s['start_loc'].y + t * (s['tgt_y'] - s['start_loc'].y)

    new_tf = carla.Transform(
        carla.Location(x=cur_tf.location.x,
                       y=new_y,
                       z=cur_tf.location.z),
        cur_tf.rotation)
    npc.set_transform(new_tf)

    if progress >= 1.0:
        # Re-enable physics once in target lane
        npc.set_simulate_physics(True)
        del _state[npc_id]
        print(f"[LC] Lane change COMPLETE → lane {target_lane}")
        return True

    return False

# ── Indicator ─────────────────────────────────────────────────────────────────
def flash_indicator(world, npc, duration_s=2.5):
    """Orange arrow on LEFT side of NPC."""
    if not is_alive(npc): return
    try:
        tf  = npc.get_transform()
        fwd = tf.get_forward_vector()
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


# ── HUD ───────────────────────────────────────────────────────────────────────
class HUD:
    def __init__(self, world, ego):
        self.world = world; self.ego = ego

    def update(self, spd, warning, t, label=""):
        if not is_alive(self.ego): return
        try:    loc = self.ego.get_transform().location
        except: return
        col = (COLOR_OK    if SPEED_TARGET_LOW <= spd <= SPEED_TARGET_HIGH else
               COLOR_DANGER if spd < SPEED_WARN_LOW or spd > SPEED_TARGET_HIGH
               else COLOR_WARN)
        lt = FIXED_DELTA_SECONDS * 2
        self.world.debug.draw_string(
            loc + carla.Location(z=5.0),
            f"Speed: {spd:.1f} km/h   T+{t:.0f}s",
            draw_shadow=True, color=col, life_time=lt, persistent_lines=False)
        if label:
            self.world.debug.draw_string(
                loc + carla.Location(z=6.5), label,
                draw_shadow=True, color=COLOR_INFO,
                life_time=lt, persistent_lines=False)
        if warning:
            self.world.debug.draw_string(
                loc + carla.Location(z=3.8), warning,
                draw_shadow=True, color=COLOR_DANGER,
                life_time=lt, persistent_lines=False)


# ── Speed monitor ─────────────────────────────────────────────────────────────
class SpeedMonitor:
    def __init__(self, world, ego):
        self.world = world; self.ego = ego
        self.hud   = HUD(world, ego)
        self.scenario_time = 0.0
        self.slow_since    = None
        self.warning       = ""

    def ego_kmh(self):
        if not is_alive(self.ego): return 0.0
        try:
            v = self.ego.get_velocity()
            return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        except: return 0.0

    def _warn(self, spd):
        if spd > SPEED_TARGET_HIGH:
            self.slow_since = None
            return f"! Too fast ({spd:.0f}) target {SPEED_TARGET_LOW:.0f}-{SPEED_TARGET_HIGH:.0f}"
        if spd < SPEED_WARN_LOW:
            if self.slow_since is None: self.slow_since = self.scenario_time
            elif self.scenario_time - self.slow_since >= SLOW_WARN_DELAY_S:
                return f"! Too slow ({spd:.0f}) reach {SPEED_TARGET_LOW:.0f}-{SPEED_TARGET_HIGH:.0f}"
        else:
            self.slow_since = None
        if SPEED_TARGET_LOW <= spd <= SPEED_TARGET_HIGH: return ""
        return self.warning

    def tick(self, dt, label=""):
        self.scenario_time += dt
        spd = self.ego_kmh()
        self.warning = self._warn(spd)
        self.hud.update(spd, self.warning, self.scenario_time, label)
        return self.scenario_time, spd


# ── A-Event state machine ─────────────────────────────────────────────────────
class AEvent:
    """
    States:
      waiting  : before start_t
      catchup  : NPC in lane 3, driving 120 km/h to get ahead of ego
      cutin    : NPC ahead, manually steering into lane 2
      decel    : NPC in lane 2, ramping 100->70 km/h over decel_time_s
      tail     : holding 70 km/h
      reaccel  : ramping back to 100 km/h
      done     : cruising at 100 km/h
    """
    def __init__(self, label, indicator, decel_time_s, start_t):
        self.label        = label
        self.indicator    = indicator
        self.decel_time_s = decel_time_s
        self.start_t      = start_t
        self.npc          = None
        self.state        = "waiting"
        self._state_t     = 0.0
        self._ind_done    = False
        self._cut_done    = False
        self._follow_wp   = None   # current waypoint for manual follow
        print(f"[{label}] Ready  indicator={'ON' if indicator else 'OFF'}  "
              f"decel={decel_time_s}s  starts t={start_t:.0f}s")

    def _set_state(self, state, t):
        print(f"[{self.label}] {self.state} -> {state}  T={t:.1f}s")
        self.state    = state
        self._state_t = t

    def _advance_waypoint(self, speed_kmh):
        """Move NPC along waypoints at speed_kmh. Call every tick after cutin."""
        if not is_alive(self.npc) or self._follow_wp is None:
            return
        try:
            dist = (speed_kmh / 3.6) * FIXED_DELTA_SECONDS
            nexts = self._follow_wp.next(dist)
            if nexts:
                self._follow_wp = nexts[0]
            tf = self._follow_wp.transform
            tf.location.z += 0.05
            self.npc.set_transform(tf)
        except Exception as e:
            print(f"[{self.label}] waypoint advance error: {e}")

    def update(self, world, ego, tm, t):
        try:
            return self._update(world, ego, tm, t)
        except Exception as e:
            print(f"[{self.label}] update() error: {e}")
            return f"{self.label} (error)"

    def _update(self, world, ego, tm, t):

        # ── Waiting ───────────────────────────────────────────────────
        if self.state == "waiting":
            if t >= self.start_t:
                self.npc = spawn_npc_behind(world, ego, tm, behind_m=60.0)
                if self.npc:
                    # Set initial follow waypoint
                    loc = self.npc.get_transform().location
                    wp  = world.get_map().get_waypoint(
                        loc, project_to_road=True,
                        lane_type=carla.LaneType.Driving)
                    self._follow_wp = wp
                self._set_state("catchup", t)
            return ""

        if not is_alive(self.npc):
            return f"{self.label} (NPC gone)"

        # ── Catchup ───────────────────────────────────────────────────
        if self.state == "catchup":
            ahead = dist_ahead(ego, self.npc)
            # Ramp speed: start at ego speed, gradually increase to CATCHUP
            catchup_elapsed = t - self._state_t
            ramp_progress   = min(catchup_elapsed / 5.0, 1.0)  # 5s ramp
            ego_spd         = kmh_of(ego) if is_alive(ego) else SPEED_CRUISE
            # Start at ego speed, ramp to SPEED_CATCHUP
            catchup_spd = ego_spd + ramp_progress * (SPEED_CATCHUP - ego_spd)
            # Use waypoint follow even during catchup for smoothness
            self._advance_waypoint(catchup_spd)
            if ahead >= AHEAD_THRESHOLD_M:
                if self.indicator:
                    flash_indicator(world, self.npc, duration_s=2.5)
                    print(f"[{self.label}] Indicator ON")
                self._set_state("cutin", t)
            return (f"{self.label} — catchup  "
                    f"npc={catchup_spd:.0f} km/h  ahead={ahead:.0f}m")

        # ── Cut-in: interpolate into lane 2, then respawn there ─────
        elif self.state == "cutin":
            done = manual_lane_change(world, self.npc, target_lane=-2)
            if done:
                try:
                    # Keep NPC fully manual — NO autopilot ever again
                    # We drive it via waypoints ourselves from here on
                    self.npc.set_autopilot(False)
                    self.npc.set_simulate_physics(False)
                    # Snap to lane 2 waypoint cleanly
                    loc = self.npc.get_transform().location
                    wp  = world.get_map().get_waypoint(
                        loc, project_to_road=True,
                        lane_type=carla.LaneType.Driving)
                    if wp and wp.lane_id != -2:
                        l = wp.get_left_lane()
                        if l and l.lane_type == carla.LaneType.Driving:
                            wp = l
                    if wp:
                        self._follow_wp = wp
                    print(f"[{self.label}] In lane 2 — waypoint follow mode")
                    self._set_state("decel", t)
                except Exception as e:
                    print(f"[{self.label}] Post-cutin error: {e}")
                    self._set_state("decel", t)
            return f"{self.label} — cutting in..."

        # ── Decel ─────────────────────────────────────────────────────
        elif self.state == "decel":
            elapsed  = t - self._state_t
            progress = min(elapsed / self.decel_time_s, 1.0)
            spd      = SPEED_CRUISE + progress * (SPEED_SLOW - SPEED_CRUISE)
            self._advance_waypoint(spd)
            if progress >= 1.0:
                self._set_state("tail", t)
            style = "rapid" if self.decel_time_s == 3 else "gradual"
            return f"{self.label} — decel ({style})  {spd:.0f} km/h"

        # ── Tail ──────────────────────────────────────────────────────
        elif self.state == "tail":
            self._advance_waypoint(SPEED_SLOW)
            if t - self._state_t >= 5.0:
                self._set_state("reaccel", t)
            return f"{self.label} — tail  {SPEED_SLOW:.0f} km/h"

        # ── Re-accel ──────────────────────────────────────────────────
        elif self.state == "reaccel":
            elapsed  = t - self._state_t
            progress = min(elapsed / 20.0, 1.0)
            spd      = SPEED_SLOW + progress * (SPEED_CRUISE - SPEED_SLOW)
            self._advance_waypoint(spd)
            if progress >= 1.0:
                self._set_state("done", t)
            return f"{self.label} — reaccel  {spd:.0f} km/h"

        # ── Done ──────────────────────────────────────────────────────
        elif self.state == "done":
            self._advance_waypoint(SPEED_CRUISE)
            # Destroy NPC once it is 300m ahead — prevents loop-around crash
            ahead = dist_ahead(ego, self.npc)
            if ahead > 300.0 or ahead < -50.0:
                print(f"[{self.label}] NPC too far ({ahead:.0f}m) — removing")
                safe_destroy(self.npc)
                self.npc = None
                return f"{self.label} — done"
            return f"{self.label} — done  cruising"

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
    ego           = None
    all_npcs      = []

    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(False)   # async mode — matches your setup
    tm.set_random_device_seed(42)
    tm.set_global_distance_to_leading_vehicle(8.0)

    try:
        ego = find_ego_vehicle(world)
        # precise_respawn(ego, location=(388.5, -154.7, 0.5), yaw=None)
        world.tick()

        beep(world, ego.get_transform().location)

        events = [
            AEvent("A1", indicator=True,  decel_time_s=3.0, start_t=60.0),
            AEvent("A2", indicator=True,  decel_time_s=6.0, start_t=170.0),
            AEvent("A3", indicator=False, decel_time_s=3.0, start_t=280.0),
            AEvent("A4", indicator=False, decel_time_s=6.0, start_t=390.0),
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
                ev_label = ev.update(world, ego, tm, t)
                if ev_label:
                    label = ev_label
                if ev.npc and ev.npc not in all_npcs and is_alive(ev.npc):
                    all_npcs.append(ev.npc)

            monitor.hud.update(ego_spd, monitor.warning, t, label)

            tick_count += 1
            if tick_count % status_every == 0:
                print(f"  T={t:>6.1f}s  ego={ego_spd:>5.1f} km/h  [{label}]")

            if t >= 500.0:
                print("\n[Step 4] Complete at T=500s.")
                break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[Step 4] Stopped.")

    finally:
        for npc in all_npcs:
            safe_destroy(npc)
        try:
            s = world.get_settings()
            s.synchronous_mode    = False
            s.fixed_delta_seconds = None
            world.apply_settings(s)
        except: pass
        print("[Step 4] Done.")


if __name__ == "__main__":
    main()