"""
Scenario 2 - Step 3: Merge Blocking Event
==========================================
Timeline:
  0-100s   : Adaptation — ego drives ramp at 30 km/h
  65s      : Voice prompt "Turn on left signal and merge into main lane"
  100-180s : Merge attempt — highway vehicles block with <1s headway
  120s+    : Horn pressure from behind every 5s
  160-180s : Ego merges successfully (gap finally opens)
  180-220s : Stabilization on highway at 100 km/h

Highway vehicles (blockers):
  - 3-4 vehicles in lane -4 (rightmost) of road 36
  - Maintain tight headway (<1s = <28m at 100 km/h) past merge point
  - Continue until ~160s then leave a gap for ego to merge

Horn vehicle:
  - Spawns behind ego on ramp at t=120s
  - Honks (debug sound flash) every 5s
"""

import carla, time, math, random

CARLA_HOST = "localhost"
CARLA_PORT = 2000
FDS        = 0.05
TM_PORT    = 8001

# Ego starts 20m before the ramp connects to highway (road 863, lane -1)
# Road 863 start is at x=371.7, y=-168.5 — walk back 20m along approach
EGO_SPAWN      = carla.Location(x=361.7, y=-168.5, z=0.3)
MERGE_LOCATION = carla.Location(x=381.9, y=-157.2, z=0.3)
# Obstacle: construction zone 30m ahead of merge point on highway lane -4
OBSTACLE_LOCATION = carla.Location(x=381.9, y=-107.2, z=0.3)
MERGE_ROAD     = 36
MERGE_LANE     = -4

RAMP_SPEED_KMH    = 30.0
HIGHWAY_SPEED_KMH = 100.0
BLOCKER_HEADWAY_M = 20.0   # <1s headway at 100 km/h = 27.8m, use 20m to be tight

SPEED_LOW  = 100.0; SPEED_HIGH = 105.0
WARN_LOW   = 90.0;  WARN_DELAY = 20.0

COLOR_OK     = carla.Color(0, 220, 0)
COLOR_WARN   = carla.Color(255, 180, 0)
COLOR_DANGER = carla.Color(255, 40, 40)
COLOR_INFO   = carla.Color(100, 200, 255)
COLOR_RATING = carla.Color(255, 255, 0)

def pct(kmh, limit=120.0): return ((limit - kmh) / limit) * 100.0
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
def dist_to(a, loc):
    if not is_alive(a): return 999.0
    try:
        p = a.get_transform().location
        return math.sqrt((p.x-loc.x)**2 + (p.y-loc.y)**2)
    except: return 999.0


# ── Connect ───────────────────────────────────────────────────────────────────
def connect(host, port, map_name="Town04"):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world  = client.load_world(map_name)
    if map_name not in world.get_map().name:
        client.load_world(map_name)
        time.sleep(3)
        world = client.get_world()
    world.set_weather(carla.WeatherParameters(
        cloudiness=10, precipitation=0,
        sun_altitude_angle=45, sun_azimuth_angle=200,
        fog_density=0, wetness=0))
    return client, world

def find_ego_vehicle(world):
    v = list(world.get_actors().filter("harplab.dreyevr_vehicle.*"))
    if v: return v[0]
    bp = world.get_blueprint_library().find("harplab.dreyevr_vehicle.model3")
    return world.spawn_actor(bp, world.get_map().get_spawn_points()[0])

def precise_respawn(vehicle, location, yaw=None):
    try:
        if not isinstance(location, carla.Location):
            location = carla.Location(*location)
        if yaw is None:
            wp  = vehicle.get_world().get_map().get_waypoint(
                location, project_to_road=True)
            yaw = wp.transform.rotation.yaw if wp else 0.0
        vehicle.set_simulate_physics(False)
        vehicle.set_transform(carla.Transform(
            location, carla.Rotation(yaw=float(yaw))))
        time.sleep(0.1)
        vehicle.set_simulate_physics(True)
        return True
    except Exception as e:
        print(f"Respawn failed: {e}")
        try: vehicle.set_simulate_physics(True)
        except: pass
        return False


# ── Spawn congested highway traffic on all lanes ─────────────────────────────
def spawn_highway_congestion(world, tm, num_per_lane=6):
    """
    Spawn slow congested vehicles on lanes -1, -2, -3 of road 36.
    Each lane is found by direct road+lane waypoint lookup,
    not by walking from another lane (which can miss lanes).
    """
    town_map  = world.get_map()
    all_wps   = town_map.generate_waypoints(5.0)
    all_npcs  = []
    blib      = world.get_blueprint_library()
    models    = [
        "vehicle.audi.tt", "vehicle.chevrolet.impala",
        "vehicle.ford.mustang", "vehicle.lincoln.mkz_2017",
        "vehicle.toyota.prius", "vehicle.dodge.charger_2020",
        "vehicle.mini.cooper_s", "vehicle.seat.leon",
    ]

    # Get reference s-value near merge on road 36
    merge_ref = town_map.get_waypoint(MERGE_LOCATION, project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
    if not merge_ref: return []
    ref_s = merge_ref.s

    for target_lane in [-1, -2, -3]:
        # Find waypoints on road 36 in this lane near merge point
        lane_wps = [wp for wp in all_wps
                    if wp.road_id == MERGE_ROAD
                    and wp.lane_id == target_lane]
        if not lane_wps:
            print(f"[Congestion] Lane {target_lane}: no waypoints found on road 36")
            continue

        # Sort by s and find the one closest to merge s-value
        lane_wps.sort(key=lambda w: abs(w.s - ref_s))
        base_wp = lane_wps[0]

        spawned = 0
        for i in range(num_per_lane):
            # Cover 100m behind to 150m ahead of merge point
            # so participant sees solid congestion in both directions
            offset_m = -100.0 + i * (250.0 / num_per_lane)
            if offset_m < 0:
                wps = base_wp.previous(abs(offset_m))
            else:
                wps = base_wp.next(offset_m)
            if not wps: continue
            sp_wp = wps[0]

            # Verify still on road 36 in correct lane
            if sp_wp.road_id != MERGE_ROAD or sp_wp.lane_id != target_lane:
                continue

            model = random.choice(models)
            bp    = blib.filter(model)
            if not bp: bp = blib.filter("vehicle.tesla.model3")
            if not bp: continue
            bp = bp[0]
            bp.set_attribute("role_name", "congestion")

            tf = carla.Transform(
                carla.Location(x=sp_wp.transform.location.x,
                               y=sp_wp.transform.location.y,
                               z=sp_wp.transform.location.z + 0.3),
                sp_wp.transform.rotation)

            npc = world.try_spawn_actor(bp, tf)
            if not npc: continue

            npc.set_autopilot(True, TM_PORT)
            tm.auto_lane_change(npc, False)
            tm.ignore_lights_percentage(npc, 100)
            tm.ignore_signs_percentage(npc, 100)
            tm.ignore_vehicles_percentage(npc, 0)
            tm.vehicle_percentage_speed_difference(
                npc, pct(random.uniform(10.0, 20.0)))
            all_npcs.append(npc)
            spawned += 1

        print(f"[Congestion] Lane {target_lane}: {spawned} vehicles spawned")

    return all_npcs


# ── Spawn stationary obstacle ────────────────────────────────────────────────
def spawn_obstacle(world):
    """
    Spawn construction cones on highway lane -4 at merge point.
    Uses static.prop.constructioncone (or closest available cone prop).
    Spawns 3 cones side by side across the merge lane.
    """
    town_map    = world.get_map()
    obstacle_wp = town_map.get_waypoint(
        OBSTACLE_LOCATION, project_to_road=True,
        lane_type=carla.LaneType.Driving)
    if not obstacle_wp:
        print("[Obstacle] No waypoint found"); return []

    # Ensure lane -4
    wp = obstacle_wp
    for _ in range(4):
        if wp.lane_id == MERGE_LANE: break
        r = wp.get_right_lane()
        if r and r.lane_type == carla.LaneType.Driving: wp = r
        else: break

    blib = world.get_blueprint_library()

    # Try common cone blueprint names
    # Confirmed available cone blueprints in this CARLA build
    cone_bp     = blib.find("static.prop.trafficcone01")
    barrier_bp  = blib.find("static.prop.streetbarrier")
    warning_bp  = blib.find("static.prop.trafficwarning")

    loc   = wp.transform.location
    rot   = wp.transform.rotation
    right = wp.transform.get_right_vector()
    fwd   = wp.transform.get_forward_vector()
    spawned = []

    # Row 1: 3 traffic cones spread across lane (at obstacle location)
    for off in [-1.2, 0.0, 1.2]:
        cone_loc = carla.Location(
            x=loc.x + right.x*off,
            y=loc.y + right.y*off,
            z=loc.z + 0.05)
        actor = world.try_spawn_actor(cone_bp, carla.Transform(cone_loc, rot))
        if actor: spawned.append(actor)

    # Row 2: street barrier 3m ahead of cones
    for off in [-0.8, 0.8]:
        barrier_loc = carla.Location(
            x=loc.x + fwd.x*3.0 + right.x*off,
            y=loc.y + fwd.y*3.0 + right.y*off,
            z=loc.z + 0.05)
        actor = world.try_spawn_actor(barrier_bp, carla.Transform(barrier_loc, rot))
        if actor: spawned.append(actor)

    # Traffic warning sign 2m behind cones
    warn_loc = carla.Location(
        x=loc.x - fwd.x*2.0,
        y=loc.y - fwd.y*2.0,
        z=loc.z + 0.05)
    actor = world.try_spawn_actor(warning_bp, carla.Transform(warn_loc, rot))
    if actor: spawned.append(actor)

    print(f"[Obstacle] {len(spawned)} props placed (cones + barriers + warning) "
          f"at x={loc.x:.1f}  y={loc.y:.1f}  lane={wp.lane_id}")
    return spawned


# ── Spawn highway blocker vehicles ────────────────────────────────────────────
def spawn_blockers(world, tm, num=1):
    """
    Spawn vehicles on road 36 lane -4 (highway rightmost lane),
    spaced 20m apart, starting ~200m before merge so they arrive
    continuously past the merge zone blocking ego.
    """
    town_map  = world.get_map()
    merge_wp  = town_map.get_waypoint(MERGE_LOCATION, project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
    if not merge_wp:
        print("[Blockers] No merge waypoint found"); return []

    # Walk to lane -4 at merge
    wp = merge_wp
    for _ in range(4):
        if wp.lane_id == MERGE_LANE: break
        r = wp.get_right_lane()
        if r and r.lane_type == carla.LaneType.Driving: wp = r
        else: break

    blockers = []
    blib     = world.get_blueprint_library()
    models   = ["vehicle.audi.tt", "vehicle.chevrolet.impala",
                "vehicle.ford.mustang", "vehicle.lincoln.mkz_2017"]

    for i in range(num):
        # Spread blockers from 100m before to 150m after merge
        # so lane -4 is congested on both sides of the merge point
        behind_m = 100.0 - i * 40.0   # negative = ahead of merge
        if behind_m < 0:
            # Some blockers are AHEAD of merge point
            pass
        behind_m = max(behind_m, -150.0)
        if behind_m >= 0:
            prevs = wp.previous(behind_m)
        else:
            prevs = wp.next(abs(behind_m))
        if not prevs: continue
        spawn_wp = prevs[0]
        # Ensure lane -4
        for _ in range(4):
            if spawn_wp.lane_id == MERGE_LANE: break
            r = spawn_wp.get_right_lane()
            if r and r.lane_type == carla.LaneType.Driving: spawn_wp = r
            else: break

        bp = blib.filter(models[i % len(models)])
        if not bp: continue
        bp = bp[0]
        bp.set_attribute("role_name", "blocker")

        tf = carla.Transform(
            carla.Location(x=spawn_wp.transform.location.x,
                           y=spawn_wp.transform.location.y,
                           z=spawn_wp.transform.location.z + 0.3),
            spawn_wp.transform.rotation)

        npc = world.try_spawn_actor(bp, tf)
        if not npc:
            print(f"[Blockers] Spawn {i} failed"); continue

        # TM autopilot — stay in lane -4, tight following distance
        npc.set_autopilot(True, TM_PORT)
        tm.auto_lane_change(npc, False)
        tm.ignore_lights_percentage(npc, 100)
        tm.ignore_signs_percentage(npc, 100)
        tm.vehicle_percentage_speed_difference(npc, pct(HIGHWAY_SPEED_KMH))
        tm.set_global_distance_to_leading_vehicle(BLOCKER_HEADWAY_M / 5.0)
        blockers.append(npc)
        print(f"[Blocker {i}] id={npc.id}  lane={spawn_wp.lane_id}  "
              f"{behind_m:.0f}m before merge")

    return blockers


# ── Horn simulation ───────────────────────────────────────────────────────────
class HornVehicle:
    """
    Spawns behind ego on ramp at t=120s.
    Flashes a red point + debug string every 5s to simulate horn.
    """
    def __init__(self):
        self.npc         = None
        self.ctrl        = None
        self.spawned     = False
        self._last_horn  = 0.0
        self.horn_interval = 5.0

    def spawn(self, world, ego, tm):
        """Spawn horn vehicle 20m behind ego on ramp."""
        town_map = world.get_map()
        ego_wp   = town_map.get_waypoint(
            ego.get_transform().location, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        if not ego_wp: return

        prevs = ego_wp.previous(20.0)
        if not prevs: return
        spawn_wp = prevs[0]

        blib = world.get_blueprint_library()
        bp   = blib.filter("vehicle.dodge.charger_2020")
        if not bp: bp = blib.filter("vehicle.tesla.model3")
        if not bp: return
        bp = bp[0]
        bp.set_attribute("role_name", "horn")

        tf = carla.Transform(
            carla.Location(x=spawn_wp.transform.location.x,
                           y=spawn_wp.transform.location.y,
                           z=spawn_wp.transform.location.z + 0.3),
            spawn_wp.transform.rotation)

        self.npc = world.try_spawn_actor(bp, tf)
        if not self.npc:
            print("[Horn] Spawn failed"); return

        # Follow ego on ramp
        self.npc.set_autopilot(True, TM_PORT)
        tm.auto_lane_change(self.npc, False)
        tm.ignore_lights_percentage(self.npc, 100)
        tm.vehicle_percentage_speed_difference(self.npc, pct(RAMP_SPEED_KMH))
        tm.set_global_distance_to_leading_vehicle(5.0)
        self.spawned = True
        print(f"[Horn] Vehicle spawned id={self.npc.id}  20m behind ego")

    def update(self, world, ego, t):
        """Flash horn indicator every 5s."""
        if not self.spawned or not is_alive(self.npc): return
        if t - self._last_horn >= self.horn_interval:
            self._last_horn = t
            # Flash red point above horn vehicle (simulates horn sound)
            try:
                loc = self.npc.get_transform().location
                world.debug.draw_point(
                    loc + carla.Location(z=2.5),
                    size=0.5, color=carla.Color(255, 0, 0),
                    life_time=0.8, persistent_lines=False)
                world.debug.draw_string(
                    loc + carla.Location(z=3.5),
                    "📯 HOOONK!",
                    draw_shadow=True,
                    color=carla.Color(255, 50, 50),
                    life_time=1.5, persistent_lines=False)
                print(f"  [Horn] HONK at T={t:.0f}s")
            except: pass

    def destroy(self):
        safe_destroy(self.npc)


# ── Merge detector ────────────────────────────────────────────────────────────
def ego_has_merged(world, ego):
    """Returns True when ego is on the highway (road 36)."""
    if not is_alive(ego): return False
    try:
        wp = world.get_map().get_waypoint(
            ego.get_transform().location, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        return wp is not None and wp.road_id == MERGE_ROAD
    except: return False

def ego_lane_on_highway(world, ego):
    if not is_alive(ego): return None
    try:
        wp = world.get_map().get_waypoint(
            ego.get_transform().location, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        return wp.lane_id if wp else None
    except: return None


# ── HUD ───────────────────────────────────────────────────────────────────────
class HUD:
    def __init__(self, world, ego):
        self.world = world; self.ego = ego

    def update(self, spd, warning, t, label="", ramp_mode=True):
        if not is_alive(self.ego): return
        try: loc = self.ego.get_transform().location
        except: return

        # On ramp: speed target is 30 km/h, not 100-105
        if ramp_mode:
            col = COLOR_OK if 25 <= spd <= 40 else COLOR_WARN
        else:
            col = (COLOR_OK if SPEED_LOW <= spd <= SPEED_HIGH else
                   COLOR_DANGER if spd < WARN_LOW or spd > SPEED_HIGH
                   else COLOR_WARN)

        lt = FDS * 2
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
        self.ramp_mode     = True   # True until ego merges

    def ego_kmh(self):
        if not is_alive(self.ego): return 0.0
        try:
            v = self.ego.get_velocity()
            return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        except: return 0.0

    def _warn(self, spd):
        # Only apply 100-105 km/h warning AFTER merge
        if self.ramp_mode: return ""
        if spd > SPEED_HIGH:
            self.slow_since = None
            return "Please drive at 100-105 km/h. Your current speed is too fast."
        if spd < WARN_LOW:
            if self.slow_since is None: self.slow_since = self.scenario_time
            elif self.scenario_time - self.slow_since >= WARN_DELAY:
                return ("Please drive at 100-105 km/h. "
                        "Your current speed is too low. "
                        "Please accelerate to the target speed.")
        else: self.slow_since = None
        if SPEED_LOW <= spd <= SPEED_HIGH: return ""
        return self.warning

    def tick(self, dt, label=""):
        self.scenario_time += dt
        spd = self.ego_kmh()
        self.warning = self._warn(spd)
        self.hud.update(spd, self.warning, self.scenario_time,
                        label, ramp_mode=self.ramp_mode)
        return self.scenario_time, spd


def show_prompt(world, ego, msg, duration_s=12.0):
    """Display instruction/rating prompt above ego."""
    if not is_alive(ego): return
    try:
        loc = ego.get_transform().location
        world.debug.draw_string(
            loc + carla.Location(z=8.0), msg,
            draw_shadow=True, color=COLOR_RATING,
            life_time=duration_s, persistent_lines=False)
        print(f"\n[PROMPT] {msg}\n")
    except: pass

def beep(world, loc):
    world.debug.draw_point(loc + carla.Location(z=2.5),
        size=0.8, color=carla.Color(255, 255, 255), life_time=0.5)
    print("[BEEP] *** t=0 scenario 2 start ***")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    client, world = connect(CARLA_HOST, CARLA_PORT)
    all_npcs = []

    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(False)
    tm.set_random_device_seed(42)

    try:
        ego = find_ego_vehicle(world)
        precise_respawn(ego, EGO_SPAWN)
        world.tick()
        beep(world, ego.get_transform().location)

        # Spawn construction cones 10m ahead on highway lane -4
        cones = spawn_obstacle(world)
        all_npcs.extend(cones)
        world.tick()

        # Spawn congested traffic on lanes -1, -2, -3 (visible wall of traffic)
        cong_npcs = spawn_highway_congestion(world, tm, num_per_lane=6)
        all_npcs.extend(cong_npcs)
        world.tick()
        print(f"[Congestion] {len(cong_npcs)} vehicles on lanes -1/-2/-3")

        # Spawn highway blockers in lane -4 (merge lane, very tight)
        blockers = spawn_blockers(world, tm, num=6)
        all_npcs.extend(blockers)
        world.tick()

        horn_vehicle  = HornVehicle()
        monitor       = SpeedMonitor(world, ego)
        status_every  = int(2.0 / FDS)
        tick_count    = 0

        # State flags
        merge_prompt_shown  = False
        horn_spawned        = False
        merged              = False
        merge_time          = None
        gap_opened          = False
        prompts_shown       = set()

        prompt_schedule = {
            65.0:  "Turn on the left signal and merge into the main lane.",
            185.0: "How much anger or frustration did you feel\ndue to the merging refusal situation?",
            220.0: "Maintain a safe distance and follow the traffic speed.",
        }

        print("\n[Step 3] Running. Ctrl+C to stop.\n")

        while True:
            world.tick()
            if not is_alive(ego):
                print("[Step 3] Ego lost."); break

            t, ego_spd = monitor.tick(FDS)

            # Check if ego has merged
            on_highway = ego_has_merged(world, ego)
            if on_highway and not merged:
                merged     = True
                merge_time = t
                monitor.ramp_mode = False
                print(f"[Merge] ✓ Ego merged onto highway at T={t:.1f}s  "
                      f"lane={ego_lane_on_highway(world, ego)}")

            # ── 0-100s: Adaptation on ramp ────────────────────────────
            if t < 100.0:
                label = "Merge zone — wait for gap, obstacle ahead in lane -4"

            # ── 100-160s: Blocking phase ──────────────────────────────
            elif 100.0 <= t < 160.0:
                label = "Merge zone — highway vehicles blocking"
                # Keep blockers tight
                for b in blockers:
                    if is_alive(b):
                        tm.vehicle_percentage_speed_difference(
                            b, pct(HIGHWAY_SPEED_KMH))

                # Spawn horn vehicle at t=120s
                if t >= 120.0 and not horn_spawned:
                    horn_vehicle.spawn(world, ego, tm)
                    horn_spawned = True

            # ── 160-180s: Gap opens — ego can merge ───────────────────
            elif 160.0 <= t < 180.0:
                label = "Gap opening — merge now!"
                if not gap_opened:
                    # Speed up first blocker to create a gap
                    if blockers and is_alive(blockers[0]):
                        tm.vehicle_percentage_speed_difference(
                            blockers[0], pct(130.0))   # accelerate away
                    gap_opened = True
                    print(f"[Merge] Gap opened at T={t:.1f}s — "
                          f"first blocker accelerating away")

            # ── 180-220s: Stabilization ───────────────────────────────
            elif 180.0 <= t < 220.0:
                label = "Stabilization — maintain 100-105 km/h"
                # Gradually restore blockers to normal speed
                for b in blockers:
                    if is_alive(b):
                        tm.vehicle_percentage_speed_difference(
                            b, pct(HIGHWAY_SPEED_KMH))

            elif t >= 220.0:
                label = "Highway — proceed to congestion phase"

            # Horn updates
            if horn_spawned:
                horn_vehicle.update(world, ego, t)

            # Rating/instruction prompts
            for pt, msg in prompt_schedule.items():
                if t >= pt and pt not in prompts_shown:
                    show_prompt(world, ego, msg)
                    prompts_shown.add(pt)

            tick_count += 1
            if tick_count % status_every == 0:
                merged_str = f"✓ merged lane={ego_lane_on_highway(world, ego)}" \
                             if merged else "ramp"
                print(f"  T={t:>6.1f}s  ego={ego_spd:>5.1f} km/h  "
                      f"status={merged_str}  [{label[:35]}]")

            monitor.hud.update(ego_spd, monitor.warning, t, label,
                               ramp_mode=not merged)

            if t >= 220.0 and merged:
                print("\n[Step 3] Merge phase complete. "
                      "Proceeding to Step 4 — Congestion.\n")
                break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[Step 3] Stopped.")
    finally:
        horn_vehicle.destroy()
        for npc in all_npcs: safe_destroy(npc)
        try:
            s = world.get_settings()
            s.synchronous_mode = False
            s.fixed_delta_seconds = None
            world.apply_settings(s)
        except: pass
        print("[Step 3] Done.")

if __name__ == "__main__":
    main()