"""
Scenario 2 - Step 3: Merge Blocking Event
==========================================
  0-100s  : Adaptation on ramp
  65s     : "Turn on left signal and merge"
  100-160s: All lanes congested 10-20 km/h — ego blocked from merging
  120s    : Horn vehicle spawns behind ego, honks every 5s
  160-180s: Gap opens in lane -3 — ego can merge
  180-220s: Stabilization on highway

Congestion: lanes -1, -2, -3 using NPCController (same as Scenario 1 queue).
No lane -4 blockers needed. Merge is into lane -3.
"""

import carla, time, math, random

CARLA_HOST = "localhost"
CARLA_PORT = 2000
FDS        = 0.05
TM_PORT    = 8001

EGO_SPAWN         = carla.Location(x=361.7, y=-168.5, z=0.3)
MERGE_LOCATION    = carla.Location(x=381.9, y=-157.2, z=0.3)
OBSTACLE_LOCATION = carla.Location(x=385.4, y=-107.2, z=0.3)
MERGE_ROAD        = 36
MERGE_LANE        = -3

SPEED_CONG_MIN = 10.0
SPEED_CONG_MAX = 20.0
SPEED_LOW      = 100.0; SPEED_HIGH = 105.0
WARN_LOW       = 90.0;  WARN_DELAY = 20.0

COLOR_OK     = carla.Color(0, 220, 0)
COLOR_WARN   = carla.Color(255, 180, 0)
COLOR_DANGER = carla.Color(255, 40, 40)
COLOR_INFO   = carla.Color(100, 200, 255)
COLOR_RATING = carla.Color(255, 255, 0)

# ── Utilities ─────────────────────────────────────────────────────────────────
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
def pct(kmh, limit=120.0): return ((limit - kmh) / limit) * 100.0


# ── Connect ───────────────────────────────────────────────────────────────────
def connect(host, port, map_name="Town04"):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world  = client.load_world(map_name)
    if map_name not in world.get_map().name:
        client.load_world(map_name); time.sleep(3)
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


# ── NPC Controller (identical to Scenario 1) ─────────────────────────────────
class NPCController:
    MIN_LOOKAHEAD = 3.0; MAX_LOOKAHEAD = 10.0
    KP_STEER = 1.2;      MAX_STEER     = 0.6

    def __init__(self, world, npc, target_lane):
        self.world = world; self.npc = npc; self.target_lane = target_lane

    def change_lane(self, lane_id):
        self.target_lane = lane_id

    def in_target_lane(self):
        if not is_alive(self.npc): return False
        try:
            wp = self.world.get_map().get_waypoint(
                self.npc.get_transform().location, project_to_road=True,
                lane_type=carla.LaneType.Driving)
            return wp is not None and wp.lane_id == self.target_lane
        except: return False

    def _lookahead(self, spd):
        t = max(0.0, min(1.0, spd / 100.0))
        return self.MIN_LOOKAHEAD + t * (self.MAX_LOOKAHEAD - self.MIN_LOOKAHEAD)

    def _closest_ahead(self, max_dist=25.0):
        if not is_alive(self.npc): return None, None
        try:
            tf  = self.npc.get_transform()
            loc = tf.location; fwd = tf.get_forward_vector()
            town_map = self.world.get_map()
            best_d = max_dist; best_a = None
            for a in self.world.get_actors().filter("vehicle.*"):
                if not is_alive(a) or a.id == self.npc.id: continue
                d   = a.get_transform().location - loc
                dot = d.x*fwd.x + d.y*fwd.y
                if dot <= 0 or dot > max_dist: continue
                wp = town_map.get_waypoint(a.get_transform().location,
                                            project_to_road=True,
                                            lane_type=carla.LaneType.Driving)
                if wp and wp.lane_id == self.target_lane and dot < best_d:
                    best_d = dot; best_a = a
            return (best_d, best_a) if best_a else (None, None)
        except: return None, None

    def tick(self, target_kmh):
        if not is_alive(self.npc): return
        try:
            tf  = self.npc.get_transform()
            loc = tf.location; fwd = tf.get_forward_vector()
            spd = kmh_of(self.npc)

            # Following distance
            d_ahead, leader = self._closest_ahead()
            if d_ahead is not None:
                if d_ahead < 4.0:
                    target_kmh = max(0.0, target_kmh * 0.2)
                elif d_ahead < 10.0:
                    ratio = (d_ahead - 4.0) / 6.0
                    target_kmh = max(kmh_of(leader)*ratio if leader else 0.0, 2.0)

            lookahead = self._lookahead(spd)
            town_map  = self.world.get_map()
            look = carla.Location(x=loc.x+fwd.x*lookahead,
                                  y=loc.y+fwd.y*lookahead, z=loc.z)
            wp = town_map.get_waypoint(look, project_to_road=True,
                                        lane_type=carla.LaneType.Driving)
            if not wp:
                wp = town_map.get_waypoint(loc, project_to_road=True,
                                            lane_type=carla.LaneType.Driving)
            if not wp: return

            # Junction: follow road naturally
            cur_wp = town_map.get_waypoint(loc, project_to_road=True,
                                            lane_type=carla.LaneType.Driving)
            if cur_wp and cur_wp.is_junction:
                nexts = cur_wp.next(lookahead)
                tgt   = nexts[0] if nexts else cur_wp
            else:
                tgt = wp
                for _ in range(6):
                    if tgt.lane_id == self.target_lane: break
                    if abs(self.target_lane) > abs(tgt.lane_id):
                        r = tgt.get_right_lane()
                        if r and r.lane_type == carla.LaneType.Driving: tgt = r
                        else: break
                    else:
                        l = tgt.get_left_lane()
                        if l and l.lane_type == carla.LaneType.Driving: tgt = l
                        else: break

            to  = carla.Vector3D(tgt.transform.location.x - loc.x,
                                  tgt.transform.location.y - loc.y, 0.0)
            mag = math.sqrt(to.x**2 + to.y**2)
            if mag > 0.001: to.x /= mag; to.y /= mag
            cross = fwd.x*to.y - fwd.y*to.x
            steer = max(-self.MAX_STEER, min(self.MAX_STEER, self.KP_STEER*cross))

            err = target_kmh - spd
            if target_kmh <= 0.0: th=0.0; br=0.8
            elif err > 0:         th=min(1.0,err/20.0); br=0.0
            else:                 th=0.0; br=min(1.0,abs(err)/10.0)

            self.npc.apply_control(carla.VehicleControl(
                throttle=float(th), steer=float(steer),
                brake=float(br), hand_brake=False))
        except Exception as e:
            print(f"[NPCCtrl] {e}")


# ── Congestion vehicle (same pattern as Scenario 1 CongestionQueueVehicle) ───
class CongestionVehicle:
    def __init__(self, world, npc, lane_id):
        self.npc   = npc
        self.ctrl  = NPCController(world, npc, lane_id)
        self.alive = True

    def update(self, target_speed):
        if not is_alive(self.npc):
            self.alive = False; return
        self.ctrl.tick(target_speed)


# ── Spawn congestion on lanes -1, -2, -3 (same logic as Scenario 1 queue) ────
def spawn_congestion_lanes(world, num_per_lane=8):
    """
    Spawn congestion vehicles on road 36 lanes -1, -2, -3.
    Each lane independently found by direct road+lane query.
    Vehicles span 100m before to 150m after merge point (12m spacing).
    Physics ON, NPCController — same as Scenario 1 queue.
    """
    town_map  = world.get_map()
    all_wps   = town_map.generate_waypoints(5.0)
    all_vehs  = []
    blib      = world.get_blueprint_library()
    models    = [
        "vehicle.audi.tt",          "vehicle.chevrolet.impala",
        "vehicle.ford.mustang",     "vehicle.lincoln.mkz_2017",
        "vehicle.toyota.prius",     "vehicle.dodge.charger_2020",
        "vehicle.mini.cooper_s",    "vehicle.seat.leon",
    ]

    # Reference waypoint near merge on road 36
    merge_ref = town_map.get_waypoint(MERGE_LOCATION, project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
    if not merge_ref:
        print("[Congestion] No merge reference waypoint"); return []
    ref_s = merge_ref.s

    for target_lane in [-1, -2, -3]:
        # Find all waypoints on road 36 in this lane
        lane_wps = [wp for wp in all_wps
                    if wp.road_id == MERGE_ROAD and wp.lane_id == target_lane]
        if not lane_wps:
            print(f"[Congestion] Lane {target_lane}: no wps on road {MERGE_ROAD}")
            continue

        # Pick the waypoint closest to merge s-value as anchor
        lane_wps.sort(key=lambda w: abs(w.s - ref_s))
        anchor = lane_wps[0]

        # Walk back 200m from anchor to get queue start
        start_wps = anchor.previous(200.0)
        cur_wp    = start_wps[0] if start_wps else anchor

        spawned = 0
        for i in range(num_per_lane):
            if i > 0:
                # Step 6m forward — tight congestion spacing
                nexts = cur_wp.next(6.0)
                if not nexts: break
                cur_wp = nexts[0]

            # Skip if waypoint drifted off road 36 or wrong lane
            if cur_wp.road_id != MERGE_ROAD: continue
            if cur_wp.lane_id != target_lane:
                continue   # don't try to walk lanes — use next() to stay on road

            bp = blib.filter(random.choice(models))
            if not bp: bp = blib.filter("vehicle.tesla.model3")
            if not bp: continue
            bp = bp[0]
            bp.set_attribute("role_name", "congestion")

            tf = carla.Transform(
                carla.Location(x=cur_wp.transform.location.x,
                               y=cur_wp.transform.location.y,
                               z=cur_wp.transform.location.z + 0.3),
                cur_wp.transform.rotation)

            npc = world.try_spawn_actor(bp, tf)
            if not npc:
                # Try next position if blocked
                continue

            npc.set_autopilot(False)
            npc.set_simulate_physics(True)
            all_vehs.append(CongestionVehicle(world, npc, target_lane))
            spawned += 1

        print(f"[Congestion] Lane {target_lane}: {spawned}/{num_per_lane} spawned")

    print(f"[Congestion] Total: {len(all_vehs)} vehicles across lanes -1/-2/-3")
    return all_vehs


# ── Construction zone obstacle ────────────────────────────────────────────────
def spawn_obstacle(world):
    """3 cones + 2 barriers + 1 warning sign on lane -3, 30m ahead of merge."""
    town_map = world.get_map()
    wp = town_map.get_waypoint(OBSTACLE_LOCATION, project_to_road=True,
                                lane_type=carla.LaneType.Driving)
    if not wp: print("[Obstacle] No waypoint"); return []

    # Walk to lane -3
    for _ in range(4):
        if wp.lane_id == MERGE_LANE: break
        l = wp.get_left_lane()
        if l and l.lane_type == carla.LaneType.Driving: wp = l
        else: break

    blib    = world.get_blueprint_library()
    cone_bp = blib.find("static.prop.trafficcone01")
    bar_bp  = blib.find("static.prop.streetbarrier")
    wrn_bp  = blib.find("static.prop.trafficwarning")
    loc     = wp.transform.location
    rot     = wp.transform.rotation
    right   = wp.transform.get_right_vector()
    fwd     = wp.transform.get_forward_vector()
    spawned = []

    # Row of 3 cones across the lane
    for off in [-1.2, 0.0, 1.2]:
        p = carla.Location(x=loc.x+right.x*off, y=loc.y+right.y*off, z=loc.z+0.05)
        a = world.try_spawn_actor(cone_bp, carla.Transform(p, rot))
        if a: spawned.append(a)

    # 2 barriers 3m ahead
    for off in [-0.8, 0.8]:
        p = carla.Location(x=loc.x+fwd.x*3+right.x*off,
                           y=loc.y+fwd.y*3+right.y*off, z=loc.z+0.05)
        a = world.try_spawn_actor(bar_bp, carla.Transform(p, rot))
        if a: spawned.append(a)

    # Warning sign 2m behind
    p = carla.Location(x=loc.x-fwd.x*2, y=loc.y-fwd.y*2, z=loc.z+0.05)
    a = world.try_spawn_actor(wrn_bp, carla.Transform(p, rot))
    if a: spawned.append(a)

    print(f"[Obstacle] {len(spawned)} props at lane={wp.lane_id}  "
          f"x={loc.x:.1f}  y={loc.y:.1f}")
    return spawned


# ── Lane -3 blockers (TM-controlled, flow past merge point) ─────────────────
def spawn_lane3_blockers(world, tm, num=8):
    """
    Spawn vehicles in lane -3 using TM at highway speed (100 km/h),
    spread from 400m before merge to 200m before merge so they arrive
    continuously and block ego from merging for the required duration.
    These are fast-moving vehicles that make it hard to find a gap.
    """
    town_map  = world.get_map()
    all_wps   = town_map.generate_waypoints(5.0)
    blib      = world.get_blueprint_library()
    models    = ["vehicle.audi.tt", "vehicle.ford.mustang",
                 "vehicle.dodge.charger_2020", "vehicle.lincoln.mkz_2017",
                 "vehicle.chevrolet.impala", "vehicle.toyota.prius",
                 "vehicle.mini.cooper_s", "vehicle.seat.leon"]

    merge_ref = town_map.get_waypoint(MERGE_LOCATION, project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
    if not merge_ref: return []
    ref_s = merge_ref.s

    # Find lane -3 waypoints on road 36
    lane3_wps = sorted(
        [wp for wp in all_wps
         if wp.road_id == MERGE_ROAD and wp.lane_id == -3],
        key=lambda w: abs(w.s - ref_s))
    if not lane3_wps:
        print("[Blockers] No lane -3 wps on road 36"); return []

    anchor = lane3_wps[0]
    blockers = []
    spacing  = 400.0 / max(num, 1)   # spread over 400m before merge

    for i in range(num):
        behind_m = 200.0 + i * spacing   # 200m to 600m before merge
        prevs = anchor.previous(behind_m)
        if not prevs: continue
        sp_wp = prevs[0]
        if sp_wp.road_id != MERGE_ROAD or sp_wp.lane_id != -3:
            continue

        bp = blib.filter(models[i % len(models)])
        if not bp: continue
        bp = bp[0]; bp.set_attribute("role_name", "blocker")

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
        # Highway speed — tight headway
        tm.vehicle_percentage_speed_difference(npc, pct(100.0))
        blockers.append(npc)

    print(f"[Blockers] {len(blockers)} vehicles in lane -3 at highway speed")
    return blockers


# ── Merge detection ───────────────────────────────────────────────────────────
def ego_has_merged(world, ego):
    if not is_alive(ego): return False
    try:
        wp = world.get_map().get_waypoint(
            ego.get_transform().location, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        return wp is not None and wp.road_id == MERGE_ROAD
    except: return False

def ego_lane(world, ego):
    if not is_alive(ego): return None
    try:
        wp = world.get_map().get_waypoint(
            ego.get_transform().location, project_to_road=True,
            lane_type=carla.LaneType.Driving)
        return wp.lane_id if wp else None
    except: return None


# ── Horn vehicle ──────────────────────────────────────────────────────────────
class HornVehicle:
    def __init__(self):
        self.npc = None; self.spawned = False; self._last_horn = 0.0

    def spawn(self, world, ego, tm):
        town_map = world.get_map()
        ego_wp   = town_map.get_waypoint(ego.get_transform().location,
                                          project_to_road=True,
                                          lane_type=carla.LaneType.Driving)
        if not ego_wp: return
        prevs = ego_wp.previous(20.0)
        if not prevs: return
        sp = prevs[0]
        blib = world.get_blueprint_library()
        bp   = blib.filter("vehicle.dodge.charger_2020")
        if not bp: bp = blib.filter("vehicle.tesla.model3")
        if not bp: return
        bp = bp[0]; bp.set_attribute("role_name","horn")
        tf = carla.Transform(
            carla.Location(x=sp.transform.location.x,
                           y=sp.transform.location.y,
                           z=sp.transform.location.z+0.3),
            sp.transform.rotation)
        self.npc = world.try_spawn_actor(bp, tf)
        if not self.npc: print("[Horn] Spawn failed"); return
        self.npc.set_autopilot(True, TM_PORT)
        tm.auto_lane_change(self.npc, False)
        tm.ignore_lights_percentage(self.npc, 100)
        tm.vehicle_percentage_speed_difference(self.npc, pct(30.0))
        self.spawned = True
        print(f"[Horn] id={self.npc.id}  20m behind ego")

    def update(self, world, ego, t):
        if not self.spawned or not is_alive(self.npc): return
        if t - self._last_horn >= 5.0:
            self._last_horn = t
            try:
                loc = self.npc.get_transform().location
                world.debug.draw_point(loc+carla.Location(z=2.5),
                    size=0.5, color=carla.Color(255,0,0),
                    life_time=0.8, persistent_lines=False)
                world.debug.draw_string(loc+carla.Location(z=3.5),
                    "HOOONK!", draw_shadow=True,
                    color=carla.Color(255,50,50),
                    life_time=1.5, persistent_lines=False)
                print(f"  [Horn] HONK at T={t:.0f}s")
            except: pass

    def destroy(self): safe_destroy(self.npc)


# ── HUD ───────────────────────────────────────────────────────────────────────
class HUD:
    def __init__(self, world, ego): self.world=world; self.ego=ego

    def update(self, spd, warning, t, label="", ramp_mode=True):
        if not is_alive(self.ego): return
        try: loc = self.ego.get_transform().location
        except: return
        col = (COLOR_OK if (25<=spd<=40 if ramp_mode else SPEED_LOW<=spd<=SPEED_HIGH)
               else COLOR_WARN)
        lt = FDS*2
        self.world.debug.draw_string(loc+carla.Location(z=5.0),
            f"Speed: {spd:.1f} km/h   T+{t:.0f}s",
            draw_shadow=True, color=col, life_time=lt, persistent_lines=False)
        if label:
            self.world.debug.draw_string(loc+carla.Location(z=6.5), label,
                draw_shadow=True, color=COLOR_INFO,
                life_time=lt, persistent_lines=False)
        if warning:
            self.world.debug.draw_string(loc+carla.Location(z=3.8), warning,
                draw_shadow=True, color=COLOR_DANGER,
                life_time=lt, persistent_lines=False)


# ── Speed monitor ─────────────────────────────────────────────────────────────
class SpeedMonitor:
    def __init__(self, world, ego):
        self.world=world; self.ego=ego; self.hud=HUD(world,ego)
        self.scenario_time=0.0; self.slow_since=None
        self.warning=""; self.ramp_mode=True

    def ego_kmh(self):
        if not is_alive(self.ego): return 0.0
        try:
            v=self.ego.get_velocity()
            return 3.6*math.sqrt(v.x**2+v.y**2+v.z**2)
        except: return 0.0

    def _warn(self, spd):
        if self.ramp_mode: return ""
        if spd>SPEED_HIGH:
            self.slow_since=None
            return "Please drive at 100-105 km/h. Your current speed is too fast."
        if spd<WARN_LOW:
            if self.slow_since is None: self.slow_since=self.scenario_time
            elif self.scenario_time-self.slow_since>=WARN_DELAY:
                return ("Please drive at 100-105 km/h. "
                        "Your current speed is too low.")
        else: self.slow_since=None
        if SPEED_LOW<=spd<=SPEED_HIGH: return ""
        return self.warning

    def tick(self, dt, label=""):
        self.scenario_time+=dt
        spd=self.ego_kmh(); self.warning=self._warn(spd)
        self.hud.update(spd,self.warning,self.scenario_time,
                        label,ramp_mode=self.ramp_mode)
        return self.scenario_time, spd


def show_prompt(world, ego, msg, duration_s=12.0):
    if not is_alive(ego): return
    try:
        loc=ego.get_transform().location
        world.debug.draw_string(loc+carla.Location(z=8.0), msg,
            draw_shadow=True, color=COLOR_RATING,
            life_time=duration_s, persistent_lines=False)
        print(f"\n[PROMPT] {msg}\n")
    except: pass

def beep(world, loc):
    world.debug.draw_point(loc+carla.Location(z=2.5),
        size=0.8, color=carla.Color(255,255,255), life_time=0.5)
    print("[BEEP] *** Scenario 2 t=0 ***")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    client, world = connect(CARLA_HOST, CARLA_PORT)
    all_npcs = []

    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(False)
    tm.set_random_device_seed(42)
    tm.set_global_distance_to_leading_vehicle(5.0)

    try:
        ego = find_ego_vehicle(world)
        precise_respawn(ego, EGO_SPAWN)
        world.tick()
        beep(world, ego.get_transform().location)

        # Construction zone on lane -3, 30m ahead of merge
        cones = spawn_obstacle(world)
        all_npcs.extend(cones)
        world.tick()

        # Congestion on lanes -1, -2, -3 (NPCController, physics ON)
        # 14 vehicles per lane, 6m spacing = tight bumper-to-bumper congestion
        cong_vehicles = spawn_congestion_lanes(world, num_per_lane=14)
        all_npcs.extend([cv.npc for cv in cong_vehicles])
        world.tick()

        # Lane -3 TM blockers — fast vehicles flowing past merge at 100 km/h
        # These are separate from the NPCController congestion above
        lane3_blockers = spawn_lane3_blockers(world, tm, num=8)
        all_npcs.extend(lane3_blockers)
        world.tick()

        horn_vehicle = HornVehicle()
        monitor      = SpeedMonitor(world, ego)
        status_every = int(2.0 / FDS)
        tick_count   = 0
        merged       = False
        gap_opened   = False
        horn_spawned = False
        prompts_shown= set()

        prompt_schedule = {
            65.0:  "Turn on the left signal and merge into the main lane.",
            50.0:  "How much anger or frustration do you feel\ndue to the current traffic condition?",
            185.0: "How much anger or frustration did you feel\ndue to the merging refusal situation?",
            220.0: "Maintain a safe distance and follow the traffic speed.",
        }

        print("\n[Step 3] Running. Ctrl+C to stop.\n")

        while True:
            world.tick()
            if not is_alive(ego): print("Ego lost."); break

            t, ego_spd = monitor.tick(FDS)

            # Detect merge
            if ego_has_merged(world, ego) and not merged:
                merged = True
                monitor.ramp_mode = False
                print(f"[Merge] Ego merged at T={t:.1f}s  lane={ego_lane(world,ego)}")

            # ── Phase logic ───────────────────────────────────────────
            if t < 100.0:
                label = "Ramp — prepare to merge into lane 3"
                # Keep all congestion crawling
                for cv in cong_vehicles:
                    cv.update(random.uniform(SPEED_CONG_MIN, SPEED_CONG_MAX))

            elif 100.0 <= t < 160.0:
                label = "Merge zone — all lanes congested, waiting for gap"
                # Horn at 120s
                if t >= 120.0 and not horn_spawned:
                    horn_vehicle.spawn(world, ego, tm)
                    horn_spawned = True
                for cv in cong_vehicles:
                    cv.update(random.uniform(SPEED_CONG_MIN, SPEED_CONG_MAX))

            elif 160.0 <= t < 180.0:
                label = "Gap opening — merge now into lane 3!"
                if not gap_opened:
                    # Slow lane -3 NPCController vehicles near merge
                    lane3 = [cv for cv in cong_vehicles
                             if cv.ctrl.target_lane == -3 and is_alive(cv.npc)]
                    lane3.sort(key=lambda cv: math.sqrt(
                        (cv.npc.get_transform().location.x - MERGE_LOCATION.x)**2 +
                        (cv.npc.get_transform().location.y - MERGE_LOCATION.y)**2))
                    for cv in lane3[:3]:
                        cv.update(3.0)
                    for cv in [c for c in cong_vehicles if c not in lane3[:3]]:
                        cv.update(random.uniform(SPEED_CONG_MIN, SPEED_CONG_MAX))
                    # Also slow TM blockers to create gap in flowing lane -3
                    for b in lane3_blockers:
                        if is_alive(b):
                            tm.vehicle_percentage_speed_difference(b, pct(15.0))
                    gap_opened = True
                    print(f"[Gap] Opened in lane -3 at T={t:.1f}s")
                else:
                    for cv in cong_vehicles:
                        cv.update(random.uniform(SPEED_CONG_MIN, SPEED_CONG_MAX))

            elif 180.0 <= t < 220.0:
                label = "Stabilization — maintain 100-105 km/h"
                for cv in cong_vehicles:
                    cv.update(random.uniform(SPEED_CONG_MIN, SPEED_CONG_MAX))
                # Restore lane -3 TM blockers to normal speed after gap
                for b in lane3_blockers:
                    if is_alive(b):
                        tm.vehicle_percentage_speed_difference(b, pct(100.0))

            else:
                label = "Proceed to congestion phase"
                for cv in cong_vehicles:
                    cv.update(random.uniform(SPEED_CONG_MIN, SPEED_CONG_MAX))

            if horn_spawned:
                horn_vehicle.update(world, ego, t)

            for pt, msg in prompt_schedule.items():
                if t >= pt and pt not in prompts_shown:
                    show_prompt(world, ego, msg)
                    prompts_shown.add(pt)

            tick_count += 1
            if tick_count % status_every == 0:
                status = f"merged lane={ego_lane(world,ego)}" if merged else "ramp"
                print(f"  T={t:>6.1f}s  ego={ego_spd:>5.1f} km/h  "
                      f"{status}  [{label[:40]}]")

            if t >= 220.0 and merged:
                print("\n[Step 3] Complete — proceed to Step 4.\n"); break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[Step 3] Stopped.")
    finally:
        horn_vehicle.destroy()
        for npc in all_npcs: safe_destroy(npc)
        try:
            s=world.get_settings(); s.synchronous_mode=False
            s.fixed_delta_seconds=None; world.apply_settings(s)
        except: pass
        print("[Step 3] Done.")

if __name__ == "__main__":
    main()