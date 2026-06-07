"""
Scenario 1 - Step 4: A1–A4 Cut-in Events
==========================================
Timeline (from document):
  0–60 s   : Adaptation
  60–170 s  : A1 — Indicator ON,  rapid decel   (100→70 km/h in 3 s)
  170–280 s : A2 — Indicator ON,  gradual decel (100→70 km/h in 6 s)
  280–390 s : A3 — Indicator OFF, rapid decel   (100→70 km/h in 3 s)
  390–500 s : A4 — Indicator OFF, gradual decel (100→70 km/h in 6 s)

Each A1–A4 block (110 s):
  0–20 s   : Free driving at 100 km/h (NPC follows ego in lane 3)
  20–45 s  : Interference — NPC cuts into lane 2, decelerates (25 s fixed)
  45–50 s  : Tail effect — ego must brake / slow behind NPC
  50–70 s  : NPC re-accelerates to 100 km/h
  70–110 s : Stable driving, NPC drives beside or disappears

Lane mapping:
  Lane 2 (ego)  = CARLA lane -2
  Lane 3 (NPC)  = CARLA lane -3
"""

from typing import Optional

import carla
import time
import math
import collections

from pygame import camera

# ── Configuration ─────────────────────────────────────────────────────────────
CARLA_HOST          = "localhost"
CARLA_PORT          = 2000
FIXED_DELTA_SECONDS = 0.05   # 20 Hz

EGO_SPAWN_INDEX     = 28     # road=36, lane=-2
HIGHWAY_ROAD_ID     = 36

# NPC spawns slightly ahead of ego in lane 3
NPC_SPAWN_INDEX     = 27     # road=36, lane=-3  (one index ahead)

# Speed settings
SPEED_NPC_CRUISE    = 100.0  # km/h — NPC cruise speed
SPEED_NPC_SLOW      = 70.0   # km/h — NPC speed after cut-in decel
SPEED_TARGET_LOW    = 100.0
SPEED_TARGET_HIGH   = 105.0
SPEED_WARN_LOW      = 90.0
SLOW_WARN_DELAY_S   = 20.0

# HUD colours
COLOR_OK      = carla.Color(r=0,   g=220, b=0)
COLOR_WARN    = carla.Color(r=255, g=180, b=0)
COLOR_DANGER  = carla.Color(r=255, g=40,  b=40)
COLOR_INFO    = carla.Color(r=100, g=200, b=255)
# ──────────────────────────────────────────────────────────────────────────────


# ── Shared utilities ──────────────────────────────────────────────────────────
def is_alive(actor) -> bool:
    try:
        return actor is not None and actor.is_alive
    except Exception:
        return False


def kmh_of(actor) -> float:
    if not is_alive(actor):
        return 0.0
    try:
        v = actor.get_velocity()
        return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
    except RuntimeError:
        return 0.0


def ms(kmh: float) -> float:
    return kmh / 3.6


def safe_destroy(actor) -> None:
    try:
        if is_alive(actor):
            actor.destroy()
    except Exception:
        pass


def connect_and_load(host, port, map_name="Town04"):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world  = client.load_world(map_name)
    # if map_name not in world.get_map().name:
    #     client.load_world(map_name)
    #     time.sleep(3)
    #     world = client.get_world()
    # settings = world.get_settings()
    # settings.synchronous_mode    = True
    # settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    # world.apply_settings(settings)
    world.set_weather(carla.WeatherParameters(
        cloudiness=10, precipitation=0,
        sun_altitude_angle=45, sun_azimuth_angle=200,
        fog_density=0, wetness=0,
    ))
    return client, world


# ── Ego spawn (DReyeVR) ───────────────────────────────────────────────────────
def spawn_ego(world: carla.World) -> carla.Vehicle:
    sp   = world.get_map().get_spawn_points()[EGO_SPAWN_INDEX]
    blib = world.get_blueprint_library()
    # Print ALL dreyevr blueprints so we can identify the vehicle one
    all_dreyevr = [(b.id, b.tags) for b in blib.filter("*dreyevr*")]
    print(f"[Ego] Available DReyeVR blueprints:")
    for bid, tags in all_dreyevr:
        print(f"      {bid}  tags={list(tags)}")

    # Pick the blueprint whose id contains 'vehicle' (not 'sensor')
    bp = None
    for b in blib.filter("*dreyevr*"):
        if "vehicle" in b.id.lower() and "sensor" not in b.id.lower():
            bp = b
            print(f"[Ego] Selected vehicle blueprint: {b.id}")
            break

    # Fallback: try exact known names
    if bp is None:
        for name in ["harplab.dreyevr_vehicle",
                     "vehicle.dreyevr.dreyevrvehicle",
                     "vehicle.lincoln.mkz_2017"]:   # last resort placeholder
            matches = blib.filter(name)
            if matches:
                bp = matches[0]
                print(f"[Ego] Fallback blueprint: {bp.id}")
                break

    if bp is None:
        raise RuntimeError(
            f"No DReyeVR vehicle blueprint found.\n"
            f"All dreyevr blueprints: {[b.id for b in blib.filter('*dreyevr*')]}\n"
            f"Edit spawn_ego() with the correct vehicle blueprint id."
        )

    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "hero")
    ego = world.try_spawn_actor(bp, sp)
    if ego is None:
        raise RuntimeError("Ego spawn failed.")
    print(f"[Ego] Spawned  id={ego.id}  bp={bp.id}")
    return ego

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


# ── NPC spawn ─────────────────────────────────────────────────────────────────
def spawn_npc(world: carla.World, spawn_index: int,
              color: str = "255,50,50") -> carla.Vehicle:
    """Spawn a red NPC vehicle at the given spawn point index."""
    sp   = world.get_map().get_spawn_points()[spawn_index]
    blib = world.get_blueprint_library()
    bp   = blib.find("vehicle.tesla.model3")
    bp.set_attribute("role_name", "npc")
    if bp.has_attribute("color"):
        bp.set_attribute("color", color)
    npc = world.try_spawn_actor(bp, sp)
    if npc is None:
        # Try nearby spawn points if preferred index is blocked
        for alt_idx in [26, 61, 62, 63]:
            sp  = world.get_map().get_spawn_points()[alt_idx]
            npc = world.try_spawn_actor(bp, sp)
            if npc:
                print(f"[NPC] Spawned at alt index {alt_idx}")
                break
    if npc is None:
        raise RuntimeError("NPC spawn failed — all fallback points blocked.")
 
    # Disable autopilot — we control NPC entirely via set_target_velocity
    npc.set_autopilot(False)
 
    # Disable physics-based collision response so NPC doesn't bounce/crash
    # when we teleport it during lane change
    npc.set_simulate_physics(True)   # keep physics ON for realism
    print(f"[NPC] Spawned  id={npc.id}  (manual velocity control, no autopilot)")
    return npc
 
 
# ── NPC speed control (manual velocity override) ──────────────────────────────
def set_npc_velocity(npc: carla.Vehicle, target_kmh: float,
                     world: carla.World = None) -> None:
    """
    Set NPC velocity along the ROAD direction (from nearest waypoint),
    not the vehicle body direction — prevents skidding after teleport.
    """
    if not is_alive(npc):
        return
    try:
        spd = ms(target_kmh)
        # Use waypoint forward vector for road-aligned velocity
        if world is not None:
            wp = world.get_map().get_waypoint(
                npc.get_transform().location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            if wp is not None:
                fwd = wp.transform.get_forward_vector()
                npc.set_target_velocity(carla.Vector3D(
                    x=fwd.x * spd, y=fwd.y * spd, z=0.0))
                return
        # Fallback: use vehicle forward vector
        fwd = npc.get_transform().get_forward_vector()
        npc.set_target_velocity(carla.Vector3D(
            x=fwd.x * spd, y=fwd.y * spd, z=0.0))
    except RuntimeError:
        pass
 
 
def set_npc_velocity_direct(npc: carla.Vehicle, target_kmh: float, world: carla.World) -> None:
    """Alias kept for compatibility."""
    set_npc_velocity(npc, target_kmh, world)


# ── Lane change helper ────────────────────────────────────────────────────────
def get_adjacent_lane_transform(world: carla.World, actor: carla.Vehicle,
                                target_lane_id: int) -> carla.Transform:
    """
    Return a transform in the adjacent lane directly beside the actor.
    target_lane_id: CARLA lane id to move to (e.g. -2 to move left).
    """
    town_map = world.get_map()
    loc      = actor.get_transform().location
    wp       = town_map.get_waypoint(loc, project_to_road=True,
                                     lane_type=carla.LaneType.Driving)
    if wp is None:
        return None

    # Walk left/right to find target lane
    candidate = wp
    for _ in range(4):
        if candidate.lane_id == target_lane_id:
            tf = candidate.transform
            tf.rotation = actor.get_transform().rotation
            return tf
        left  = candidate.get_left_lane()
        right = candidate.get_right_lane()
        if left and abs(left.lane_id) < abs(candidate.lane_id):
            candidate = left
        elif right:
            candidate = right
        else:
            break
    return None

# ── Lane change helper ────────────────────────────────────────────────────────
def get_waypoint_in_lane(world: carla.World, actor: carla.Vehicle,
                         target_lane_id: int):
    """
    Return the nearest waypoint in target_lane_id on the same road.
    Uses get_left_lane / get_right_lane to walk across lanes safely.
    Returns None if not found.
    """
    town_map = world.get_map()
    loc      = actor.get_transform().location
    wp       = town_map.get_waypoint(loc, project_to_road=True,
                                     lane_type=carla.LaneType.Driving)
    if wp is None:
        return None
 
    candidate = wp
    for _ in range(6):
        if candidate.lane_id == target_lane_id:
            return candidate
        # Try left then right
        left  = candidate.get_left_lane()
        right = candidate.get_right_lane()
        if left and left.lane_type == carla.LaneType.Driving:
            candidate = left
        elif right and right.lane_type == carla.LaneType.Driving:
            candidate = right
        else:
            break
    return None

def teleport_npc_to_lane(world: carla.World, npc: carla.Vehicle,
                          target_lane_id: int, speed_kmh: float) -> bool:
    """
    Move NPC to adjacent lane using the waypoint transform so it is
    correctly aligned with the road direction — prevents skidding.
    Velocity is re-applied along the waypoint forward vector.
    """
    wp = get_waypoint_in_lane(world, npc, target_lane_id)
    if wp is None:
        print(f"[NPC] Could not find waypoint for lane {target_lane_id}")
        return False
 
    # Use the WAYPOINT transform (road-aligned rotation), not the actor rotation
    tf = wp.transform
    # Lift slightly to avoid ground clipping on teleport
    tf.location.z += 0.3
 
    npc.set_transform(tf)
 
    # Re-apply velocity along the waypoint forward vector (road direction)
    fwd = tf.get_forward_vector()
    spd = ms(speed_kmh)
    npc.set_target_velocity(carla.Vector3D(
        x=fwd.x * spd,
        y=fwd.y * spd,
        z=0.0
    ))
    return True
 


# ── Indicator flash ───────────────────────────────────────────────────────────
def flash_indicator(world: carla.World, npc: carla.Vehicle,
                    side: str = "right", duration_s: float = 3.0) -> None:
    """
    Simulate an indicator by drawing a coloured arrow beside the NPC
    for `duration_s` seconds. CARLA does not expose blinker lights
    via the Python API for all vehicle types.
    """
    if not is_alive(npc):
        return
    tf  = npc.get_transform()
    fwd = tf.get_forward_vector()
    # Right offset for right indicator (lane 3 → lane 2 = moving left/right
    # depending on road orientation); adjust sign if needed
    offset = carla.Location(
        x = tf.location.x + fwd.y * (-3.5 if side == "right" else 3.5),
        y = tf.location.y - fwd.x * (-3.5 if side == "right" else 3.5),
        z = tf.location.z + 1.5,
    )
    world.debug.draw_arrow(
        tf.location + carla.Location(z=1.0),
        offset,
        thickness=0.15,
        arrow_size=0.3,
        color=carla.Color(r=255, g=165, b=0),   # orange
        life_time=duration_s,
        persistent_lines=False,
    )


# ── HUD & speed monitor ───────────────────────────────────────────────────────
class HUD:
    def __init__(self, world, ego):
        self.world = world
        self.ego   = ego

    def update(self, speed_kmh, warning, elapsed_s, event_label=""):
        if not is_alive(self.ego):
            return
        try:
            loc = self.ego.get_transform().location
        except RuntimeError:
            return

        col = (COLOR_OK if SPEED_TARGET_LOW <= speed_kmh <= SPEED_TARGET_HIGH
               else COLOR_DANGER if speed_kmh < SPEED_WARN_LOW or
               speed_kmh > SPEED_TARGET_HIGH else COLOR_WARN)
        lt = FIXED_DELTA_SECONDS * 2

        self.world.debug.draw_string(
            loc + carla.Location(z=5.0),
            f"Speed: {speed_kmh:.1f} km/h   T+{elapsed_s:.0f}s",
            draw_shadow=True, color=col, life_time=lt, persistent_lines=False)
        if event_label:
            self.world.debug.draw_string(
                loc + carla.Location(z=6.2),
                event_label,
                draw_shadow=True, color=COLOR_INFO,
                life_time=lt, persistent_lines=False)
        if warning:
            self.world.debug.draw_string(
                loc + carla.Location(z=3.8),
                warning,
                draw_shadow=True, color=COLOR_DANGER,
                life_time=lt, persistent_lines=False)


class SpeedMonitor:
    def __init__(self, world, ego):
        self.world         = world
        self.ego           = ego
        self.hud           = HUD(world, ego)
        self.scenario_time = 0.0
        self.slow_since    = None
        self.warning       = ""

    def _warn(self, spd):
        if spd > SPEED_TARGET_HIGH:
            self.slow_since = None
            return (f"! Too fast ({spd:.0f} km/h) — "
                    f"target {SPEED_TARGET_LOW:.0f}–{SPEED_TARGET_HIGH:.0f}")
        if spd < SPEED_WARN_LOW:
            if self.slow_since is None:
                self.slow_since = self.scenario_time
            elif self.scenario_time - self.slow_since >= SLOW_WARN_DELAY_S:
                return (f"! Too slow ({spd:.0f} km/h) — "
                        f"please reach {SPEED_TARGET_LOW:.0f}–"
                        f"{SPEED_TARGET_HIGH:.0f} km/h")
        else:
            self.slow_since = None
        if SPEED_TARGET_LOW <= spd <= SPEED_TARGET_HIGH:
            return ""
        return self.warning

    def tick(self, dt, event_label=""):
        self.scenario_time += dt
        spd          = kmh_of(self.ego)
        self.warning = self._warn(spd)
        self.hud.update(spd, self.warning, self.scenario_time, event_label)
        return self.scenario_time, spd


# ── Collision guard ───────────────────────────────────────────────────────────
class CollisionGuard:
    def __init__(self, world, ego):
        bp = world.get_blueprint_library().find("sensor.other.collision")
        self.sensor   = world.spawn_actor(bp, carla.Transform(), attach_to=ego)
        self.collided = False
        self.sensor.listen(lambda e: self._hit(e))

    def _hit(self, event):
        print(f"[Collision] {event.other_actor.type_id}")
        self.collided = True

    def destroy(self):
        try:
            if is_alive(self.sensor):
                self.sensor.stop()
                self.sensor.destroy()
        except Exception:
            pass


# ── Spectator ─────────────────────────────────────────────────────────────────
class SpectatorCamera:
    """
    Smooth chase camera — interpolates toward ego each update
    and only writes to the spectator every N ticks to avoid jitter.
    """
    def __init__(self, world, update_every=10, lerp_alpha=0.05):
        self.world        = world
        self.update_every = update_every  # ticks between moves
        self.alpha        = lerp_alpha    # 0=frozen 1=instant
        self.tick_count   = 0
        self.cur_x = self.cur_y = self.cur_z = self.cur_yaw = None

    def update(self, ego):
        self.tick_count += 1
        if self.tick_count % self.update_every != 0:
            return
        if not is_alive(ego):
            return
        try:
            tf  = ego.get_transform()
            fwd = tf.get_forward_vector()
            tx   = tf.location.x - fwd.x * 14
            ty   = tf.location.y - fwd.y * 14
            tz   = tf.location.z + 7
            tyaw = tf.rotation.yaw
            if self.cur_x is None:
                self.cur_x, self.cur_y = tx, ty
                self.cur_z, self.cur_yaw = tz, tyaw
            a = self.alpha
            self.cur_x   += a * (tx   - self.cur_x)
            self.cur_y   += a * (ty   - self.cur_y)
            self.cur_z   += a * (tz   - self.cur_z)
            self.cur_yaw += a * (tyaw - self.cur_yaw)
            self.world.get_spectator().set_transform(carla.Transform(
                carla.Location(x=self.cur_x, y=self.cur_y, z=self.cur_z),
                carla.Rotation(pitch=-12, yaw=self.cur_yaw, roll=0)))
        except RuntimeError:
            pass


def set_spectator(world, ego):
    """Legacy single-call — kept for compatibility."""
    if not is_alive(ego):
        return
    try:
        tf  = ego.get_transform()
        fwd = tf.get_forward_vector()
        world.get_spectator().set_transform(carla.Transform(
            carla.Location(x=tf.location.x - fwd.x*14,
                           y=tf.location.y - fwd.y*14,
                           z=tf.location.z + 7),
            carla.Rotation(pitch=-12, yaw=tf.rotation.yaw)))
    except RuntimeError:
        pass


# ── Beep ──────────────────────────────────────────────────────────────────────
def beep(world, location):
    world.debug.draw_point(location + carla.Location(z=2.5),
                           size=0.8, color=carla.Color(255, 255, 255),
                           life_time=0.5)
    print("[BEEP] *** t = 0  scenario start ***")


# ── NPC Traffic Manager setup ─────────────────────────────────────────────────
def setup_npc_tm(client, npc, tm_port=8001):
    """
    Use a SEPARATE traffic manager port for the NPC so it doesn't
    interfere with the ego's TM (if any). Returns the TM instance.
    """
    tm = client.get_trafficmanager(tm_port)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(0)
    npc.set_autopilot(True, tm_port)
    tm.auto_lane_change(npc, False)          # we control lane changes manually
    tm.ignore_lights_percentage(npc, 100)
    tm.ignore_signs_percentage(npc,  100)
    # Set NPC cruise speed to 100 km/h (120 limit → 17% reduction)
    tm.vehicle_percentage_speed_difference(npc, 17.0)
    return tm


# ── A-event executor ──────────────────────────────────────────────────────────
class AEvent:
    """
    Executes one A1–A4 cut-in block.
 
    Parameters
    ----------
    label        : "A1" / "A2" / "A3" / "A4"
    indicator    : True = blinker shown before cut-in
    decel_time_s : 3 s (rapid) or 6 s (gradual)
    start_t      : absolute scenario time when this block starts
    """
 
    BLOCK_DURATION  = 110.0   # total block length (s)
    FREE_DRIVE      = 20.0    # NPC follows ego in lane 3
    INTERFERE_START = 20.0    # cut-in begins
    INTERFERE_END   = 45.0    # interference ends
    TAIL_END        = 50.0    # tail effect ends
    REACCEL_END     = 70.0    # NPC back to 100 km/h
    # 70–110 s → stable
 
    def __init__(self, label: str, indicator: bool,
                 decel_time_s: float, start_t: float):
        self.label        = label
        self.indicator    = indicator
        self.decel_time_s = decel_time_s
        self.start_t      = start_t
        self.cut_in_done  = False
        self.phase        = "free"
        print(f"[{label}] Initialized  indicator={'ON' if indicator else 'OFF'}  "
              f"decel={decel_time_s}s  starts at T={start_t:.0f}s")
 
    def update(self, world: carla.World, npc: carla.Vehicle,
               scenario_t: float, dt: float) -> str:
        """
        Called every tick. Returns current phase label for HUD.
        scenario_t : absolute time since scenario start
        """
        if not is_alive(npc):
            return self.label + " (NPC gone)"
 
        rel_t = scenario_t - self.start_t   # time within this block
        spd   = kmh_of(npc)
 
        # ── Phase: free driving (0–20 s) ──────────────────────────────────
        if rel_t < self.FREE_DRIVE:
            self.phase = "free"
            set_npc_velocity(npc, SPEED_NPC_CRUISE, world)
            return f"{self.label} — NPC following in lane 3"
 
        # ── Phase: interference (20–45 s) ─────────────────────────────────
        elif rel_t < self.INTERFERE_END:
            self.phase = "interfere"
 
            # Show indicator then cut in (once)
            if not self.cut_in_done:
                if self.indicator:
                    flash_indicator(world, npc, side="left",
                                    duration_s=2.0)
                    print(f"[{self.label}] Indicator ON — cutting in ...")
                else:
                    print(f"[{self.label}] No indicator — cutting in ...")
 
                # Teleport NPC from lane 3 → lane 2
                success = teleport_npc_to_lane(world, npc,
                                               target_lane_id=-2,
                                               speed_kmh=SPEED_NPC_CRUISE)
                if success:
                    print(f"[{self.label}] NPC cut into lane 2")
                self.cut_in_done = True
 
            # Smoothly interpolate NPC speed from 100 → 70 km/h
            # over decel_time_s seconds
            decel_elapsed = rel_t - self.INTERFERE_START
            progress      = min(decel_elapsed / self.decel_time_s, 1.0)
            target        = SPEED_NPC_CRUISE + progress * (SPEED_NPC_SLOW - SPEED_NPC_CRUISE)
            set_npc_velocity(npc, target, world)
            return (f"{self.label} — CUT-IN  "
                    f"NPC {target:.0f} km/h  "
                    f"({'rapid' if self.decel_time_s == 3 else 'gradual'})")
 
        # ── Phase: tail effect (45–50 s) ──────────────────────────────────
        elif rel_t < self.TAIL_END:
            self.phase = "tail"
            set_npc_velocity(npc, SPEED_NPC_SLOW, world)
            return f"{self.label} — Tail effect (ego must slow)"
 
        # ── Phase: re-acceleration (50–70 s) ──────────────────────────────
        elif rel_t < self.REACCEL_END:
            self.phase = "reaccel"
            set_npc_velocity(npc, SPEED_NPC_CRUISE, world)
            return f"{self.label} — NPC re-accelerating to 100 km/h"
 
        # ── Phase: stable (70–110 s) ──────────────────────────────────────
        else:
            self.phase = "stable"
            set_npc_velocity(npc, SPEED_NPC_CRUISE, world)
            return f"{self.label} — Stable driving"
 
# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    client, world   = connect_and_load(CARLA_HOST, CARLA_PORT)
    ego             = None
    npc             = None
    collision_guard = None

    try:
        # ── Spawn actors ───────────────────────────────────────────────
        ego = find_ego_vehicle(world)
        success = precise_respawn(ego, location=(388.5, -154.7, 0.5), yaw=None)
        time.sleep(0.5)
        # world.tick()

        # collision_guard = CollisionGuard(world, ego)
        # world.tick()

        npc = spawn_npc(world, NPC_SPAWN_INDEX)
        # world.tick()

        # Give NPC initial speed so it doesn't just sit still
        set_npc_velocity_direct(npc, SPEED_NPC_CRUISE, world)

        # Set up NPC traffic manager
        tm_npc = setup_npc_tm(client, npc, tm_port=8001)
        # world.tick()

        # ── Start beep ─────────────────────────────────────────────────
        beep(world, ego.get_transform().location)
        # world.tick()

        # ── Build scenario timeline ────────────────────────────────────
        #   A1 starts at 60 s, A2 at 170 s, A3 at 280 s, A4 at 390 s
        events = [
            AEvent("A1", indicator=True,  decel_time_s=3.0, start_t=60.0),
            AEvent("A2", indicator=True,  decel_time_s=6.0, start_t=170.0),
            AEvent("A3", indicator=False, decel_time_s=3.0, start_t=280.0),
            AEvent("A4", indicator=False, decel_time_s=6.0, start_t=390.0),
        ]

        monitor      = SpeedMonitor(world, ego)
        # camera       = SpectatorCamera(world, update_every=40, lerp_alpha=0.01)
        status_every = int(2.0 / FIXED_DELTA_SECONDS)
        tick_count   = 0
        active_event = None
        world.tick()

        print("\n[Step 4] Scenario running — A1–A4 cut-in events active.")
        print("         Press Ctrl+C to stop.\n")

        # ── Main loop ──────────────────────────────────────────────────
        while True:
            # world.tick()

            # if not is_alive(ego):
            #     print("[Step 4] Ego lost.")
            #     break
            # if collision_guard.collided:
            #     print("[Step 4] Collision — stopping.")
            #     break

            t, ego_spd = monitor.tick(FIXED_DELTA_SECONDS)

            # Determine which A-event is active
            active_event = None
            for ev in events:
                rel = t - ev.start_t
                if 0 <= rel < AEvent.BLOCK_DURATION:
                    active_event = ev
                    break

            # Phase label for HUD
            if t < 60.0:
                label = "Adaptation — drive 100–105 km/h"
            elif t >= 500.0:
                label = "A1–A4 complete — Step 5 next"
            elif active_event:
                label = active_event.update(world, npc, t, FIXED_DELTA_SECONDS)
            else:
                label = "Between events"

            # Update HUD event label
            monitor.hud.update(ego_spd, monitor.warning, t, label)

            # camera.update(ego)

            tick_count += 1
            if tick_count % status_every == 0:
                print(f"  T={t:>6.1f}s  ego={ego_spd:>5.1f} km/h  "
                      f"npc={kmh_of(npc):>5.1f} km/h  [{label}]")

            # Stop automatically after A4 ends (t = 500 s)
            if t >= 500.0:
                print("\n[Step 4] A1–A4 complete at T=500 s.")
                print("         Proceed to Step 5 — Congestion Phase.\n")
                break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[Step 4] Stopped by user.")

    finally:
        if collision_guard:
            collision_guard.destroy()
        if is_alive(npc):
            npc.set_autopilot(False)
        safe_destroy(npc)
        safe_destroy(ego)
        try:
            s = world.get_settings()
            s.synchronous_mode    = False
            s.fixed_delta_seconds = None
            world.apply_settings(s)
        except Exception:
            pass
        print("[Step 4] Done.")


if __name__ == "__main__":
    main()