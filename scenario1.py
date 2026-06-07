"""
Scenario 1 - Step 2: Road & Lane Configuration
===============================================
- Selects Road 36 as the highway stretch (8-lane road, long straight)
- Maps CARLA lane IDs to scenario lanes 1, 2, 3
- Spawns the ego vehicle in lane 2 (CARLA lane -2)
- Visualizes lane waypoints in the CARLA spectator
- Confirms lane markings (solid right boundary on lane 3)

Lane Mapping (negative direction, yaw ~90°, northbound):
  Scenario Lane 1  →  CARLA lane -1  (leftmost / fast lane)
  Scenario Lane 2  →  CARLA lane -2  (ego vehicle lane)
  Scenario Lane 3  →  CARLA lane -3  (NPC cut-in source lane)
  Scenario Lane 4  →  CARLA lane -4  (rightmost / shoulder side)

Ego spawn point index: 28  (road=36, lane=-2, x=388.5, y=-134.7)
"""

import carla
import time
import math

# ── Configuration ─────────────────────────────────────────────────────────────
CARLA_HOST          = "localhost"
CARLA_PORT          = 2000
FIXED_DELTA_SECONDS = 0.05

# Road selection (from inspector output)
HIGHWAY_ROAD_ID     = 36

# CARLA lane IDs → scenario lane numbers
LANE_MAP = {
    "lane1": -1,   # leftmost (fast lane)
    "lane2": -2,   # ego vehicle
    "lane3": -3,   # NPC cut-in source
    "lane4": -4,   # rightmost
}

# Ego spawn: spawn point index 28 (road=36, lane=-2)
EGO_SPAWN_INDEX     = 28

# Waypoint sampling interval along the road (metres)
WP_SAMPLE_INTERVAL  = 10.0
# ──────────────────────────────────────────────────────────────────────────────


# ── Helper: km/h → m/s ────────────────────────────────────────────────────────
def kmh_to_ms(kmh: float) -> float:
    return kmh / 3.6


# ── Step 1 helpers (inline so Step 2 is self-contained) ──────────────────────
def connect_and_load(host, port, map_name="Town04"):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.load_world(map_name)
    if map_name not in world.get_map().name:
        print(f"[Setup] Loading {map_name} ...")
        client.load_world(map_name)
        time.sleep(3)
        world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode    = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)
    world.set_weather(carla.WeatherParameters(
        cloudiness=10, precipitation=0, sun_altitude_angle=45,
        sun_azimuth_angle=200, fog_density=0, wetness=0,
    ))
    print(f"[Setup] Connected — map: {world.get_map().name}")
    return client, world


# ── Lane configuration ────────────────────────────────────────────────────────
def get_lane_waypoints(town_map: carla.Map, road_id: int,
                       lane_id: int, interval: float) -> list:
    """
    Return ordered waypoints along a specific road+lane,
    sampled every `interval` metres.
    """
    all_wps  = town_map.generate_waypoints(interval)
    lane_wps = [wp for wp in all_wps
                if wp.road_id == road_id and wp.lane_id == lane_id]
    # Sort by s (distance along road) for a clean ordered list
    lane_wps.sort(key=lambda w: w.s)
    return lane_wps


def print_lane_summary(town_map: carla.Map, road_id: int) -> None:
    """Print marking types for all lanes on the chosen road."""
    print(f"\n[Lane Config] Road {road_id} — lane marking summary:")
    print(f"  {'ScenarioLane':<15} {'CARLA lane':<12} "
          f"{'left_mark':<15} {'right_mark':<15} {'width'}")
    print("  " + "-" * 65)

    for scenario_name, carla_lid in LANE_MAP.items():
        wps = get_lane_waypoints(town_map, road_id, carla_lid, 50.0)
        if wps:
            wp = wps[len(wps) // 2]   # use middle waypoint for representative marks
            print(f"  {scenario_name:<15} lane {carla_lid:<9} "
                  f"{str(wp.left_lane_marking.type):<15} "
                  f"{str(wp.right_lane_marking.type):<15} "
                  f"{wp.lane_width:.1f} m")
        else:
            print(f"  {scenario_name:<15} lane {carla_lid:<9} (no waypoints found)")


def get_ego_spawn_transform(world: carla.World, spawn_index: int) -> carla.Transform:
    """Return the spawn transform for the ego vehicle."""
    spawn_pts = world.get_map().get_spawn_points()
    if spawn_index >= len(spawn_pts):
        raise IndexError(f"Spawn index {spawn_index} out of range "
                         f"(total: {len(spawn_pts)})")
    sp = spawn_pts[spawn_index]
    print(f"\n[Ego Spawn] Index {spawn_index}: "
          f"x={sp.location.x:.1f}  y={sp.location.y:.1f}  "
          f"z={sp.location.z:.1f}  yaw={sp.rotation.yaw:.1f}°")
    return sp


# ── Ego vehicle ───────────────────────────────────────────────────────────────
def spawn_ego(world: carla.World, spawn_tf: carla.Transform) -> carla.Vehicle:
    """Spawn the ego vehicle (Tesla Model 3) at the given transform."""
    blueprint_lib = world.get_blueprint_library()
    ego_bp        = blueprint_lib.find("vehicle.tesla.model3")
    ego_bp.set_attribute("role_name", "ego")
    ego_bp.set_attribute("color", "255,255,255")   # white for visibility

    ego = world.try_spawn_actor(ego_bp, spawn_tf)
    if ego is None:
        raise RuntimeError("[Ego] Failed to spawn ego vehicle — "
                           "spawn point may be blocked.")
    print(f"[Ego] Spawned: {ego.type_id}  id={ego.id}")
    return ego


# ── Spectator ─────────────────────────────────────────────────────────────────
def follow_spectator(world: carla.World, ego: carla.Vehicle) -> None:
    """Position spectator behind and above the ego for a chase-cam view."""
    ego_tf   = ego.get_transform()
    fwd      = ego_tf.get_forward_vector()
    spec_loc = carla.Location(
        x = ego_tf.location.x - fwd.x * 12,
        y = ego_tf.location.y - fwd.y * 12,
        z = ego_tf.location.z + 6,
    )
    world.get_spectator().set_transform(
        carla.Transform(spec_loc,
                        carla.Rotation(pitch=-15,
                                       yaw=ego_tf.rotation.yaw,
                                       roll=0))
    )


# ── Lane visualisation (debug draw) ──────────────────────────────────────────
def draw_lane_waypoints(world: carla.World, town_map: carla.Map,
                        road_id: int) -> None:
    """
    Draw coloured dots along each lane so you can see them in the
    CARLA spectator window.
      Lane 1 (fast)  → blue
      Lane 2 (ego)   → green
      Lane 3 (NPC)   → yellow
      Lane 4 (right) → red
    """
    colours = {
        "lane1": carla.Color(r=0,   g=0,   b=255),   # blue
        "lane2": carla.Color(r=0,   g=255, b=0),     # green  ← ego
        "lane3": carla.Color(r=255, g=255, b=0),     # yellow ← NPC cut-in
        "lane4": carla.Color(r=255, g=0,   b=0),     # red
    }
    for name, lid in LANE_MAP.items():
        wps = get_lane_waypoints(town_map, road_id, lid, WP_SAMPLE_INTERVAL)
        for wp in wps:
            world.debug.draw_point(
                wp.transform.location + carla.Location(z=0.3),
                size=0.12,
                color=colours[name],
                life_time=60.0,    # visible for 60 s
                persistent_lines=False,
            )
    print("[Viz] Lane waypoints drawn (blue=L1, green=L2/ego, "
          "yellow=L3/NPC, red=L4).")


# ── Waypoint store (used by later steps) ─────────────────────────────────────
class RoadConfig:
    """
    Holds the lane waypoints and ego spawn info for use in Steps 3+.
    """
    def __init__(self, world: carla.World, road_id: int):
        town_map   = world.get_map()
        self.road_id    = road_id
        self.lane_map   = LANE_MAP

        # Ordered waypoint lists per scenario lane
        self.wps = {
            name: get_lane_waypoints(town_map, road_id, lid, WP_SAMPLE_INTERVAL)
            for name, lid in LANE_MAP.items()
        }

        # Road start / end from lane 2 waypoints
        lane2_wps = self.wps["lane2"]
        self.road_start = lane2_wps[0].transform.location  if lane2_wps else None
        self.road_end   = lane2_wps[-1].transform.location if lane2_wps else None

        print(f"\n[RoadConfig] Road {road_id} waypoints loaded:")
        for name, wlist in self.wps.items():
            print(f"  {name}: {len(wlist)} waypoints")
        if self.road_start:
            print(f"  Road start: x={self.road_start.x:.1f}  "
                  f"y={self.road_start.y:.1f}")
            print(f"  Road end  : x={self.road_end.x:.1f}  "
                  f"y={self.road_end.y:.1f}")

    def get_spawn_transform_on_lane(self, lane_name: str,
                                    s_fraction: float = 0.1) -> carla.Transform:
        """
        Return a transform along the lane at `s_fraction` (0=start, 1=end).
        Useful for placing NPCs at known positions relative to the road.
        """
        wps   = self.wps[lane_name]
        idx   = int(len(wps) * s_fraction)
        idx   = max(0, min(idx, len(wps) - 1))
        return wps[idx].transform


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    client, world = connect_and_load(CARLA_HOST, CARLA_PORT)
    town_map      = world.get_map()
    ego           = None

    try:
        # 1. Print lane marking summary for Road 36
        print_lane_summary(town_map, HIGHWAY_ROAD_ID)

        # 2. Build road config (waypoint store)
        road_cfg = RoadConfig(world, HIGHWAY_ROAD_ID)

        # 3. Draw lane waypoints in spectator
        draw_lane_waypoints(world, town_map, HIGHWAY_ROAD_ID)

        # 4. Get ego spawn transform and spawn vehicle
        ego_tf = get_ego_spawn_transform(world, EGO_SPAWN_INDEX)
        ego    = spawn_ego(world, ego_tf)

        # 5. Position spectator behind ego
        world.tick()
        follow_spectator(world, ego)

        # 6. Confirm road extents
        print(f"\n[Step 2] ✓ Road & lane configuration complete.")
        print(f"         Road ID    : {HIGHWAY_ROAD_ID}")
        print(f"         Ego lane   : lane2 (CARLA lane -2)")
        print(f"         Cut-in lane: lane3 (CARLA lane -3)")
        print(f"         Waypoints  : {len(road_cfg.wps['lane2'])} points "
              f"along lane 2")
        print(f"\n  Ready for Step 3 — Ego Vehicle Speed Control & HUD.\n")

        # Keep the scene alive for inspection
        print("[Step 2] Scene running — press Ctrl+C to exit.")
        while True:
            world.tick()
            follow_spectator(world, ego)
            time.sleep(FIXED_DELTA_SECONDS)

    except KeyboardInterrupt:
        print("\n[Step 2] Interrupted.")

    finally:
        if ego:
            ego.destroy()
            print("[Step 2] Ego vehicle destroyed.")
        settings = world.get_settings()
        settings.synchronous_mode    = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("[Step 2] Simulation settings restored.")


if __name__ == "__main__":
    main()