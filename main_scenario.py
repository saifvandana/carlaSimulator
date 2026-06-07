"""
Town04 Map Inspector
====================
Run this BEFORE Step 2 to identify:
  - Good highway spawn points for the ego vehicle (lane 2)
  - Waypoints on 3-lane highway sections
  - Lane IDs and road IDs on the highway ring

Output will tell us exactly which road_id / lane_id to use in Step 2.
"""

import carla

CARLA_HOST = "localhost"
CARLA_PORT = 2000


def main():
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(10.0)
    world    = client.load_world("Town04")
    town_map = world.get_map()

    print(f"\n[Inspector] Map: {town_map.name}")
    print("=" * 60)

    # ── 1. All spawn points ───────────────────────────────────────
    spawn_pts = town_map.get_spawn_points()
    print(f"\n[Spawn Points] Total: {len(spawn_pts)}")
    print(f"{'Index':<6} {'x':>8} {'y':>8} {'z':>6} {'yaw':>7}  road  lane")
    print("-" * 60)

    for i, sp in enumerate(spawn_pts):
        wp = town_map.get_waypoint(sp.location,
                                   project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        if wp:
            print(f"[{i:>3}]  {sp.location.x:>8.1f} {sp.location.y:>8.1f} "
                  f"{sp.location.z:>6.1f} {sp.rotation.yaw:>7.1f}°  "
                  f"road={wp.road_id:<4} lane={wp.lane_id}")
        else:
            print(f"[{i:>3}]  {sp.location.x:>8.1f} {sp.location.y:>8.1f} "
                  f"{sp.location.z:>6.1f} {sp.rotation.yaw:>7.1f}°  (no waypoint)")

    # ── 2. Find 3-lane road sections ─────────────────────────────
    print("\n[3-Lane Roads] Scanning waypoints every 10 m ...")
    print(f"{'road_id':<10} {'lane_id':<10} {'x':>8} {'y':>8} {'lane_width':>11}")
    print("-" * 60)

    seen_roads = {}   # road_id → set of lane_ids

    # Sample waypoints across the whole map
    all_wps = town_map.generate_waypoints(10.0)   # one every 10 m
    for wp in all_wps:
        rid = wp.road_id
        lid = wp.lane_id
        if rid not in seen_roads:
            seen_roads[rid] = set()
        seen_roads[rid].add(lid)

    # Print roads that have 3 or more driving lanes
    for rid, lids in sorted(seen_roads.items()):
        if len(lids) >= 3:
            # Sample a waypoint on this road to get coordinates
            sample = next((w for w in all_wps if w.road_id == rid), None)
            if sample:
                print(f"road={rid:<6}  lanes={sorted(lids)}  "
                      f"x={sample.transform.location.x:>8.1f}  "
                      f"y={sample.transform.location.y:>8.1f}")

    # ── 3. Detailed lane info for highway candidates ──────────────
    print("\n[Lane Detail] Checking lane types on 3-lane roads ...")
    print("-" * 60)
    for rid, lids in sorted(seen_roads.items()):
        if len(lids) >= 3:
            for wp in all_wps:
                if wp.road_id == rid:
                    print(f"  road={rid}  lane={wp.lane_id:>3}  "
                          f"type={wp.lane_type}  "
                          f"width={wp.lane_width:.1f}m  "
                          f"left_mark={wp.left_lane_marking.type}  "
                          f"right_mark={wp.right_lane_marking.type}")
                    break   # one sample per road is enough here

    print("\n[Inspector] Done. Use the road_id and lane_ids above in Step 2.\n")


if __name__ == "__main__":
    main()