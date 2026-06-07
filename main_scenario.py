import math

import carla
import time
import random

c = carla.Client("127.0.0.1", 2000)
c.set_timeout(5.0)
world = c.load_world("Town05")
world_map = world.get_map()


# Test: freeze just one approach light and check if others in group change
tl_ids = [5131, 5128, 5127]  # IDs may have changed — rescan first
all_tls = world.get_actors().filter("traffic.traffic_light*")

# Find by position
approach = []
positions = [(-140.4, -78.1), (-113.1, -78.9), (-61.1, -78.1)]
for (tx, ty) in positions:
    for tl in all_tls:
        loc = tl.get_location()
        if abs(loc.x - tx) < 2 and abs(loc.y - ty) < 2:
            approach.append(tl)
            print(f"Found: id={tl.id} at ({loc.x:.1f}, {loc.y:.1f}) "
                  f"group={tl.get_group_traffic_lights()}")

print(f"\nFreezing {len(approach)} lights and setting GREEN...")
for tl in approach:
    # Freeze the whole group this light belongs to
    for group_tl in tl.get_group_traffic_lights():
        print(f"  Group member: id={group_tl.id} "
              f"({group_tl.get_location().x:.1f}, {group_tl.get_location().y:.1f})")
    tl.freeze(True)
    tl.set_state(carla.TrafficLightState.Green)

print("\nWaiting 10s — check if lights stay green in CARLA...")
time.sleep(10)

for tl in approach:
    tl.freeze(False)
print("Unfrozen.")