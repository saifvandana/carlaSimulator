import carla
import sys
sys.path.append('../')
sys.path.insert(0,r'C:\DReyeVR\carla\PythonAPI\carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(2000.0)
world = client.load_world('Town03')

amap = world.get_map()
sampling_resolution = 2
# dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
grp = GlobalRoutePlanner(amap, sampling_resolution)
# grp.setup()
spawn_points = world.get_map().get_spawn_points()
print(spawn_points[50])
a = carla.Location(spawn_points[159].location)
b = carla.Location(spawn_points[11].location)
w1 = grp.trace_route(a, b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
i = 0
# for w in w1:
#     if i % 10 == 0:
#         world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
#         color=carla.Color(r=255, g=0, b=0), life_time=120.0,
#         persistent_lines=True)
#     else:
#         world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
#         color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
#         persistent_lines=True)
#     i += 1

for enumIndex,w in enumerate(spawn_points):
    if i % 10 == 0:
        world.debug.draw_string(carla.Location(w.location), str(enumIndex), draw_shadow=False,
        color=carla.Color(r=255, g=0, b=0), life_time=1000.0,
        persistent_lines=True)
    else:
        world.debug.draw_string(carla.Location(w.location), str(enumIndex), draw_shadow=False,
        color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
        persistent_lines=True)
    i += 1