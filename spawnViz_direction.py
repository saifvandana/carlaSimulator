import carla
import sys
import math
sys.path.append('../')
sys.path.insert(0,r'C:\DReyeVR\carla\PythonAPI\carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

waypoints = [218,228,261,57]
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(2000.0)
world = client.load_world('Town03')
amap = world.get_map() 
sampling_resolution = 2
# dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
grp = GlobalRoutePlanner(amap, sampling_resolution)
# grp.setup()
spawn_points = world.get_map().get_spawn_points()
ways = []
for i in range(0,len(waypoints)-1):
    a = carla.Location(spawn_points[waypoints[i]].location)
    b = carla.Location(spawn_points[waypoints[i+1]].location)
    w1 = grp.trace_route(a,b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
    ways+=w1
i = 0
for w in ways:
    t = w[0].transform
    begin = t.location + carla.Location(z=0.1)
    angle = math.radians(t.rotation.yaw)
    end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
    world.debug.draw_arrow(begin, end, thickness=0.1, arrow_size=0.1, life_time=0)
    if i==0:
        world.debug.draw_string(carla.Location(w[0].transform.location), 'Move to left lane', draw_shadow=False,
                color=carla.Color(r=255, g=0, b=0), life_time=1000.0,
                persistent_lines=True)
    i+=1
# for enumIndex,w in enumerate(spawn_points):
#     if i % 10 == 0:
#         world.debug.draw_string(carla.Location(w.location), str(enumIndex), draw_shadow=False,
#         color=carla.Color(r=255, g=0, b=0), life_time=1000.0,
#         persistent_lines=True)
#     else:
#         world.debug.draw_string(carla.Location(w.location), str(enumIndex), draw_shadow=False,
#         color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
#         persistent_lines=True)
#     i += 1