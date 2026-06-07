import time
import argparse
import glob
import sys
import os
from numpy import random
from DReyeVR_utils import find_ego_vehicle
import math

try:
    sys.path.append(
        glob.glob(
            "C:/Carla/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass


import carla
sys.path.append('../')
sys.path.insert(0,r'C:\Carla\carla\PythonAPI\carla')

import io

# Read the .osm data
f = io.open("map.osm", mode="r", encoding="utf-8")
osm_data = f.read()
f.close()

# Define the desired settings. In this case, default values.
settings = carla.Osm2OdrSettings()
# Set OSM road types to export to OpenDRIVE
settings.set_osm_way_types(["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential"])
# Convert to .xodr
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# save opendrive file
f = open("output", 'w')
f.write(xodr_data)
f.close()