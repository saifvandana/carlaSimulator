"""
Pedestrian Manager — direct velocity control (no AI pathfinder)
================================================================
Spawns multiple pedestrians and walks them in a straight line
using set_target_velocity() — much more reliable than WalkerAIController
which gets confused by road geometry.

Pedestrians:
  P1 t=65s  (GREEN 1) Legal crossing beside Q1 at x=-66
             walks south from y=-80 to y=-89
  P2 t=185s (GREEN 2) Illegal jaywalker beside Q2 spot at x=-74
             walks south slowly, no warning
  P3 t=305s (GREEN 3) Sudden illegal crosser in ego path at x=-64
             walks south fast, coincides with illegal turn vehicle

Usage: import and use PedestrianManager in phase scripts.
"""

import carla
import time
import math
import random
import threading


# Walk direction: south = positive Y in this map
# Road at y=-84.4, north pavement y=-80, south pavement y=-89

PEDESTRIANS = [
    {
        "name"      : "P1",
        "trigger_t" : 50.0,        # red 1
        "spawn"     : carla.Location(x=-62.0, y=-78.0, z=1.0),
        "direction" : carla.Vector3D(x=0.0, y=-1.0, z=0.0),  # south
        "speed"     : 1.2,          # m/s normal walk
        "stop_y"    : -99.0,        # stop when y reaches this
        "label"     : "P1 legal",
    },
    {
        "name"      : "P2",
        "trigger_t" : 185.0,       # GREEN 2
        "spawn"     : carla.Location(x=-63.0, y=-79.0, z=1.0),
        "direction" : carla.Vector3D(x=1.0, y=0.0, z=0.0),
        "speed"     : 1.6,          # m/s slow jaywalk
        "stop_y"    : -99.0,
        "label"     : "P2 jaywalk",
    },
    {
        "name"      : "P3",
        "trigger_t" : 306.0,       # GREEN 3
        "spawn"     : carla.Location(x=-64.0, y=-80.0, z=1.0),
        "direction" : carla.Vector3D(x=0.0, y=-1.0, z=0.0),
        "speed"     : 1.6,          # m/s fast/sudden
        "stop_y"    : -99.0,
        "label"     : "P3 sudden",
    },
    {
        "name"      : "P4",
        "trigger_t" : 317.0,       # GREEN 3
        "spawn"     : carla.Location(x=-39.0, y=-80.0, z=0.97),
        "direction" : carla.Vector3D(x=0.0, y=-1.0, z=0.0),
        "speed"     : 1.3,          # m/s fast/sudden
        "stop_y"    : -99.0,
        "label"     : "P3 sudden",
    },
    {
        "name"      : "P5",
        "trigger_t" : 209.0,       # GREEN 3
        "spawn"     : carla.Location(x=-39.5, y=-80.0, z=0.97),
        "direction" : carla.Vector3D(x=0.0, y=-1.0, z=0.0),
        "speed"     : 1.6,          # m/s fast/sudden
        "stop_y"    : -99.0,
        "label"     : "P3 sudden",
    },
    {
        "name"      : "P6",
        "trigger_t" : 109.0,       # GREEN 3
        "spawn"     : carla.Location(x=-64.0, y=-99.0, z=0.97),
        "direction" : carla.Vector3D(x=1.0, y=0.0, z=0.0),
        "speed"     : 1.2,          # m/s fast/sudden
        "stop_y"    : -99.0,
        "label"     : "P3 sudden",
    },
    {
        "name"      : "P7",
        "trigger_t" : 209.0,       # GREEN 3
        "spawn"     : carla.Location(x=-64.5, y=-98.0, z=0.97),
        "direction" : carla.Vector3D(x=0.0, y=0.0, z=0.0),
        "speed"     : 1.6,          # m/s fast/sudden
        "stop_y"    : -99.0,
        "label"     : "P3 sudden",
    },
]


class SinglePedestrian:
    """
    One pedestrian controlled via direct velocity.
    Spawned at start, held still, activated at trigger_t.
    """

    STATE_WAITING  = "waiting"
    STATE_WALKING  = "walking"
    STATE_DONE     = "done"

    def __init__(self, world, client, cfg):
        self.world  = world
        self.client = client
        self.cfg    = cfg
        self.walker = None
        self.state  = self.STATE_WAITING
        self._spawned = False

    def spawn(self):
        bpl       = self.world.get_blueprint_library()
        walker_bp = random.choice(list(bpl.filter("walker.pedestrian.*")))

        # Spawn facing south (yaw=90 in this map)
        spawn_tf = carla.Transform(
            carla.Location(
                x=self.cfg["spawn"].x,
                y=self.cfg["spawn"].y,
                z=1.0
            ),
            carla.Rotation(yaw=0.0)
        )

        batch   = [carla.command.SpawnActor(walker_bp, spawn_tf)]
        results = self.client.apply_batch_sync(batch, True)

        if not results or results[0].error:
            print(f"[Ped] {self.cfg['name']} spawn FAILED: "
                  f"{results[0].error if results else 'no result'}")
            return False

        self.walker = self.world.get_actor(results[0].actor_id)
        if self.walker is None:
            print(f"[Ped] {self.cfg['name']} could not retrieve actor.")
            return False

        # Hold still — zero velocity
        self.walker.apply_control(carla.WalkerControl(
            direction=carla.Vector3D(0, 0, 0),
            speed=0.0
        ))

        loc = self.walker.get_location()
        print(f"[Ped] {self.cfg['name']} spawned at "
              f"({loc.x:.1f}, {loc.y:.1f}) — trigger at t={self.cfg['trigger_t']:.0f}s")
        self._spawned = True
        return True

    def update(self, t):
        if not self._spawned or self.walker is None:
            return
        if not self.walker.is_alive:
            self.state = self.STATE_DONE
            return

        loc = self.walker.get_location()

        if self.state == self.STATE_WAITING:
            if t >= self.cfg["trigger_t"]:
                self.state = self.STATE_WALKING
                print(f"[Ped] {self.cfg['name']} walking at t={t:.1f}s  "
                      f"speed={self.cfg['speed']} m/s")

            else:
                # Hold in place
                self.walker.apply_control(carla.WalkerControl(
                    direction=carla.Vector3D(0, 0, 0),
                    speed=0.0
                ))

        elif self.state == self.STATE_WALKING:
            # Check if reached south pavement
            # if loc.y <= self.cfg["stop_y"]:
            #     self.state = self.STATE_DONE
            #     self.walker.apply_control(carla.WalkerControl(
            #         direction=carla.Vector3D(0, 0, 0),
            #         speed=0.0
            #     ))
            #     print(f"[Ped] {self.cfg['name']} reached south pavement — done")
            # else:
                # Walk south using direct velocity control
            self.walker.apply_control(carla.WalkerControl(
                direction=self.cfg["direction"],
                speed=self.cfg["speed"]
            ))

    def destroy(self):
        if self.walker and self.walker.is_alive:
            self.walker.destroy()


class PedestrianManager:
    """Manages all pedestrians for Scenario 3."""

    def __init__(self, world, client):
        self.world  = world
        self.client = client
        self.peds   = [
            SinglePedestrian(world, client, cfg)
            for cfg in PEDESTRIANS
        ]

    def spawn_all(self):
        spawned = 0
        for ped in self.peds:
            if ped.spawn():
                spawned += 1
        print(f"[PedManager] {spawned}/{len(self.peds)} pedestrians spawned.")

    def update(self, t):
        for ped in self.peds:
            ped.update(t)

    def destroy_all(self):
        for ped in self.peds:
            ped.destroy()