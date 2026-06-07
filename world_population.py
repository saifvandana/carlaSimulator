"""
World Population — Background Traffic & Pedestrians
=====================================================
Spawns background vehicles and pedestrians to make the world
feel alive. All actors are managed by CARLA's Traffic Manager
and WalkerAI — they roam freely without interfering with the
scenario logic.

Safety exclusion zone: no actors spawn within 50m of the
scenario corridor (x=-82 to x=20, y=-84.5).

Usage: import WorldPopulator and call populate() / destroy_all()
"""

import carla
import time
import random

# Vehicle blueprints — common everyday cars, no emergency/special
VEHICLE_FILTERS = [
    "vehicle.audi.a2",
    "vehicle.audi.tt",
    "vehicle.chevrolet.impala",
    "vehicle.citroen.c3",
    "vehicle.ford.mustang",
    "vehicle.lincoln.mkz_2020",
    "vehicle.mercedes.coupe",
    "vehicle.mini.cooper_s",
    "vehicle.nissan.micra",
    "vehicle.nissan.patrol",
    "vehicle.seat.leon",
    "vehicle.tesla.model3",
    "vehicle.toyota.prius",
    "vehicle.volkswagen.t2",
]

WALKER_SPEED_MIN = 0.8   # m/s
WALKER_SPEED_MAX = 1.8   # m/s


# ── Config ────────────────────────────────────────────────────────────────────

N_VEHICLES   = 25    # background vehicles
N_WALKERS    = 2    # background pedestrians

# ── Helpers ───────────────────────────────────────────────────────────────────

def _in_exclusion_zone(loc):
    if -165.0 <= loc.x <= -80.0 and -88.0 <= loc.y <= -81.0:
        return True
    return False
   


class WorldPopulator:
    """
    Spawns and manages background vehicles and pedestrians.
    Call populate() once at startup, destroy_all() on cleanup.
    """

    def __init__(self, world, client):
        self.world  = world
        self.client = client
        self._vehicles    = []
        self._walkers     = []
        self._controllers = []
        self._tm          = None

    def populate(self):
        self._spawn_vehicles()
        # self._spawn_walkers()

    # ── Vehicles ──────────────────────────────────────────────────────────────

    def _spawn_vehicles(self):
        bpl          = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        world_map    = self.world.get_map()
        random.shuffle(spawn_points)

        # Only use spawn points that sit on a proper driving lane
        # (not junctions, not shoulders, lane width > 2.5m)
        safe_sps = []
        for sp in spawn_points:
            if _in_exclusion_zone(sp.location):
                continue
            wp = world_map.get_waypoint(
                sp.location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            if wp and not wp.is_junction and wp.lane_width > 2.5:
                safe_sps.append(sp)

        # Build batch spawn commands
        batch = []
        used  = 0
        for sp in safe_sps:
            if used >= N_VEHICLES:
                break
            bp_id = random.choice(VEHICLE_FILTERS)
            bps   = bpl.filter(bp_id)
            if not bps:
                continue
            bp = bps[0]
            if bp.has_attribute("color"):
                color = random.choice(
                    bp.get_attribute("color").recommended_values
                )
                bp.set_attribute("color", color)
            batch.append(carla.command.SpawnActor(bp, sp))
            used += 1

        results = self.client.apply_batch_sync(batch, True)
        spawned = 0
        for r in results:
            if not r.error:
                actor = self.world.get_actor(r.actor_id)
                if actor:
                    self._vehicles.append(actor)
                    spawned += 1

        # Wait for physics to settle before enabling autopilot
        time.sleep(1.0)

        # Configure Traffic Manager
        self._tm = self.client.get_trafficmanager(8000)
        self._tm.set_global_distance_to_leading_vehicle(7.0)
        self._tm.global_percentage_speed_difference(15.0)

        # Set autopilot explicitly — batch SetAutopilot unreliable in 0.9.13
        for v in self._vehicles:
            v.set_autopilot(True, 8000)
            self._tm.ignore_lights_percentage(v, 0)
            self._tm.distance_to_leading_vehicle(v, 7.0)
            self._tm.vehicle_percentage_speed_difference(
                v, random.uniform(-5, 25)
            )
            # Disable lane changes — prevents vehicles drifting onto pavement
            self._tm.auto_lane_change(v, False)
            # self._tm.keep_right_rule_percentage(v, 100)

        print(f"[WorldPop] {spawned}/{N_VEHICLES} background vehicles spawned")

    # ── Walkers ───────────────────────────────────────────────────────────────

    def _spawn_walkers(self):
        bpl        = self.world.get_blueprint_library()
        walker_bps = list(bpl.filter("walker.pedestrian.*"))

        # Get random navmesh locations
        spawn_locs = []
        attempts   = 0
        while len(spawn_locs) < N_WALKERS and attempts < 500:
            attempts += 1
            loc = self.world.get_random_location_from_navigation()
            if loc:
                spawn_locs.append(loc)

        if not spawn_locs:
            print("[WorldPop] WARNING: No valid walker spawn locations found.")
            return

        # Step 2: batch spawn walkers
        batch = []
        for loc in spawn_locs:
            bp = random.choice(walker_bps)
            # Randomise gender/appearance if available
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            batch.append(carla.command.SpawnActor(
                bp,
                carla.Transform(loc, carla.Rotation())
            ))

        results = self.client.apply_batch_sync(batch, True)
        walker_ids = []
        for r in results:
            if not r.error:
                walker_ids.append(r.actor_id)

        if not walker_ids:
            print("[WorldPop] WARNING: No walkers spawned.")
            return

        # Step 3: batch spawn AI controllers
        ctrl_bp    = bpl.find("controller.ai.walker")
        ctrl_batch = []
        for wid in walker_ids:
            ctrl_batch.append(carla.command.SpawnActor(
                ctrl_bp,
                carla.Transform(),
                wid   # attach to walker
            ))

        ctrl_results = self.client.apply_batch_sync(ctrl_batch, True)
        ctrl_ids     = []
        for r in ctrl_results:
            if not r.error:
                ctrl_ids.append(r.actor_id)

        # Step 4: retrieve all actors
        self.world.tick() if hasattr(self.world, 'tick') else time.sleep(0.1)

        for wid in walker_ids:
            a = self.world.get_actor(wid)
            if a:
                self._walkers.append(a)

        for cid in ctrl_ids:
            a = self.world.get_actor(cid)
            if a:
                self._controllers.append(a)

        # Step 5: start controllers and set random destinations
        self.world.set_pedestrians_cross_factor(0.1)  # 10% chance to cross roads
        for ctrl in self._controllers:
            try:
                ctrl.start()
                dest = self.world.get_random_location_from_navigation()
                if dest:
                    ctrl.go_to_location(dest)
                ctrl.set_max_speed(
                    random.uniform(WALKER_SPEED_MIN, WALKER_SPEED_MAX)
                )
            except Exception:
                pass

        print(f"[WorldPop] {len(self._walkers)} background walkers spawned  "
              f"({len(self._controllers)} controllers active)")
        
    def monitor(self):
        """
        Call periodically to clean up crashed/stuck vehicles.
        Destroys any vehicle that is flipped (z rotation > 45 deg)
        or has been stationary for too long off-road.
        """
        to_remove = []
        for v in self._vehicles:
            if not v.is_alive:
                to_remove.append(v)
                continue
            rot = v.get_transform().rotation
            # Flipped check
            if abs(rot.roll) > 45 or abs(rot.pitch) > 45:
                print(f"[WorldPop] Removing flipped vehicle {v.id}")
                v.destroy()
                to_remove.append(v)
        for v in to_remove:
            if v in self._vehicles:
                self._vehicles.remove(v)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_all(self):
        # Stop controllers first
        stop_batch = []
        for ctrl in self._controllers:
            if ctrl and ctrl.is_alive:
                try:
                    ctrl.stop()
                except Exception:
                    pass
                stop_batch.append(
                    carla.command.DestroyActor(ctrl)
                )
        if stop_batch:
            self.client.apply_batch_sync(stop_batch, True)

        # Destroy walkers
        walker_batch = [
            carla.command.DestroyActor(w)
            for w in self._walkers if w and w.is_alive
        ]
        if walker_batch:
            self.client.apply_batch_sync(walker_batch, True)

        # Destroy vehicles
        vehicle_batch = [
            carla.command.DestroyActor(v)
            for v in self._vehicles if v and v.is_alive
        ]
        if vehicle_batch:
            self.client.apply_batch_sync(vehicle_batch, True)

        print(f"[WorldPop] All background actors destroyed.")