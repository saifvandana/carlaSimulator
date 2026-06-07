"""
Scenario 2: Merge Failure + Congestion + Shoulder Violations + Blocking Vehicle
Entry ramp (1 lane) → 3-lane highway (ramp → lane 3).
"""

import carla
import time
import random
from scenario_base import (
    HUD, SpeedMonitor, PromptScheduler, WaypointFollower,
    SpeedController, spawn_vehicle, set_indicator,
    get_speed_kmh, play_beep
)

# ── Rating-prompt schedule ────────────────────────────────────────────────────
PROMPT_SCHEDULE = [
    (50.0,  "ramp_stress"),
    (185.0, "merge_refusal"),
    (270.0, "shoulder_cut_in"),
    (335.0, "shoulder_driving"),
    (390.0, "shoulder_cut_in"),
    (445.0, "shoulder_driving"),
    (745.0, "blocking_vehicle"),
]

# ── Driver instruction texts ──────────────────────────────────────────────────
INSTR_65S  = "Turn on the left signal and merge into the main lane."
INSTR_220S = "Maintain a safe distance and follow the traffic speed."

HORN_INTERVAL = 5.0   # seconds between horn toots while blocked


class ShoulderVehicle:
    """An NPC that runs on the shoulder at 50–60 km/h, optionally cuts in."""

    def __init__(self, vehicle, cut_in_at: float | None = None):
        self.vehicle   = vehicle
        self.cut_in_at = cut_in_at          # abs sim time to cut in front
        self._did_cut  = False
        self._speed_ctrl = SpeedController()
        self.speed_kmh = random.uniform(50, 60)
        self._follower = None               # set by caller after world ref available

    def update(self, t: float, dt: float, world):
        if self._follower is None:
            self._follower = WaypointFollower(self.vehicle, world, self.speed_kmh)

        if self.cut_in_at and t >= self.cut_in_at and not self._did_cut:
            set_indicator(self.vehicle, "left")
            self._did_cut = True

        self._follower.run_step(dt)


class BlockingLeader:
    """Drives at 60 km/h ahead of ego in lane 1 after t=460."""

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self._ctrl   = SpeedController()

    def update(self, dt: float):
        ctrl = self._ctrl.run_step(self.vehicle, 60.0, dt)
        self.vehicle.apply_control(ctrl)


class Scenario2:

    def __init__(self, client: carla.Client, ego: carla.Vehicle,
                 ramp_transforms: list[carla.Transform],
                 highway_transforms: list[carla.Transform],
                 shoulder_transforms: list[carla.Transform]):
        self.client = client
        self.world  = client.get_world()
        self.ego    = ego

        # Spawn-point lists provided by caller (map-specific)
        self.ramp_tfs     = ramp_transforms
        self.highway_tfs  = highway_transforms
        self.shoulder_tfs = shoulder_transforms

        self.hud           = HUD()
        self.speed_monitor = SpeedMonitor(self.hud)
        self.prompt_sched  = PromptScheduler(self.hud, PROMPT_SCHEDULE)

        self._npcs: list[carla.Vehicle] = []
        self._blocking_npcs: list[carla.Vehicle] = []   # lane change blockers
        self._shoulder_agents: list[ShoulderVehicle] = []
        self._blocking_leader: BlockingLeader | None = None

        self._horn_last = 0.0
        self._blocking_spawned = False
        self._instr_65_shown  = False
        self._instr_220_shown = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def run(self):
        play_beep()
        t_start = time.time()
        self._spawn_merge_blockers()

        try:
            while True:
                t  = time.time() - t_start
                dt = 0.05
                self._tick(t, dt)
                if t > 780.0:
                    break
                time.sleep(dt)
        finally:
            self._cleanup()

    # ── Per-tick ──────────────────────────────────────────────────────────────

    def _tick(self, t: float, dt: float):
        speed = get_speed_kmh(self.ego)
        self.hud.render(speed)
        self.speed_monitor.update(speed, t)
        self.prompt_sched.update(t)

        # Driver instructions
        if not self._instr_65_shown and t >= 65.0:
            self.hud.set_message(INSTR_65S, color=(100, 220, 255))
            self._instr_65_shown = True
        if not self._instr_220_shown and t >= 220.0:
            self.hud.set_message(INSTR_220S, color=(100, 220, 255))
            self._instr_220_shown = True

        # Horn pressure from behind (120–180 s, every 5 s)
        if 120.0 <= t <= 180.0:
            if t - self._horn_last >= HORN_INTERVAL:
                self._play_horn()
                self._horn_last = t

        # Shoulder events (220–440 s)
        if 220.0 <= t <= 440.0:
            if not self._shoulder_agents:
                self._spawn_shoulder_vehicles()
            for sa in self._shoulder_agents:
                sa.update(t, dt, self.world)

        # Recovery (440 s): clear congestion blockers
        if t >= 440.0:
            for npc in self._blocking_npcs:
                if npc.is_alive:
                    # gently speed up
                    ctrl = carla.VehicleControl(throttle=0.6, brake=0.0)
                    npc.apply_control(ctrl)

        # Blocking leader (460 s)
        if t >= 460.0 and not self._blocking_spawned:
            self._spawn_blocking_leader()
            self._blocking_spawned = True
        if self._blocking_leader:
            self._blocking_leader.update(dt)

    # ── Spawns ────────────────────────────────────────────────────────────────

    def _spawn_merge_blockers(self):
        """Dense traffic in main lane 3 to block ego merge until ~180 s."""
        bp_filter = "vehicle.audi.tt"
        for i, tf in enumerate(self.highway_tfs[:8]):
            try:
                npc = spawn_vehicle(self.world, bp_filter, tf)
                follower = WaypointFollower(npc, self.world, target_speed_kmh=90.0)
                # All run via blocking_npcs list (re-used for recovery phase)
                self._blocking_npcs.append(npc)
                self._npcs.append(npc)
                # Start follower in background thread
                import threading
                th = threading.Thread(target=self._run_follower, args=(follower,), daemon=True)
                th.start()
            except RuntimeError as e:
                print(f"[Scenario2] Merge blocker spawn failed: {e}")

    def _run_follower(self, follower: WaypointFollower):
        while True:
            follower.run_step(0.05)
            time.sleep(0.05)

    def _spawn_shoulder_vehicles(self):
        """
        B1 (240 s): pass only
        B2 (300 s): cut in front (cut_in_at=330 s)
        B3 (360 s): pass only
        B4 (400 s): cut in front (cut_in_at=420 s)
        """
        configs = [
            (240.0, None),
            (300.0, 330.0),
            (360.0, None),
            (400.0, 420.0),
        ]
        bp = "vehicle.mercedes.coupe_2020"
        for i, (appear_t, cut_t) in enumerate(configs):
            tf = self.shoulder_tfs[min(i, len(self.shoulder_tfs) - 1)]
            try:
                npc = spawn_vehicle(self.world, bp, tf, color="200,50,50")
                agent = ShoulderVehicle(npc, cut_in_at=cut_t)
                self._shoulder_agents.append(agent)
                self._npcs.append(npc)
            except RuntimeError as e:
                print(f"[Scenario2] Shoulder vehicle spawn failed: {e}")

    def _spawn_blocking_leader(self):
        """Slow vehicle in lane 1 at ~60 km/h, surrounded by lane-change blockers."""
        tf = self.highway_tfs[-1] if self.highway_tfs else None
        if tf is None:
            return
        try:
            leader = spawn_vehicle(self.world, "vehicle.volkswagen.t2", tf, color="50,50,200")
            self._blocking_leader = BlockingLeader(leader)
            self._npcs.append(leader)
        except RuntimeError as e:
            print(f"[Scenario2] Blocking leader spawn failed: {e}")

    def _play_horn(self):
        """Trigger a brief horn sound effect."""
        try:
            import pygame
            pygame.mixer.Sound("assets/horn.wav").play()
        except Exception:
            pass   # gracefully skip if asset missing

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def _cleanup(self):
        for npc in self._npcs:
            if npc.is_alive:
                npc.destroy()
        self.hud.set_message(None)
        self.hud.set_prompt(None)
        print("[Scenario2] Complete.")