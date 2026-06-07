"""
Base utilities shared across all driving anger study scenarios.
Requires: CARLA 0.9.13+, Python 3.7+
"""

import carla
import time
import math
import threading
import pygame
import numpy as np


# ── Colour constants ──────────────────────────────────────────────────────────
WHITE  = (255, 255, 255)
BLACK  = (0,   0,   0)
RED    = (220, 50,  50)
YELLOW = (255, 220, 0)
GREEN  = (50,  200, 50)

# ── Speed helpers ─────────────────────────────────────────────────────────────

def kmh_to_ms(kmh: float) -> float:
    return kmh / 3.6

def ms_to_kmh(ms: float) -> float:
    return ms * 3.6

def get_speed_kmh(actor) -> float:
    v = actor.get_velocity()
    return ms_to_kmh(math.sqrt(v.x**2 + v.y**2 + v.z**2))


# ── Simple PID longitudinal controller ───────────────────────────────────────

class SpeedController:
    """Keeps an NPC vehicle at a target speed (km/h)."""

    def __init__(self, kp=0.5, ki=0.02, kd=0.1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._integral = 0.0
        self._prev_error = 0.0

    def run_step(self, vehicle, target_kmh: float, dt: float = 0.05) -> carla.VehicleControl:
        error = target_kmh - get_speed_kmh(vehicle)
        self._integral += error * dt
        derivative = (error - self._prev_error) / max(dt, 1e-4)
        self._prev_error = error

        throttle = self.kp * error + self.ki * self._integral + self.kd * derivative
        control = carla.VehicleControl()
        if throttle >= 0:
            control.throttle = min(throttle, 1.0)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(-throttle, 1.0)
        return control


# ── HUD overlay ──────────────────────────────────────────────────────────────

class HUD:
    """Pygame-based HUD for speed warnings and rating prompts."""

    def __init__(self, width=1280, height=720):
        pygame.init()
        self.surface = pygame.display.set_mode((width, height), pygame.NOFRAME)
        pygame.display.set_caption("Driving Study")
        self.font_large = pygame.font.SysFont("Arial", 32, bold=True)
        self.font_med   = pygame.font.SysFont("Arial", 24)
        self._message: str | None = None
        self._message_color = RED
        self._prompt: str | None = None
        self._lock = threading.Lock()

    def set_message(self, text: str, color=RED):
        with self._lock:
            self._message = text
            self._message_color = color

    def set_prompt(self, text: str):
        with self._lock:
            self._prompt = text

    def render(self, speed_kmh: float):
        self.surface.fill((0, 0, 0, 0))  # transparent bg (camera renders behind)
        with self._lock:
            msg  = self._message
            pmt  = self._prompt

        # Speed HUD (top-left)
        spd_surf = self.font_large.render(f"{speed_kmh:.0f} km/h", True, WHITE)
        self.surface.blit(spd_surf, (30, 30))

        # Warning message (top-centre)
        if msg:
            rect = pygame.Rect(200, 20, 880, 70)
            pygame.draw.rect(self.surface, (0, 0, 0, 180), rect, border_radius=8)
            txt = self.font_med.render(msg, True, self._message_color)
            self.surface.blit(txt, txt.get_rect(center=rect.center))

        # Rating prompt (bottom-centre)
        if pmt:
            rect2 = pygame.Rect(100, 620, 1080, 80)
            pygame.draw.rect(self.surface, (20, 20, 80, 210), rect2, border_radius=10)
            lines = pmt.split("\n")
            y = rect2.top + 10
            for line in lines:
                t = self.font_med.render(line, True, YELLOW)
                self.surface.blit(t, t.get_rect(centerx=rect2.centerx, top=y))
                y += 30

        pygame.display.flip()


# ── Beep synchronisation ──────────────────────────────────────────────────────

def play_beep(frequency=880, duration_ms=200):
    """Single sync beep at scenario start."""
    pygame.mixer.init(frequency=44100, size=-16, channels=1, buffer=512)
    sample_rate = 44100
    n_samples = int(sample_rate * duration_ms / 1000)
    buf = np.sin(2 * np.pi * frequency * np.arange(n_samples) / sample_rate)
    buf = (buf * 32767).astype(np.int16)
    sound = pygame.sndarray.make_sound(buf)
    sound.play()
    time.sleep(duration_ms / 1000 + 0.05)


# ── Speed-control HUD logic (Scenarios 1 & 2) ────────────────────────────────

TARGET_LOW_KMH  = 90.0
TARGET_HIGH_KMH = 105.0
LOW_DURATION_S  = 20.0   # warn only after 20 s below threshold

MSG_TOO_SLOW = (
    "Please drive at 100–105 km/h. "
    "Your current speed is too low. Please accelerate to the target speed."
)
MSG_TOO_FAST = "Please drive at 100–105 km/h. Your current speed is too fast."


class SpeedMonitor:
    """Issues HUD warnings when ego speed is out of [90, 105] km/h."""

    def __init__(self, hud: HUD):
        self.hud = hud
        self._below_since: float | None = None
        self._warning_active = False

    def update(self, speed_kmh: float, t: float):
        if speed_kmh > TARGET_HIGH_KMH:
            self.hud.set_message(MSG_TOO_FAST, RED)
            self._below_since = None
            self._warning_active = True

        elif speed_kmh < TARGET_LOW_KMH:
            if self._below_since is None:
                self._below_since = t
            elif t - self._below_since >= LOW_DURATION_S:
                self.hud.set_message(MSG_TOO_SLOW, YELLOW)
                self._warning_active = True

        else:  # in target range
            if self._warning_active:
                self.hud.set_message(None)
                self._warning_active = False
            self._below_since = None


# ── Rating-prompt scheduler ───────────────────────────────────────────────────

RATING_QUESTION_VARIANTS = {
    "cut_in":
        "How much anger or frustration did you feel\ndue to the vehicle cutting in and slowing down traffic?",
    "congestion":
        "How much anger or frustration do you feel\ndue to the current traffic congestion?",
    "congestion_cut_in":
        "How much anger or frustration did you feel\ndue to the recent cut-in during congestion?",
    "merge_refusal":
        "How much anger or frustration did you feel\ndue to the merging refusal situation?",
    "shoulder_cut_in":
        "How much anger or frustration did you feel\ndue to the vehicle merging from the shoulder?",
    "shoulder_driving":
        "How much anger or frustration did you feel\ndue to vehicles driving on the shoulder?",
    "blocking_vehicle":
        "How much anger or frustration did you feel\ndue to the slow vehicle blocking your lane?",
    "delay":
        "How much anger or frustration did you feel\ndue to the delayed start of the lead vehicle?",
    "honking":
        "How much anger or frustration did you feel\ndue to the honking from the vehicle behind you?",
    "rule_violation":
        "How much anger or frustration did you feel\ndue to the traffic rule violations you just experienced?",
    "ramp_stress":
        "How much anger or frustration do you feel\ndue to the current traffic condition?",
}

class PromptScheduler:
    """Fires rating prompts at pre-defined times, shown for `display_s` seconds."""

    def __init__(self, hud: HUD, schedule: list[tuple[float, str]], display_s: float = 15.0):
        """
        schedule: list of (trigger_time_s, variant_key)
        """
        self.hud = hud
        self.schedule = sorted(schedule, key=lambda x: x[0])
        self.display_s = display_s
        self._idx = 0
        self._prompt_until: float | None = None

    def update(self, t: float):
        # Hide expired prompts
        if self._prompt_until is not None and t >= self._prompt_until:
            self.hud.set_prompt(None)
            self._prompt_until = None

        # Fire next prompt
        if self._idx < len(self.schedule):
            trigger, key = self.schedule[self._idx]
            if t >= trigger:
                text = RATING_QUESTION_VARIANTS.get(key, key)
                self.hud.set_prompt(text)
                self._prompt_until = t + self.display_s
                self._idx += 1


# ── Indicator helper ──────────────────────────────────────────────────────────

def set_indicator(vehicle, direction: str):
    """direction: 'left' | 'right' | None"""
    state = carla.VehicleLightState.NONE
    if direction == "left":
        state = carla.VehicleLightState.LeftBlinker
    elif direction == "right":
        state = carla.VehicleLightState.RightBlinker
    vehicle.set_light_state(carla.VehicleLightState(state))


# ── Spawn helper ──────────────────────────────────────────────────────────────

def spawn_vehicle(world, bp_filter: str, transform: carla.Transform,
                  color: str) -> carla.Vehicle:
    bpl = world.get_blueprint_library()
    bp = bpl.filter(bp_filter)[0]
    if color and bp.has_attribute("color"):
        bp.set_attribute("color", color)
    actor = world.try_spawn_actor(bp, transform)
    if actor is None:
        raise RuntimeError(f"Failed to spawn {bp_filter} at {transform.location}")
    return actor


# ── Waypoint follower (lane-keeping) ─────────────────────────────────────────

class WaypointFollower:
    """
    Moves an NPC along the road using CARLA waypoints.
    Call `run_step()` at ~20 Hz.
    """

    def __init__(self, vehicle, world, target_speed_kmh: float = 100.0):
        self.vehicle = vehicle
        self.world = world
        self.map = world.get_map()
        self.target_speed_kmh = target_speed_kmh
        self._speed_ctrl = SpeedController()
        self._next_wp: carla.Waypoint | None = None

    def run_step(self, dt: float = 0.05) -> None:
        loc = self.vehicle.get_location()
        wp = self.map.get_waypoint(loc, project_to_road=True)

        # Advance look-ahead proportional to speed
        speed = get_speed_kmh(self.vehicle)
        look_ahead = max(5.0, speed * 0.5)

        nexts = wp.next(look_ahead)
        if nexts:
            target_wp = nexts[0]
            target_loc = target_wp.transform.location

            # Steering
            ego_tf = self.vehicle.get_transform()
            fwd = ego_tf.get_forward_vector()
            to_tgt = target_loc - loc
            cross = fwd.x * to_tgt.y - fwd.y * to_tgt.x
            steer = max(-1.0, min(1.0, cross * 0.5))

            ctrl = self._speed_ctrl.run_step(self.vehicle, self.target_speed_kmh, dt)
            ctrl.steer = steer
            self.vehicle.apply_control(ctrl)