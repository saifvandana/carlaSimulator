"""
HUD — CARLA debug string overlay + TTS voice prompts
======================================================
No pygame window. Everything renders in the CARLA spectator view
as bold debug strings. Rating prompts also spoken via TTS.

Usage:
    from hud import HUD
    hud = HUD(world)
    hud.start_tts()
    hud.countdown(ego)
    hud.update(t, ego, phase, remaining)
"""

import carla
import math
import time
import subprocess
import sys


# ── Colours ───────────────────────────────────────────────────────────────────

WHITE      = carla.Color(255, 255, 255)
YELLOW     = carla.Color(255, 220,   0)
ORANGE     = carla.Color(255, 140,   0)
LIGHT_BLUE = carla.Color(100, 200, 255)
GREEN      = carla.Color( 50, 220,  50)
RED        = carla.Color(220,  50,  50)
BLACK      = carla.Color(  0,   0,   0)
GREY       = carla.Color(180, 180, 180)

LIFE = 0.12   # life_time for debug strings — just over one tick

# ── Instructions ──────────────────────────────────────────────────────────────
# (trigger_t, display_s, display_text, spoken_text or None)

INSTRUCTIONS = [
    ( 0.0, 25.0,
      "Speed limit: 50 km/h  |  Drive EAST and join the queue ahead",
      "Speed limit is 50 kilometres per hour. Drive forward and join the queue ahead."),
]

# ── Rating prompts ────────────────────────────────────────────────────────────
# (trigger_t, display_s, title, line1, line2, spoken_text)

PROMPTS = [
    (
        125.0, 15.0,
        ">>> RATING 1/3 <<<",
        "How much anger or frustration did you feel",
        "due to the delayed start of the lead vehicle?",
        "Rating question one. How much anger or frustration did you feel "
        "due to the delayed start of the lead vehicle? "
        "Please respond: 1 for none, 2 for slight, 3 for moderate, "
        "4 for strong, 5 for extreme.",
    ),
    (
        240.0, 15.0,
        ">>> RATING 2/3 <<<",
        "How much anger or frustration did you feel",
        "due to the honking from the vehicle behind you?",
        "Rating question two. How much anger or frustration did you feel "
        "due to the honking from the vehicle behind you? "
        "Please respond: 1 for none, 2 for slight, 3 for moderate, "
        "4 for strong, 5 for extreme.",
    ),
    (
        325.0, 15.0,
        ">>> RATING 3/3 <<<",
        "How much anger or frustration did you feel",
        "due to the traffic rule violations you experienced?",
        "Rating question three. How much anger or frustration did you feel "
        "due to the traffic rule violations you just experienced? "
        "Please respond: 1 for none, 2 for slight, 3 for moderate, "
        "4 for strong, 5 for extreme.",
    ),
]

SCALE_TEXT = "1=None  2=Slight  3=Moderate  4=Strong  5=Extreme"


# ── TTS ───────────────────────────────────────────────────────────────────────

def _speak(text: str):
    """Speak text in a separate process — no audio device conflicts."""
    script = (
        "import pyttsx3; e=pyttsx3.init(); "
        "e.setProperty('rate',155); e.setProperty('volume',1.0); "
        f"e.say({repr(text)}); e.runAndWait()"
    )
    subprocess.Popen(
        [sys.executable, "-c", script],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )


# ── Bold debug string helper ──────────────────────────────────────────────────

def _bold(debug, loc, text, color, life=LIFE):
    """Draw text with black outline for bold effect in CARLA view."""
    offsets = [
        (0.0,  0.08), (0.0, -0.08),
        (0.08,  0.0), (-0.08,  0.0),
        (0.06,  0.06), (-0.06,  0.06),
        (0.06, -0.06), (-0.06, -0.06),
    ]
    for dx, dy in offsets:
        debug.draw_string(
            carla.Location(x=loc.x + dx, y=loc.y + dy, z=loc.z),
            text, draw_shadow=False,
            color=BLACK, life_time=life
        )
    debug.draw_string(loc, text, draw_shadow=True,
                      color=color, life_time=life)


# ── HUD class ─────────────────────────────────────────────────────────────────

class HUD:

    def __init__(self, world):
        self.world       = world
        self.debug       = world.debug
        self._instr_idx  = 0
        self._instr_end  = -1.0
        self._active_instr = None
        self._prompt_idx = 0
        self._prompt_end = -1.0
        self._active_prompt = None   # (title, l1, l2, end, dur)

    def start_tts(self):
        """Warm up TTS subprocess (optional — avoids first-call delay)."""
        _speak("Ready.")
        print("[HUD] TTS ready.")

    # ── Countdown ─────────────────────────────────────────────────────────────

    def countdown(self, ego):
        """Blocking 3-2-1-GO shown in CARLA world and spoken aloud."""
        loc = ego.get_location()
        anchor = carla.Location(x=loc.x + 5, y=loc.y - 3, z=14.0)

        steps = [
            ("  3",   RED,    "3"),
            ("  2",   YELLOW, "2"),
            ("  1",   GREEN,  "1"),
            ("  GO!", carla.Color(50, 255, 50), "Go!"),
        ]

        for label, color, spoken in steps:
            _speak(spoken)
            t_end = time.time() + 1.0
            while time.time() < t_end:
                _bold(self.debug, anchor, label, color, life=0.15)
                time.sleep(0.05)

        print("[HUD] Countdown complete — scenario starting.")

    # ── Main update ───────────────────────────────────────────────────────────

    def update(self, t, ego, phase="RED", remaining=0.0):
        loc = ego.get_location()
        v   = ego.get_velocity()
        spd = math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6

        # Anchor text slightly ahead and to the side of ego
        ax = loc.x + 5
        ay = loc.y - 3

        self._draw_speed(spd, ax, ay)
        self._draw_phase(phase, remaining, ax, ay)
        # self._draw_instruction(t, ax, ay)
        self._draw_prompt(t, ax, ay)

    # ── Speed ─────────────────────────────────────────────────────────────────

    def _draw_speed(self, spd, ax, ay):
        col = GREEN if spd <= 55.0 else RED
        _bold(self.debug,
              carla.Location(x=ax, y=ay, z=12.0),
              f"  {spd:.0f} km/h", col)

    # ── Phase ─────────────────────────────────────────────────────────────────

    def _draw_phase(self, phase, remaining, ax, ay):
        cols = {"RED": RED, "YELLOW": YELLOW, "GREEN": GREEN}
        col  = cols.get(phase, WHITE)
        _bold(self.debug,
              carla.Location(x=ax, y=ay, z=10.5),
              f"  {phase}  {remaining:.0f}s", col)

    # ── Instructions ──────────────────────────────────────────────────────────

    def _draw_instruction(self, t, ax, ay):
        if self._instr_idx < len(INSTRUCTIONS):
            entry   = INSTRUCTIONS[self._instr_idx]
            trigger, dur, text = entry[0], entry[1], entry[2]
            spoken  = entry[3] if len(entry) > 3 else None
            if t >= trigger:
                self._active_instr = text
                self._instr_end    = t + dur
                self._instr_idx   += 1
                if spoken:
                    _speak(spoken)

        if t >= self._instr_end:
            self._active_instr = None

        if self._active_instr:
            _bold(self.debug,
                  carla.Location(x=ax, y=ay, z=9.0),
                  f"  {self._active_instr}", LIGHT_BLUE)

    # ── Rating prompts ────────────────────────────────────────────────────────

    def _draw_prompt(self, t, ax, ay):
        if self._prompt_idx < len(PROMPTS):
            trigger, dur, title, l1, l2, spoken = PROMPTS[self._prompt_idx]
            if t >= trigger:
                self._active_prompt = (title, l1, l2, t + dur, dur)
                self._prompt_end    = t + dur
                self._prompt_idx   += 1
                _speak(spoken)
                print(f"\n[HUD] *** {title} ***\n      {l1}")

        if t >= self._prompt_end:
            self._active_prompt = None

        # if self._active_prompt:
        #     title, l1, l2, end, dur = self._active_prompt
        #     left = max(0.0, end - t)

        #     # Flash title
        #     title_col = YELLOW if int(t * 2) % 2 == 0 else ORANGE

        #     _bold(self.debug,
        #           carla.Location(x=ax, y=ay, z=8.0),
        #           f"  {title}", title_col)
        #     _bold(self.debug,
        #           carla.Location(x=ax, y=ay, z=6.8),
        #           f"  {l1}", WHITE)
        #     _bold(self.debug,
        #           carla.Location(x=ax, y=ay, z=5.6),
        #           f"  {l2}", WHITE)
        #     _bold(self.debug,
        #           carla.Location(x=ax, y=ay, z=4.4),
        #           f"  {SCALE_TEXT}", GREY)

        #     # Countdown bar (text-based)
        #     filled = int((left / dur) * 20)
        #     bar    = "█" * filled + "░" * (20 - filled)
        #     _bold(self.debug,
        #           carla.Location(x=ax, y=ay, z=3.2),
        #           f"  [{bar}]  {left:.0f}s", YELLOW)