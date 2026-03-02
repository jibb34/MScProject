"""
Galloway Run/Walk Training Plan Generator
==========================================
Generates a personalised back-to-running schedule following Jeff Galloway's
Run/Walk/Run methodology.

Key principles implemented
--------------------------
* Run-walk intervals sized by estimated current fitness and body weight
* Weight-adjusted pace (Galloway adds ~30 s/km per 10 kg above reference weight)
* Three sessions per week: two shorter runs + one weekly long run
* Gradual progression with every 4th week as a "step-back" (recovery) week
* Long run increases by ~10 % each non-step-back week

Usage
-----
    python galloway_plan.py --weight 95 --target-weight 70 --weeks 26
    python galloway_plan.py --weight 95 --target-weight 70 --weeks 26 --output plan.json
"""

import argparse
import json
import math
from dataclasses import dataclass, asdict, field
from typing import List


# ---------------------------------------------------------------------------
# Constants & pace helpers
# ---------------------------------------------------------------------------

# Reference body weight for pace adjustment (kg).  Galloway's original work
# used lbs; we translate: ~10 lb ≈ 4.5 kg ≈ +30 s/mile ≈ +19 s/km.
_PACE_ADJUST_PER_KG_OVER_REF = 19 / 4.5  # seconds per km per kg over reference
_REF_WEIGHT_KG = 65.0  # approximate lean-elite reference weight (kg)

# Galloway run/walk ratio tiers keyed by estimated pace (s/km).
# Each entry: (max_pace_s_per_km, run_seconds, walk_seconds, label)
_RW_TIERS = [
    (300,  60, 30,  "1:00 run / 0:30 walk"),   # < 5:00 /km  — fit
    (360,  60, 60,  "1:00 run / 1:00 walk"),   # 5–6 min/km
    (420,  60, 90,  "1:00 run / 1:30 walk"),   # 6–7 min/km
    (480,  30, 60,  "0:30 run / 1:00 walk"),   # 7–8 min/km
    (540,  30, 90,  "0:30 run / 1:30 walk"),   # 8–9 min/km
    (math.inf, 30, 120, "0:30 run / 2:00 walk"),  # ≥ 9 min/km — beginner
]


def _run_walk_ratio(pace_s_per_km: float):
    """Return (run_s, walk_s, label) for the given pace."""
    for max_pace, run_s, walk_s, label in _RW_TIERS:
        if pace_s_per_km < max_pace:
            return run_s, walk_s, label
    return 30, 120, "0:30 run / 2:00 walk"


def _weight_adjusted_pace(base_pace_s_per_km: float, weight_kg: float) -> float:
    """Slow the base pace proportionally for body weight above reference."""
    excess_kg = max(0.0, weight_kg - _REF_WEIGHT_KG)
    return base_pace_s_per_km + excess_kg * _PACE_ADJUST_PER_KG_OVER_REF


def _fmt_pace(s_per_km: float) -> str:
    """Format seconds-per-km as 'M:SS /km'."""
    mins, secs = divmod(int(s_per_km), 60)
    return f"{mins}:{secs:02d} /km"


def _fmt_time(total_seconds: int) -> str:
    """Format total seconds as 'H:MM:SS' or 'MM:SS'."""
    h, rem = divmod(total_seconds, 3600)
    m, s = divmod(rem, 60)
    if h:
        return f"{h}:{m:02d}:{s:02d}"
    return f"{m}:{s:02d}"


# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------

@dataclass
class Session:
    day: int                    # 1–7 within the week
    session_type: str           # "short", "long", "rest"
    duration_min: int           # total session time in minutes
    run_seconds: int            # run interval (0 for rest)
    walk_seconds: int           # walk interval (0 for rest)
    ratio_label: str            # human-readable ratio
    est_distance_km: float      # rough estimate
    pace_label: str             # e.g. "8:30 /km"
    notes: str = ""


@dataclass
class Week:
    week_number: int
    phase: str
    phase_description: str
    is_stepback: bool
    current_weight_kg: float    # estimated weight at this week
    sessions: List[Session] = field(default_factory=list)
    weekly_notes: str = ""


# ---------------------------------------------------------------------------
# Plan builder
# ---------------------------------------------------------------------------

class GallowayPlanBuilder:
    """
    Builds a multi-week Galloway Run/Walk training plan.

    Parameters
    ----------
    start_weight_kg : float
        Athlete's current body weight in kg.
    target_weight_kg : float
        Desired body weight in kg.
    total_weeks : int
        Length of the plan in weeks (default 26).
    elite_background : bool
        True if the athlete has an elite running background.  Used to set a
        slightly more aggressive base pace and progression rate.
    """

    # Typical initial Magic Mile estimate for a very deconditioned runner (s/km)
    _BASE_PACE_DECONDITIONED = 480.0   # 8:00/km — conservative start
    _BASE_PACE_ELITE_BG      = 420.0   # 7:00/km — better mechanics even unfit

    # Duration (minutes) for short and long sessions at start and end of plan
    _SHORT_START_MIN = 20
    _SHORT_END_MIN   = 40
    _LONG_START_MIN  = 25
    _LONG_END_MIN    = 60

    def __init__(
        self,
        start_weight_kg: float,
        target_weight_kg: float,
        total_weeks: int = 26,
        elite_background: bool = True,
    ):
        self.start_weight_kg  = start_weight_kg
        self.target_weight_kg = target_weight_kg
        self.total_weeks      = total_weeks
        self.elite_background = elite_background

        # Intrinsic (unweighted) base pace for this athlete's background level.
        # Weight adjustment is applied separately at each week's current weight so
        # that the run/walk tier progresses as both fitness improves and body
        # weight decreases.
        self._base_pace_s_km = (
            self._BASE_PACE_ELITE_BG
            if elite_background
            else self._BASE_PACE_DECONDITIONED
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _current_weight(self, week: int) -> float:
        """Linear interpolation of expected weight through the plan."""
        progress = (week - 1) / max(1, self.total_weeks - 1)
        return self.start_weight_kg - progress * (
            self.start_weight_kg - self.target_weight_kg
        )

    def _current_pace(self, week: int) -> float:
        """Pace improves as fitness builds and weight drops."""
        weight = self._current_weight(week)
        # Fitness improvement: pace drops ~1 s/km per week, capped at 30 % gain
        fitness_gain_s = min(week * 1.0, self._base_pace_s_km * 0.30)
        improved_base  = self._base_pace_s_km - fitness_gain_s
        raw_pace = _weight_adjusted_pace(improved_base, weight)
        return max(raw_pace, 270.0)  # floor at 4:30/km (elite ceiling)

    def _session_durations(self, week: int, is_stepback: bool):
        """Return (short_min, long_min) for this week."""
        t = (week - 1) / max(1, self.total_weeks - 1)
        short = self._SHORT_START_MIN + t * (self._SHORT_END_MIN - self._SHORT_START_MIN)
        long  = self._LONG_START_MIN  + t * (self._LONG_END_MIN  - self._LONG_START_MIN)
        if is_stepback:
            short *= 0.80
            long  *= 0.80
        return int(round(short)), int(round(long))

    def _phase_info(self, week: int):
        """Return (phase_name, phase_description) for the given week."""
        boundaries = [
            (4,  "Phase 1 — Adaptation",
                 "Introduce impact loading gently. Focus on easy effort and form."),
            (8,  "Phase 2 — Base Building",
                 "Build aerobic base. Extend walk-run sessions. Stay conversational."),
            (12, "Phase 3 — Extending Base",
                 "Increase run intervals. Begin to feel the rhythm of run/walk."),
            (16, "Phase 4 — Building Endurance",
                 "Longer run segments. Weekly long run becomes the cornerstone session."),
            (20, "Phase 5 — Stamina Development",
                 "Sustained run blocks with short walk breaks. Pace starting to drop."),
            (math.inf, "Phase 6 — Continuous Running",
                 "Approach continuous running. Walk breaks remain available as insurance."),
        ]
        for limit, name, desc in boundaries:
            if week <= limit:
                return name, desc
        return "Phase 6 — Continuous Running", boundaries[-1][2]

    @staticmethod
    def _est_distance(duration_min: float, pace_s_per_km: float) -> float:
        """Rough distance given duration and pace."""
        if pace_s_per_km <= 0:
            return 0.0
        return round((duration_min * 60) / pace_s_per_km, 2)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(self) -> List[Week]:
        weeks: List[Week] = []

        for w in range(1, self.total_weeks + 1):
            is_stepback = (w % 4 == 0)
            pace        = self._current_pace(w)
            run_s, walk_s, ratio_label = _run_walk_ratio(pace)
            pace_label  = _fmt_pace(pace)
            weight      = self._current_weight(w)
            short_min, long_min = self._session_durations(w, is_stepback)
            phase_name, phase_desc = self._phase_info(w)

            notes = []
            if w == 1:
                notes.append(
                    "Start very easy. The goal this week is simply to establish "
                    "the habit of running three times per week."
                )
            if is_stepback:
                notes.append(
                    "Step-back week — reduce volume by ~20 % to allow adaptation. "
                    "Do not skip; easy weeks prevent injury."
                )
            if w % 4 == 1 and w > 1:
                notes.append(
                    "After last week's recovery, you should feel fresher. "
                    "Gently increase effort on the long run."
                )
            if weight <= self.target_weight_kg + 5:
                notes.append(
                    f"Approaching target weight ({self.target_weight_kg} kg). "
                    "Maintain consistent nutrition; do not cut calories aggressively."
                )

            sessions = [
                # Day 1 — short run
                Session(
                    day=1,
                    session_type="short",
                    duration_min=short_min,
                    run_seconds=run_s,
                    walk_seconds=walk_s,
                    ratio_label=ratio_label,
                    est_distance_km=self._est_distance(short_min, pace),
                    pace_label=pace_label,
                    notes=(
                        "Warm up 5 min walk before starting intervals. "
                        "Cool down 3 min walk at the end."
                    ),
                ),
                # Day 2 — rest
                Session(
                    day=2,
                    session_type="rest",
                    duration_min=0,
                    run_seconds=0,
                    walk_seconds=0,
                    ratio_label="—",
                    est_distance_km=0.0,
                    pace_label="—",
                    notes="Rest or gentle cross-training (swim/bike/yoga). No running.",
                ),
                # Day 3 — short run
                Session(
                    day=3,
                    session_type="short",
                    duration_min=short_min,
                    run_seconds=run_s,
                    walk_seconds=walk_s,
                    ratio_label=ratio_label,
                    est_distance_km=self._est_distance(short_min, pace),
                    pace_label=pace_label,
                    notes=(
                        "Same structure as Day 1. Focus on relaxed shoulders "
                        "and short, light foot-strikes."
                    ),
                ),
                # Day 4 — rest
                Session(
                    day=4,
                    session_type="rest",
                    duration_min=0,
                    run_seconds=0,
                    walk_seconds=0,
                    ratio_label="—",
                    est_distance_km=0.0,
                    pace_label="—",
                    notes="Rest or gentle cross-training.",
                ),
                # Day 5 — rest
                Session(
                    day=5,
                    session_type="rest",
                    duration_min=0,
                    run_seconds=0,
                    walk_seconds=0,
                    ratio_label="—",
                    est_distance_km=0.0,
                    pace_label="—",
                    notes="Full rest day.",
                ),
                # Day 6 — long run
                Session(
                    day=6,
                    session_type="long",
                    duration_min=long_min,
                    run_seconds=run_s,
                    walk_seconds=walk_s,
                    ratio_label=ratio_label,
                    est_distance_km=self._est_distance(long_min, pace),
                    pace_label=pace_label,
                    notes=(
                        "The most important session of the week. "
                        "Run 60–90 s/km SLOWER than your short-run pace. "
                        "Take all walk breaks — do not skip them regardless of how good you feel."
                    ),
                ),
                # Day 7 — rest
                Session(
                    day=7,
                    session_type="rest",
                    duration_min=0,
                    run_seconds=0,
                    walk_seconds=0,
                    ratio_label="—",
                    est_distance_km=0.0,
                    pace_label="—",
                    notes="Full rest day. Prioritise sleep and hydration.",
                ),
            ]

            weeks.append(
                Week(
                    week_number=w,
                    phase=phase_name,
                    phase_description=phase_desc,
                    is_stepback=is_stepback,
                    current_weight_kg=round(weight, 1),
                    sessions=sessions,
                    weekly_notes=" ".join(notes).strip(),
                )
            )

        return weeks


# ---------------------------------------------------------------------------
# Output formatters
# ---------------------------------------------------------------------------

def _print_plan(weeks: List[Week]) -> None:
    day_names = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]

    for week in weeks:
        tag = " [STEP-BACK WEEK]" if week.is_stepback else ""
        print(f"\n{'=' * 70}")
        print(f"  Week {week.week_number:>2}  |  {week.phase}{tag}")
        print(f"  Estimated weight: {week.current_weight_kg} kg")
        print(f"  {week.phase_description}")
        if week.weekly_notes:
            print(f"\n  NOTE: {week.weekly_notes}")
        print(f"{'=' * 70}")

        for s in week.sessions:
            day_name = day_names[s.day - 1]
            if s.session_type == "rest":
                print(f"  {day_name:<12}  REST  — {s.notes}")
                continue

            duration_str = _fmt_time(s.duration_min * 60)
            print(
                f"  {day_name:<12}  {s.session_type.upper():<6}  "
                f"{duration_str}  |  {s.ratio_label}  |  "
                f"~{s.est_distance_km:.1f} km @ {s.pace_label}"
            )
            if s.notes:
                print(f"               ↳ {s.notes}")


def _export_json(weeks: List[Week], path: str) -> None:
    data = [asdict(w) for w in weeks]
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print(f"Plan written to {path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args():
    ap = argparse.ArgumentParser(
        description=(
            "Generate a Jeff Galloway Run/Walk back-to-running plan.\n\n"
            "The plan is calibrated for body weight, target weight, and "
            "running background. It follows Galloway's run/walk/run method "
            "with a step-back (recovery) week every 4th week."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--weight", type=float, required=True,
        metavar="KG", help="Current body weight in kg (e.g. 95)"
    )
    ap.add_argument(
        "--target-weight", type=float, required=True,
        metavar="KG", help="Target body weight in kg (e.g. 70)"
    )
    ap.add_argument(
        "--weeks", type=int, default=26,
        metavar="N", help="Total plan length in weeks (default: 26)"
    )
    ap.add_argument(
        "--no-elite", action="store_true",
        help="Disable elite-background adjustment (slower initial pace)"
    )
    ap.add_argument(
        "--output", metavar="FILE",
        help="Write plan as JSON to FILE (optional)"
    )
    return ap.parse_args()


def main():
    args = _parse_args()

    builder = GallowayPlanBuilder(
        start_weight_kg=args.weight,
        target_weight_kg=args.target_weight,
        total_weeks=args.weeks,
        elite_background=not args.no_elite,
    )

    print(
        f"\nGalloway Run/Walk Training Plan\n"
        f"  Start weight : {args.weight} kg\n"
        f"  Target weight: {args.target_weight} kg\n"
        f"  Duration     : {args.weeks} weeks\n"
        f"  Background   : {'Elite runner (pace-adjusted)' if not args.no_elite else 'General runner'}\n"
    )

    plan = builder.build()
    _print_plan(plan)

    if args.output:
        _export_json(plan, args.output)


if __name__ == "__main__":
    main()
