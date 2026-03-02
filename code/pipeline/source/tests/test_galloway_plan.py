"""
Unit tests for galloway_plan.py
"""

import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from galloway_plan import (
    GallowayPlanBuilder,
    _run_walk_ratio,
    _weight_adjusted_pace,
    _fmt_pace,
    _fmt_time,
)


# ---------------------------------------------------------------------------
# Pace helpers
# ---------------------------------------------------------------------------

class TestRunWalkRatio:
    def test_fast_pace_gives_short_walk(self):
        run_s, walk_s, label = _run_walk_ratio(280)
        assert run_s == 60
        assert walk_s == 30

    def test_slow_pace_gives_long_walk(self):
        run_s, walk_s, label = _run_walk_ratio(600)
        assert run_s == 30
        assert walk_s == 120

    def test_label_is_string(self):
        _, _, label = _run_walk_ratio(400)
        assert isinstance(label, str)
        assert "run" in label.lower()

    def test_all_tiers_covered(self):
        """Every pace boundary should return a valid ratio."""
        for pace in [270, 310, 370, 430, 490, 550, 700]:
            run_s, walk_s, label = _run_walk_ratio(pace)
            assert run_s > 0
            assert walk_s > 0


class TestWeightAdjustedPace:
    def test_no_adjustment_at_reference_weight(self):
        base = 360.0
        adjusted = _weight_adjusted_pace(base, 65.0)
        assert adjusted == base

    def test_heavier_runner_is_slower(self):
        base = 360.0
        adjusted_heavy = _weight_adjusted_pace(base, 95.0)
        adjusted_light = _weight_adjusted_pace(base, 70.0)
        assert adjusted_heavy > adjusted_light > base

    def test_lighter_than_reference_not_faster(self):
        """We do not apply negative adjustments for athletes under reference weight."""
        base = 360.0
        adjusted = _weight_adjusted_pace(base, 50.0)
        assert adjusted == base


class TestFormatters:
    def test_fmt_pace_round_trip(self):
        # 8:00/km = 480 s/km
        assert _fmt_pace(480) == "8:00 /km"

    def test_fmt_pace_non_integer(self):
        result = _fmt_pace(390.0)  # 6:30
        assert result == "6:30 /km"

    def test_fmt_time_minutes_only(self):
        assert _fmt_time(1800) == "30:00"

    def test_fmt_time_with_hours(self):
        assert _fmt_time(3661) == "1:01:01"


# ---------------------------------------------------------------------------
# Plan builder
# ---------------------------------------------------------------------------

class TestGallowayPlanBuilder:
    def _make_builder(self, **kwargs):
        defaults = dict(start_weight_kg=95.0, target_weight_kg=70.0, total_weeks=26)
        defaults.update(kwargs)
        return GallowayPlanBuilder(**defaults)

    # --- plan length -------------------------------------------------------

    def test_correct_number_of_weeks(self):
        builder = self._make_builder(total_weeks=26)
        plan = builder.build()
        assert len(plan) == 26

    def test_short_plan(self):
        builder = self._make_builder(total_weeks=8)
        plan = builder.build()
        assert len(plan) == 8

    # --- step-back weeks ---------------------------------------------------

    def test_every_fourth_week_is_stepback(self):
        plan = self._make_builder().build()
        for week in plan:
            expected = (week.week_number % 4 == 0)
            assert week.is_stepback == expected, (
                f"Week {week.week_number}: expected is_stepback={expected}"
            )

    def test_stepback_shorter_than_previous(self):
        plan = self._make_builder().build()
        for i, week in enumerate(plan):
            if week.is_stepback and i > 0:
                prev = plan[i - 1]
                prev_long = next(s for s in prev.sessions if s.session_type == "long")
                this_long = next(s for s in week.sessions if s.session_type == "long")
                assert this_long.duration_min <= prev_long.duration_min, (
                    f"Step-back week {week.week_number} should not be longer than week {prev.week_number}"
                )

    # --- session structure -------------------------------------------------

    def test_each_week_has_seven_sessions(self):
        plan = self._make_builder().build()
        for week in plan:
            assert len(week.sessions) == 7

    def test_session_days_are_one_through_seven(self):
        plan = self._make_builder().build()
        for week in plan:
            days = [s.day for s in week.sessions]
            assert sorted(days) == list(range(1, 8))

    def test_three_running_sessions_per_week(self):
        plan = self._make_builder().build()
        for week in plan:
            running = [s for s in week.sessions if s.session_type in ("short", "long")]
            assert len(running) == 3, (
                f"Week {week.week_number} has {len(running)} running sessions, expected 3"
            )

    def test_exactly_one_long_run_per_week(self):
        plan = self._make_builder().build()
        for week in plan:
            long_runs = [s for s in week.sessions if s.session_type == "long"]
            assert len(long_runs) == 1

    # --- progression -------------------------------------------------------

    def test_long_run_grows_over_time(self):
        """Overall trend: last long run should be longer than first."""
        plan = self._make_builder().build()
        first_long = next(s for s in plan[0].sessions if s.session_type == "long")
        last_long  = next(s for s in plan[-1].sessions if s.session_type == "long")
        assert last_long.duration_min > first_long.duration_min

    def test_weight_decreases_each_week(self):
        plan = self._make_builder().build()
        weights = [w.current_weight_kg for w in plan]
        for a, b in zip(weights, weights[1:]):
            assert b <= a + 0.01  # allow float rounding

    def test_final_weight_reaches_target(self):
        plan = self._make_builder().build()
        assert abs(plan[-1].current_weight_kg - 70.0) < 0.5

    # --- elite vs non-elite background ------------------------------------

    def test_elite_background_faster_initial_pace(self):
        elite_builder   = self._make_builder(elite_background=True)
        general_builder = self._make_builder(elite_background=False)
        # Week 1 pace estimate is embedded in the run/walk ratio labels
        elite_plan   = elite_builder.build()
        general_plan = general_builder.build()
        # Elite runner should have shorter or equal walk breaks (faster pace tier)
        elite_walk   = next(s.walk_seconds for s in elite_plan[0].sessions if s.session_type == "short")
        general_walk = next(s.walk_seconds for s in general_plan[0].sessions if s.session_type == "short")
        assert elite_walk <= general_walk

    # --- run/walk interval sanity -----------------------------------------

    def test_run_and_walk_seconds_positive_for_running_sessions(self):
        plan = self._make_builder().build()
        for week in plan:
            for s in week.sessions:
                if s.session_type in ("short", "long"):
                    assert s.run_seconds > 0
                    assert s.walk_seconds > 0

    def test_rest_sessions_have_zero_intervals(self):
        plan = self._make_builder().build()
        for week in plan:
            for s in week.sessions:
                if s.session_type == "rest":
                    assert s.run_seconds == 0
                    assert s.walk_seconds == 0

    # --- distance estimate -------------------------------------------------

    def test_estimated_distance_positive(self):
        plan = self._make_builder().build()
        for week in plan:
            for s in week.sessions:
                if s.session_type in ("short", "long"):
                    assert s.est_distance_km > 0.0

    # --- phase labels ------------------------------------------------------

    def test_phase_labels_non_empty(self):
        plan = self._make_builder().build()
        for week in plan:
            assert week.phase.strip() != ""
            assert week.phase_description.strip() != ""
