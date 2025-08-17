import argparse
from typing import List, Tuple

from utils.gpx_utils import (
    iso_utc,
    to_unix,
)


def parse_args():
    ap = argparse.ArgumentParser(
        description="Enrich OSRM /match JSON with GPX metadata by coordinate matching."
    )
    ap.add_argument("osrm_json", help="Path to OSRM match result JSON")
    ap.add_argument("gpx_file", help="Path to GPX file used to create the match")
    ap.add_argument(
        "-o",
        "--out",
        default=None,
        help="Output JSON path (default: <input>_enriched.json)",
    )
    ap.add_argument(
        "--tol", type=float, default=15.0, help="Match tolerance in meters (default 15)"
    )
    ap.add_argument(
        "--precision",
        type=int,
        default=5,
        help="Grid precision for coordinate bucketing (default 5 ~ 1e-5)",
    )
    ap.add_argument(
        "--no-leg-times", action="store_true", help="Do not add per-leg time windows"
    )
    ap.add_argument(
        "--node-times",
        action="store_false",
        help="Also add per-node interpolated timestamps",
    )
    return ap.parse_args()


def _adjacent_waypoint_pairs(tracepoints) -> List[Tuple[int, int, int]]:
    """
    Return list of (leg_idx, gpx_i, gpx_j) for adjacent matched waypoints.
    Assumes each non-null tp may already have tp["gpx"]["index"] set (ok if not).
    """
    matched = []
    for i, tp in enumerate(tracepoints):
        if not tp:
            continue
        m = tp.get("matchings_index")
        w = tp.get("waypoint_index")
        g = (tp.get("gpx") or {}).get("index")
        if m is None or w is None:
            continue
        matched.append((i, g, m, w))

    pairs = []
    for (i0, g0, m0, w0), (i1, g1, m1, w1) in zip(matched, matched[1:]):
        if m0 == m1 and w1 == w0 + 1:
            pairs.append((w0, g0, g1))  # leg index, gpxi, gpxj
    return pairs


def _interpolate_node_times(leg, t0: int, t1: int) -> List[int]:
    ann = leg.get("annotation") or {}
    nodes = ann.get("nodes") or []
    if len(nodes) <= 1:
        return [t0] * len(nodes)

    dist = ann.get("distance") or []
    total = float(sum((d or 0.0) for d in dist[: len(nodes) - 1]))
    if total <= 0:
        step = (t1 - t0) / max(1, len(nodes) - 1)
        return [int(t0 + i * step) for i in range(len(nodes))]

    cum = [0.0]
    s = 0.0
    for d in dist[: len(nodes) - 1]:
        s += float(d or 0.0)
        cum.append(s)
    return [int(t0 + (c / total) * (t1 - t0)) for c in cum]


def _write_gpx_list_to_tracepoints(tracepoints, gpx_points):
    """
    For each non-null tracepoint at index i, write tp['gpx_list'] with the GPX slice
    from i (inclusive) up to the next non-null tracepoint index (exclusive).
    The last non-null TP gets [i, len(gpx_points)).
    Assumes you called /match with one input per GPX point (same ordering).
    """
    n_tp = len(tracepoints)
    n_gpx = len(gpx_points)
    if n_tp != n_gpx:
        print(
            "[WARN] Length mismatch between Tracepoints and GPX trackpoints; falling back to min length."
        )
    N = min(n_tp, n_gpx)

    # collect indices of non-null tracepoints
    anchors = [i for i in range(N) if isinstance(tracepoints[i], dict)]

    for k, i in enumerate(anchors):
        tp = tracepoints[i]
        # find next non-null tracepoint index, else end
        end = anchors[k + 1] if (k + 1) < len(anchors) else N
        start = i
        if end < start:
            end = start  # safety

        # build the slice [start, end)
        gpx_list = []
        for idx in range(start, end):
            gp = gpx_points[idx]
            t_unix = to_unix(gp.get("time")) if gp.get("time") is not None else None
            gpx_list.append(
                {
                    "index": idx,
                    "lat": gp.get("lat"),
                    "lon": gp.get("lon"),
                    "time": t_unix,
                    "time_iso": (iso_utc(t_unix) if t_unix is not None else None),
                    "elv": gp.get("elv"),
                    "extensions": gp.get("extensions"),
                }
            )

        tp["gpx_list"] = gpx_list


def attach_to_tracepoints(
    osrm,
    gpx_points,
    tol_m=15.0,
    precision=5,
    add_leg_time_windows=True,
    interpolate_node_times=False,
):
    """
    Mutates OSRM JSON in place:
      - adds tracepoints[i]["gpx"] with matched GPX metadata
      - (optional) per-leg time_window, gpx_indices, node_times
    """
    tps = osrm.get("tracepoints") or []
    if not tps:
        return osrm

    # 1) Index based mapping:

    # 2) write compact GPX dicts on tracepoints
    _write_gpx_list_to_tracepoints(tps, gpx_points)
    osrm["tracepoints"] = tps

    # 3) per-leg windows (early exit keeps branching low)
    # if not add_leg_time_windows:
    #     return osrm
    #
    # matchings = osrm.get("matchings") or []
    # if not matchings:
    # return osrm
    # legs = (matchings[0].get("legs") or [])

    # 4) figure out which leg each adjacent matched waypoint pair belongs to
    # pairs = _adjacent_waypoint_pairs(tps)   # [(leg_idx, gpx_i, gpx_j), ...]

    # 5) enrich legs
    # _add_leg_times(legs, pairs, gpx_points,
    # interpolate_nodes=interpolate_node_times)

    return osrm
