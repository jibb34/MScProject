#!/usr/bin/env python3
"""
Render an OSRM /match JSON (optionally *modified* with GPX data under each tracepoint)
into an interactive Folium map. The route is drawn as one continuous line, but
is split into colored segments per tracepoint/leg boundary (color cycles at each boundary). Colors cycle through a **small fixed palette** (default 5) to keep segments visually distinct. Each colored segment shows a hover tooltip with the GPX values from the *starting* tracepoint of that leg. If your tracepoint objects contain a `gpx_list` (a list of GPX points up to the next tracepoint), the tooltip will summarize those entries.

Usage
-----
python folium_route_by_tracepoint.py INPUT.json OUTPUT.html [--title "My Map"] [--num-colors 5] [--geometry-source osm|gpx]

Assumptions
-----------
- The JSON is an OSRM /match response *or* a structurally similar, modified
  response which includes:
    * "tracepoints": list with potentially non-null objects that may include a
      "gpx" dict of values to show in tooltips.
    * "matchings": list; we render legs from all matchings in order.
    * Each leg provides or can be resolved to geometry. Supported sources:
        1) leg["geometry"]["coordinates"] (GeoJSON)
        2) leg["geometry"] as an encoded polyline string (precision 5 or 6)
        3) leg["steps"][*]["geometry"]["coordinates"] or encoded strings
        4) Fallback: slice from the matching's overall geometry by indices if
           present in leg as {start_index,end_index} or similar.
        5) Last resort: straight line between consecutive matched tracepoints.

- Coordinates coming from OSRM are [lon, lat]. Folium expects [lat, lon].

Install
-------
pip install folium polyline
(If "polyline" isn't installed, a minimal decoder is used instead.)
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import folium

# Try to import the polyline package; if unavailable, fall back to a small
# internal decoder that supports polyline 5/6 precision (sufficient for OSRM).
try:
    import polyline as _polyline
except Exception:  # pragma: no cover
    _polyline = None


def decode_polyline(s: str, precision: int = 5) -> List[Tuple[float, float]]:
    """Decode a Google/OSRM polyline into a list of (lat, lon) tuples.

    If the external `polyline` package is available, delegate to it; otherwise
    use a minimal local decoder. The return order is (lat, lon), matching that
    library; we'll swap later when needed.
    """
    if _polyline is not None:
        return _polyline.decode(s, precision=precision)

    # Minimal local decoder adapted for precision 5/6.
    coords: List[Tuple[float, float]] = []
    index = lat = lng = 0
    factor = 10 ** precision

    while index < len(s):
        shift = result = 0
        while True:
            b = ord(s[index]) - 63
            index += 1
            result |= (b & 0x1F) << shift
            shift += 5
            if b < 0x20:
                break
        dlat = ~(result >> 1) if result & 1 else (result >> 1)
        lat += dlat

        shift = result = 0
        while True:
            b = ord(s[index]) - 63
            index += 1
            result |= (b & 0x1F) << shift
            shift += 5
            if b < 0x20:
                break
        dlng = ~(result >> 1) if result & 1 else (result >> 1)
        lng += dlng

        coords.append((lat / factor, lng / factor))

    return coords


# ---------------------------- Utilities ------------------------------------

def lonlat_to_latlon(coords: Iterable[Sequence[float]]) -> List[Tuple[float, float]]:
    """Convert [[lon, lat], ...] into [(lat, lon), ...] for Folium."""
    out: List[Tuple[float, float]] = []
    for c in coords:
        if len(c) < 2:
            continue
        lon, lat = float(c[0]), float(c[1])
        out.append((lat, lon))
    return out


def hsv_to_hex(h: float, s: float = 0.85, v: float = 0.95) -> str:
    """Convert HSV (h∈[0,1)) to hex color string."""
    i = int(h * 6)
    f = h * 6 - i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    i %= 6
    r, g, b = (
        (v, t, p) if i == 0 else
        (q, v, p) if i == 1 else
        (p, v, t) if i == 2 else
        (p, q, v) if i == 3 else
        (t, p, v) if i == 4 else
        (v, p, q)
    )
    return '#%02x%02x%02x' % (int(r * 255), int(g * 255), int(b * 255))


def distinct_colors(n: int) -> List[str]:
    """Generate n visually distinct hex colors by stepping hue."""
    if n <= 0:
        return []
    return [hsv_to_hex(i / float(n)) for i in range(n)]


def cycling_palette(num: int) -> List[str]:
    """Return a short, high-contrast palette to cycle through.

    Defaults to 5 bold colors; if a larger number is requested, extends with
    additional distinct colors while keeping the first 5 unchanged.
    """
    base = [
        "#1f77b4",  # blue
        "#ff7f0e",  # orange
        "#2ca02c",  # green
        "#d62728",  # red
        "#9467bd",  # purple
    ]
    extend = [
        "#8c564b",  # brown
        "#e377c2",  # pink
        "#7f7f7f",  # gray
        "#bcbd22",  # olive
        "#17becf",  # cyan
    ]
    palette = base + extend
    if num <= len(palette):
        return palette[:num]
    # If caller asks for more than provided, fill with HSV as a fallback
    return palette + distinct_colors(num - len(palette))


def safe_get(d: Dict[str, Any], *keys: str, default=None):
    cur: Any = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


# -------------------------- Geometry extraction ----------------------------

def extract_leg_coords(
    leg: Dict[str, Any],
    matching: Dict[str, Any],
    tracepoints: List[Optional[Dict[str, Any]]],
    leg_idx_global: int,
    precision_hint: Optional[int] = None,
) -> Optional[List[Tuple[float, float]]]:
    """Return [(lat, lon), ...] for this leg, trying multiple sources.

    Priority:
    1) leg.geometry.coordinates (GeoJSON)
    2) leg.geometry (encoded polyline string)
    3) steps[*].geometry (coords or encoded string)
    4) slice from matching geometry if indices present
    5) straight line between consecutive matched tracepoints (fallback)
    """
    # 1) GeoJSON coords on the leg
    g = leg.get("geometry")
    if isinstance(g, dict) and isinstance(g.get("coordinates"), list):
        return lonlat_to_latlon(g["coordinates"])  # type: ignore[arg-type]

    # 2) Encoded string on the leg
    if isinstance(g, str):
        # Try precision 6 first if hinted, else 5 then 6.
        precisions = [precision_hint] if precision_hint in (5, 6) else [5, 6]
        for p in precisions:
            try:
                coords_ll = decode_polyline(g, precision=p)
                # decode_polyline returns (lat, lon) already
                return list(coords_ll)
            except Exception:
                pass

    # 3) Steps geometries
    steps = leg.get("steps")
    if isinstance(steps, list) and steps:
        seg: List[Tuple[float, float]] = []
        for st in steps:
            sg = st.get("geometry")
            if isinstance(sg, dict) and isinstance(sg.get("coordinates"), list):
                seg.extend(lonlat_to_latlon(sg["coordinates"]))
            elif isinstance(sg, str):
                # try both precisions
                for p in (5, 6):
                    try:
                        seg.extend(decode_polyline(sg, precision=p))
                        break
                    except Exception:
                        continue
        if seg:
            return seg

    # 4) Slice from matching geometry by indices if present
    for start_key, end_key in (
        ("start_index", "end_index"),
        ("from_index", "to_index"),
        ("geometry_start_index", "geometry_end_index"),
        ("s_idx", "e_idx"),
    ):
        if start_key in leg and end_key in leg:
            try:
                sidx = int(leg[start_key])
                eidx = int(leg[end_key])
                mgeo = matching.get("geometry")
                if isinstance(mgeo, dict) and isinstance(mgeo.get("coordinates"), list):
                    slice_coords = mgeo["coordinates"][sidx:eidx + 1]
                    return lonlat_to_latlon(slice_coords)
                if isinstance(mgeo, str):
                    # Try to decode entire matching then slice.
                    for p in (5, 6):
                        try:
                            mcoords = decode_polyline(mgeo, precision=p)
                            return mcoords[sidx:eidx + 1]
                        except Exception:
                            continue
            except Exception:
                pass

    # 5) Fallback: straight line between consecutive matched tracepoints
    # Map leg to its starting tracepoint: by convention, leg i starts at
    # the i-th *non-null* tracepoint among all tracepoints.
    start_tp_idx = nth_non_null_index(tracepoints, leg_idx_global)
    if start_tp_idx is not None and start_tp_idx + 1 < len(tracepoints):
        tp_a = tracepoints[start_tp_idx]
        # find the next non-null
        next_tp_idx = next_non_null_index(tracepoints, start_tp_idx + 1)
        if next_tp_idx is not None:
            tp_b = tracepoints[next_tp_idx]
            if tp_a and tp_b and "location" in tp_a and "location" in tp_b:
                a_lon, a_lat = tp_a["location"][:2]
                b_lon, b_lat = tp_b["location"][:2]
                return [(a_lat, a_lon), (b_lat, b_lon)]

    # 6) Extra fallback: build from `gpx_list` if present on the starting tracepoint
    start_tp_idx = nth_non_null_index(tracepoints, leg_idx_global)
    if start_tp_idx is not None:
        tp = tracepoints[start_tp_idx]
        if isinstance(tp, dict) and isinstance(tp.get("gpx_list"), list):
            coords: List[Tuple[float, float]] = []
            for pt in tp["gpx_list"]:
                if isinstance(pt, dict):
                    lat = pt.get("lat") or pt.get("latitude") or pt.get("Lat")
                    lon = pt.get("lon") or pt.get("lng") or pt.get(
                        "longitude") or pt.get("Lon")
                    try:
                        if lat is not None and lon is not None:
                            coords.append((float(lat), float(lon)))
                    except Exception:
                        continue
                elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
                    try:
                        coords.append((float(pt[0]), float(pt[1])))
                    except Exception:
                        continue
            if coords:
                return coords

    return None


def nth_non_null_index(items: Sequence[Optional[Dict[str, Any]]], n: int) -> Optional[int]:
    """Return the absolute index of the n-th non-null element (0-based).
    If not found, return None.
    """
    count = 0
    for i, x in enumerate(items):
        if x is not None:
            if count == n:
                return i
            count += 1
    return None


def next_non_null_index(items: Sequence[Optional[Dict[str, Any]]], start: int) -> Optional[int]:
    for i in range(start, len(items)):
        if items[i] is not None:
            return i
    return None


# ----------------------------- Tooltips -------------------------------------

def gpx_tooltip_for_tracepoint(tp: Optional[Dict[str, Any]], idx: int) -> str:
    """Build a tooltip from `tp["gpx_list"]` if present; otherwise fall back to `tp["gpx"]`.

    The tooltip is HTML with line breaks. For very large lists, the content is
    truncated for hover usability.
    """
    if not tp:
        return f"Leg {idx}: (no GPX data)"

    # Prefer a list of GPX points if present
    if isinstance(tp.get("gpx_list"), list):
        rows: List[str] = []
        for i, point in enumerate(tp["gpx_list"]):
            try:
                # Compact JSON per point
                pjson = json.dumps(point, ensure_ascii=False,
                                   separators=(",", ":"))
            except Exception:
                pjson = str(point)
            rows.append(f"{i}: {pjson}")
        html = "<br/>".join(rows)
        # Trim excessively long tooltips (~10k chars)
        if len(html) > 10000:
            html = html[:10000] + "<br/>(…truncated)"
        return html or f"Leg {idx}: (empty gpx_list)"

    # Back-compat: single `gpx` dict
    if isinstance(tp.get("gpx"), dict):
        gpx = tp["gpx"]
        lines = []
        for k, v in gpx.items():
            if isinstance(v, (dict, list)):
                try:
                    v_str = json.dumps(v, ensure_ascii=False,
                                       separators=(",", ":"))
                except Exception:
                    v_str = str(v)
            else:
                v_str = str(v)
            lines.append(f"{k}: {v_str}")
        return "<br/>".join(lines) or f"Leg {idx}: (no GPX data)"

    return f"Leg {idx}: (no GPX data)"


# ----------------------------- Per-tracepoint geometry ----------------------

def _coords_from_gpx_list(gpx_list: List[Any]) -> List[Tuple[float, float]]:
    """Parse a list of GPX point structures into [(lat, lon), ...].
    Accepts dicts with common lat/lon key variants or [lat, lon] pairs.
    """
    coords: List[Tuple[float, float]] = []
    for pt in gpx_list:
        if isinstance(pt, dict):
            lat = pt.get("lat") or pt.get("latitude") or pt.get(
                "Lat") or pt.get("Latitude")
            lon = pt.get("lon") or pt.get("lng") or pt.get(
                "longitude") or pt.get("Lon") or pt.get("Longitude")
            try:
                if lat is not None and lon is not None:
                    coords.append((float(lat), float(lon)))
            except Exception:
                continue
        elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
            try:
                coords.append((float(pt[0]), float(pt[1])))
            except Exception:
                continue
    return coords


def extract_tp_segment_coords(
    tracepoints: List[Optional[Dict[str, Any]]],
    abs_idx: int,
) -> Optional[List[Tuple[float, float]]]:
    """Return the [(lat, lon), ...] path for a segment starting at tracepoint abs_idx.

    Priority:
      1) Use the starting tracepoint's `gpx_list` if present.
      2) Fallback: straight line from this tracepoint's `location` to the next
         non-null tracepoint's `location`.
    """
    if abs_idx < 0 or abs_idx >= len(tracepoints):
        return None

    tp = tracepoints[abs_idx]
    if not isinstance(tp, dict):
        return None

    if isinstance(tp.get("gpx_list"), list) and tp["gpx_list"]:
        coords = _coords_from_gpx_list(tp["gpx_list"])
        if coords:
            return coords

    # Fallback to straight line between locations
    next_idx = next_non_null_index(tracepoints, abs_idx + 1)
    if "location" in tp and next_idx is not None and tracepoints[next_idx] and "location" in tracepoints[next_idx]:
        a_lon, a_lat = tp["location"][:2]
        # type: ignore[index]
        b_lon, b_lat = tracepoints[next_idx]["location"][:2]
        return [(a_lat, a_lon), (b_lat, b_lon)]

    return None

# ------------------------------- Main ---------------------------------------


def build_map(data: Dict[str, Any], title: str = "Route", num_colors: int = 5, geometry_source: str = "osm") -> folium.Map:
    tracepoints: List[Optional[Dict[str, Any]]] = data.get(
        "tracepoints") or data.get("waypoints") or []
    matchings: List[Dict[str, Any]] = data.get(
        "matchings") or data.get("routes") or []

    m = folium.Map(location=[0, 0], zoom_start=13, control_scale=True)

    bounds: List[Tuple[float, float]] = []

    if geometry_source.lower() == "osm" and matchings:
        # Draw OSRM route geometry, segmented per leg (which aligns with tracepoint boundaries)
        colors = cycling_palette(max(1, num_colors))
        leg_counter = 0
        # Heuristic precision hint from first matching if needed
        precision_hint = None
        for matching in matchings:
            if precision_hint is None and isinstance(matching.get("geometry"), str):
                try:
                    _ = decode_polyline(matching["geometry"], precision=5)
                    precision_hint = 5
                except Exception:
                    precision_hint = 6

        for matching in matchings:
            legs = matching.get("legs", [])
            for leg in legs:
                coords = extract_leg_coords(
                    leg, matching, tracepoints, leg_counter, precision_hint=precision_hint)
                if not coords:
                    leg_counter += 1
                    continue
                bounds.extend(coords)
                # Tooltip from the starting non-null tracepoint
                tp_abs_idx = nth_non_null_index(tracepoints, leg_counter)
                tp = tracepoints[tp_abs_idx] if tp_abs_idx is not None else None
                tooltip_html = gpx_tooltip_for_tracepoint(tp, leg_counter)
                color = colors[leg_counter % len(colors)]
                folium.PolyLine(
                    locations=coords,
                    weight=5,
                    opacity=0.9,
                    color=color,
                    tooltip=folium.Tooltip(tooltip_html, sticky=True),
                ).add_to(m)
                leg_counter += 1
    else:
        # Fallback: build segments from GPX lists per tracepoint (previous behavior)
        indices: List[int] = [i for i, tp in enumerate(
            tracepoints) if tp is not None]
        colors = cycling_palette(max(1, num_colors))
        for seg_idx, abs_idx in enumerate(indices):
            tp = tracepoints[abs_idx]
            coords = extract_tp_segment_coords(tracepoints, abs_idx)
            if not coords:
                continue
            bounds.extend(coords)
            tooltip_html = gpx_tooltip_for_tracepoint(tp, seg_idx)
            color = colors[seg_idx % len(colors)]
            folium.PolyLine(
                locations=coords,
                weight=5,
                opacity=0.9,
                color=color,
                tooltip=folium.Tooltip(tooltip_html, sticky=True),
            ).add_to(m)

    # Fit map to bounds if available
    if bounds:
        m.fit_bounds(bounds)

    # Add an optional title as a floating Div
    if title:
        title_html = f"""
        <div style="position: fixed; top: 10px; left: 50%; transform: translateX(-50%);
                    background: rgba(255,255,255,0.9); padding: 6px 10px; border-radius: 8px;
                    box-shadow: 0 1px 4px rgba(0,0,0,0.2); font-weight: 600; z-index: 9999;">
            {title}
        </div>
        """
        m.get_root().html.add_child(folium.Element(title_html))

    return m


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Render OSRM match JSON to Folium HTML with per-tracepoint color cycling.")
    ap.add_argument(
        "input_json", help="Path to the modified OSRM /match JSON file")
    ap.add_argument("output_html", help="Path to write the output HTML map")
    ap.add_argument("--title", default="Route",
                    help="Optional title to show on the map")
    ap.add_argument("--num-colors", type=int, default=5,
                    help="Number of cycling colors to use (default: 5)")
    ap.add_argument("--geometry-source", choices=["osm", "gpx"], default="osm",
                    help="Use OSRM geometry ('osm') or build from GPX lists ('gpx'). Default: osm")
    args = ap.parse_args()

    with open(args.input_json, "r", encoding="utf-8") as f:
        data = json.load(f)

    fmap = build_map(data, title=args.title, num_colors=args.num_colors,
                     geometry_source=args.geometry_source)
    fmap.save(args.output_html)
    print(f"Wrote {args.output_html}")


if __name__ == "__main__":
    main()
