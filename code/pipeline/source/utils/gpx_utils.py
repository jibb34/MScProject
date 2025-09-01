import gpxpy
import osmnx as ox
import json
import os
from datetime import datetime, timezone
from utils.extension_utils import normalize_extensions_kv, load_extension_rules


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    from math import radians, sin, cos, asin, sqrt

    lat1, lon1, lat2, lon2 = map(radians, (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    return 2 * R * asin(sqrt(a))


def load_gpx(path):
    with open(path, "r") as f:
        return gpxpy.parse(f)


def bounding_box_from_data(data):
    # TODO: Possibly may need to expand the bbox slightly if error matching causes issues, route could go outside the box even if gpx points don't
    # Get all lat/long values
    lats = [p["lat"] for p in data if "lat" in p and p["lat"] is not None]
    longs = [p["lon"] for p in data if "lon" in p and p["lon"] is not None]
    # get the extrema
    north = max(lats)
    south = min(lats)
    east = max(longs)
    west = min(longs)

    # return a quaduple of the extrema in bbox format (left, bottom, right, top)
    return (south, west, north, east)


def load_json(file_path):
    with open(file_path, "r") as f:
        return json.load(f)


def write_json(points, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        # separators remove spaces, to get a smaller file size
        json.dump(points, f, separators=(",", ":"),
                  indent=2, ensure_ascii=False)


def bbox_contains(outer, inner):
    minlat_o, minlon_o, maxlat_o, maxlon_o = outer
    minlat_i, minlon_i, maxlat_i, maxlon_i = inner
    return (
        minlat_i >= minlat_o
        and minlon_i >= minlon_o
        and maxlat_i <= maxlat_o
        and maxlon_i <= maxlon_o
    )


def download_osm_file(bbox, output_path):
    import requests

    # Convert bbox to Overpass API bbox string:
    s, w, n, e = bbox
    print(f"Min Lat: {s}, max lat: {n}, min long: {w}, max long: {e}")
    bbox_str = f"{w},{s},{e},{n}"
    url = f"https://overpass-api.de/api/map?bbox={bbox_str}"

    def overpass_query(query_url):
        response = requests.get(query_url)
        with open(output_path, "wb") as f:
            f.write(response.content)

    overpass_query(url)

    # Overpass API request:
    # if os.path.exists(output_path):
    #     try:
    #         osm_bbox = read_osm_bounds(output_path)
    #         # if existing bbox contains this bbox, we don't need to redownload
    #         if bbox_contains(osm_bbox, bbox):
    #             print(f"[INFO] Existing {output_path} already covers GPX bbox {
    #                 bbox}, skipping download...")
    #             return
    #         else:
    #             print("[INFO] OSM bbox too small; redownloading...")
    #             overpass_query(url)
    #     except Exception as e:
    #         print(f"[WARNING] Couldn't parse existing OSM: {
    #               e} - redownloading...")
    #         overpass_query(url)
    # else:
    #     print("[INFO] No existing OSM file; downloading new extract.")
    #     overpass_query(url)


def parse_time_iso(s):
    # handles ...Z and offsets
    if s is None:
        return None
    try:
        if s.endswith("Z"):
            return int(datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp())
        return int(datetime.fromisoformat(s).timestamp())
    except Exception:
        return None


def iso_utc(ts: int) -> str:
    return (
        datetime.fromtimestamp(int(ts), tz=timezone.utc)
        .isoformat()
        .replace("+00:00", "Z")
    )


def to_unix(ts):
    if ts is None:
        return None
    if isinstance(ts, (int, float)):
        return int(ts)
    if isinstance(ts, str):
        # handle ...Z and offsets like +00:00
        try:
            return int(datetime.fromisoformat(ts.replace("Z", "+00:00")).timestamp())
        except Exception:
            return None
    return None


def extract_data_from_gpx(gpx):
    def _local(tag: str) -> str:
        return tag.split("}", 1)[-1] if "}" in tag else tag

    def _coerce(text):
        if text is None:
            return None
        s = text.strip()
        if s == "":
            return None
        try:
            return int(s)
        except ValueError:
            try:
                return float(s)
            except ValueError:
                return s

    # --- NEW: generic node accessors (gpxpy or ElementTree) ---
    def _node_tag(node):
        return getattr(node, "tag", None)

    def _node_text(node):
        return getattr(node, "text", None)

    def _node_children(node):
        # gpxpy GPXExtensions often use .extensions (not .children)
        kids = getattr(node, "children", None)
        if not kids:
            kids = getattr(node, "extensions", None)
        if kids:
            return list(kids)
        # ElementTree elements are iterable
        try:
            return list(node)
        except TypeError:
            return []

    def _element_to_dict(node):
        kids = _node_children(node)
        if not kids:
            return _coerce(_node_text(node))
        out = {}
        for ch in kids:
            key = _local(_node_tag(ch) or "")
            val = _element_to_dict(ch)
            if key in out:
                if not isinstance(out[key], list):
                    out[key] = [out[key]]
                out[key].append(val)
            else:
                out[key] = val
        return out

    def _iter_leaves(obj, path=()):
        if isinstance(obj, dict):
            for k, v in obj.items():
                yield from _iter_leaves(v, path + (str(k),))
        elif isinstance(obj, list):
            for i, v in enumerate(obj):
                yield from _iter_leaves(v, path + (str(i),))
        else:
            yield (path, obj)

    def _flatten_extensions_kv(ext_dict, joiner="."):
        items = []
        for path, val in _iter_leaves(ext_dict):
            if val is None:
                continue
            key = joiner.join(path).rsplit(joiner, 1)[-1]
            # print(key)
            items.append({"key": key, "value": val})
        # de-dup while preserving order
        seen = set()
        uniq = []
        for it in items:
            t = (it["key"], it["value"])
            if t in seen:
                continue
            seen.add(t)
            uniq.append(it)
        return uniq

    points = []
    rules = load_extension_rules("config/extensions_map.json")
    for track in gpx.tracks:
        for segment in track.segments:
            for pt in segment.points:
                point = {
                    "lat": pt.latitude,
                    "lon": pt.longitude,
                    "elv": pt.elevation if pt.elevation is not None else None,
                    "time": pt.time.isoformat() if pt.time else None,
                }

                # include extensions (now truly recursive for gpxpy)
                if pt.extensions:
                    ext_blob = {}
                    for ext in pt.extensions:
                        # <-- now sees .extensions
                        d = _element_to_dict(ext)
                        key = _local((_node_tag(ext) or ""))
                        if isinstance(d, dict):
                            if key in ext_blob:
                                prev = ext_blob[key]
                                ext_blob[key] = (
                                    [prev, d]
                                    if not isinstance(prev, list)
                                    else prev + [d]
                                )
                            else:
                                ext_blob[key] = d
                        else:
                            ext_blob[key] = d
                    # store as a FLAT list of {"key","value"} pairs
                    items = _flatten_extensions_kv(ext_blob, joiner=".")
                    canon = normalize_extensions_kv(items, rules)
                    point["extensions"] = [
                        {"key": k, "value": v} for k, v in canon.items()
                    ]

                points.append(point)
    return points
