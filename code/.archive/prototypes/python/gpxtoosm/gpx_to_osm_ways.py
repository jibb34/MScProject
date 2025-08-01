"""
Script: gpx_to_osm_ways.py

Reads a GPX file, fetches all highway ways within track bounding box via Overpass once,
matches each trackpoint to its nearest way in-memory, and outputs unique way IDs and tags.
"""

import sys
import requests
import gpxpy
from tqdm import tqdm
from scipy.spatial import cKDTree

OVERPASS_URL = "https://overpass-api.de/api/interpreter"


def load_track_points(gpx_path):
    with open(gpx_path, "r") as f:
        gpx = gpxpy.parse(f)
    pts = []
    for tr in gpx.tracks:
        for seg in tr.segments:
            for pt in seg.points:
                pts.append((pt.latitude, pt.longitude))
    return pts


def fetch_ways_in_bbox(min_lat, min_lon, max_lat, max_lon):
    # Correct Overpass syntax: highway as tag
    query = f"""
[out:json][timeout:60];
(
  way["highway"]({min_lat},{min_lon},{max_lat},{max_lon});
);
out body geom tags;
"""
    r = requests.post(OVERPASS_URL, data={"data": query})
    r.raise_for_status()
    elems = r.json().get("elements", [])
    ways = []
    for e in elems:
        if e.get("type") != "way":
            continue
        geom = e.get("geometry", [])
        if not geom:
            continue
        lat0 = sum(p["lat"] for p in geom) / len(geom)
        lon0 = sum(p["lon"] for p in geom) / len(geom)
        ways.append((e["id"], e.get("tags", {}), (lat0, lon0)))
    return ways


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} path/to/file.gpx")
        sys.exit(1)
    gpx_path = sys.argv[1]
    pts = load_track_points(gpx_path)
    lats = [p[0] for p in pts]
    lons = [p[1] for p in pts]
    margin = 0.0005
    min_lat, max_lat = min(lats) - margin, max(lats) + margin
    min_lon, max_lon = min(lons) - margin, max(lons) + margin

    print("Fetching ways in bounding box...")
    ways = fetch_ways_in_bbox(min_lat, min_lon, max_lat, max_lon)
    if not ways:
        print("No ways found in bounding box.")
        return
    ids, tags_list, centers = zip(*ways)
    tree = cKDTree(centers)

    unique = {}
    for lat, lon in tqdm(pts, desc="Matching points", unit="pt"):
        _, idx = tree.query((lat, lon))
        unique[ids[idx]] = tags_list[idx]

    print("\nFound ways:")
    for wid, tags in unique.items():
        print(f"Way ID {wid}, tags: {tags}")


if __name__ == "__main__":
    main()

# Requirements:
# pip install gpxpy requests tqdm scipy
