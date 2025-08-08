import os
import re
import json
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from pathlib import Path
from math import ceil
import osmnx as ox
import requests
import xml.etree.ElementTree as ET


"""Generates JSON object(s) from a list of GPX trackpoints, in the format:
{
    "lat": 51.501,
    "lon": -0.141,
    "time": "2025-08-02T12:34:56Z",
    ...
}
Args: gpx_points (list): List of gpx trackpoint objects
      output_dir (str): Directory to save the JSON files
      chunk_size (int): Number of GPX points per chunk. 0 = no chunking.
Returns:
      List of output JSON file paths
Notes:
      This is a required step outside of saving gpx data into json format
      /match OSRM API requires a specific json object format, meaning we must
      remove extensions and other metadata
    """


def points_to_osrm_json(gpx_points, output_dir, basename, radius=5, chunk_size=0):
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    # Create our payload from a list of points
    def create_payload(points, radius):
        coords = [
            [
                pt["lon"],
                pt["lat"],
            ]  # Counterintuitively, we must reverse the order for Google's polyline algorithm
            for pt in points
        ]  # Create coordinate pair from points
        timestamps = []
        radiuses = [int(radius) for pt in points]
        for pt in points:
            if "time" in pt and pt["time"]:
                ts = pt["time"]
                if isinstance(ts, str):
                    try:
                        dt = datetime.fromisoformat(ts.replace("Z", "+00:00"))
                        timestamps.append(int(dt.timestamp()))
                    except ValueError:
                        timestamps.append(None)
                else:
                    timestamps.append(int(ts.timestamp()))

            payload = {"coordinates": coords, "radiuses": radiuses}
            if any(t is None for t in timestamps):
                raise ValueError(
                    "Missing or invalid timestamps in GPX data. Attempting to interpolate..."
                )
                for i, t in enumerate(timestamps):
                    if t is None:
                        try:
                            t = timestamps[i - 1] + 1
                        except Exception:
                            t = timestamps[i + 1] - 1

            payload["timestamps"] = timestamps

        return payload

    # Create payload, and save to .json
    output_files = []
    if chunk_size <= 0:
        # no chunking
        payload = create_payload(gpx_points, radius)
        path = os.path.join(output_dir, "match.json")
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)
        output_files.append(path)
    else:
        # chunk_size elements per chunk
        for i in range(0, len(gpx_points), chunk_size):
            chunk = gpx_points[i: i + chunk_size]
            payload = create_payload(chunk, radius)
            chunk_idx = i // chunk_size
            # Setting the index to 000000...999999 instead to keep leading 0
            path = os.path.join(output_dir, f"{basename}_chunk_{
                                chunk_idx:06d}.json")
            with open(path, "w") as f:
                json.dump(payload, f, indent=2)
            output_files.append(path)
    return output_files


def haversine(a, b):
    from math import radians, cos, sin, asin, sqrt

    lat1, lon1 = a
    lat2, lon2 = b
    R = 6371000
    phi_1, phi_2 = radians(lat1), radians(lat2)
    delta_phi, delta_gamma = radians(lat2 - lat1), radians(lon2 - lon1)
    h = sin(delta_phi / 2) ** 2 + cos(phi_1) * \
        cos(phi_2) * sin(delta_gamma / 2) ** 2
    return 2 * R * asin(sqrt(h))


def compute_dynamic_radius(
    coords,
    initial_radius,
    max_radius=50,
    step=1,
    window=3,
    noise_scale=2.0,
    plot_name=None,
):
    """
    Takes a list of coordinates, and inital radius array, and returns a new list of radii that forms
    to the noise scale of the coordinates. Noise scale calculated as average delta over window
    """
    # Base case
    n = len(coords)
    if n < 2:
        return [initial_radius] * n

    # get bearing for each point
    headings = [
        ox.bearing.calculate_bearing(
            coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1]
        )
        for i in range(n - 1)
    ]
    # Computer heading change between 2 headings
    heading_delta = []
    for i in range(len(headings) - 1):
        dif = abs(headings[i + 1] - headings[i])
        # wrap at 180 (furthest distance)
        if dif > 180:
            dif = 360 - dif
        heading_delta.append(dif)
    radii = []
    # For each point, take the average heading delta over a sliding window centred on the point
    half = window // 2
    for i in range(n):
        low = max(0, i - 1 - half)
        # to make sure we don't go out of bounds
        high = min(len(heading_delta), i + half)
        # computer average if amount of low is less than high
        if low < high:
            # take average of window
            avg_delta = sum(heading_delta[low:high]) / (high - low)
        else:
            # fallback delta set to 0
            avg_delta = 0
        # multiply by noise scale
        extra = ceil(avg_delta * noise_scale)
        # add to initial radius, until max radius hit
        r = min(max_radius, initial_radius + extra)
        radii.append(r)

    if plot_name:
        fig, ax1 = plt.subplots(figsize=(10, 4))

        ax1.set_xlabel("Trackpoint Index")
        ax1.set_ylabel("Search Radius (m)", color="tab:blue")
        ax1.plot(radii, color="tab:blue", label="Radius")
        ax1.tick_params(axis="y", labelcolor="tab:blue")

        ax2 = ax1.twinx()
        ax2.set_ylabel("Heading (°)", color="tab:orange")
        ax2.plot(headings, color="tab:orange", label="Heading")
        ax2.tick_params(axis="y", labelcolor="tab:orange")

        plt.title("Dynamic Radius and Heading Over Time")
        fig.tight_layout()

        safe_name = plot_name.replace(" ", "_").replace(".gpx", "")
        plt.savefig(f"radius_heading_plot_{safe_name}.png", dpi=150)
        plt.close()

    return radii


def prune_spurs(match_coords, raw_coords, min_length=50, min_raw_hits=2, proximity=10):
    """Return True if the matched segment should be kept
    - min_length: the spur must be at least this long to exist
    - min_raw_hits: keep if at least this many points lie near it
    - proximity: distance in m to count a raw point as on the segment
    """
    seg_length = sum(
        haversine((lat1, lon1), (lat2, lon2))
        for (lon1, lat1), (lon2, lat2) in zip(match_coords, match_coords[1:])
    )
    if seg_length < min_length:
        return False

    hits = 0

    for lon, lat in raw_coords:
        pt = (lat, lon)
        dists = [haversine(pt, (c_lat, c_lon))
                 for c_lon, c_lat in match_coords]
        if min(dists) <= proximity:
            hits += 1
            if hits >= min_raw_hits:
                return True
    return False


def extract_chunk_idx(file_path: str) -> int:
    # Given a file name, ending in _<digits>.json, return the integer in <digits>
    filename = os.path.basename(file_path)
    m = re.search(r"_(\d+)\.json$", filename)
    if not m:
        raise ValueError(f"Couldn't parse index from {filename}")
    return int(m.group(1))


def diagnose_points(OSRM, coords, radius=100):
    """
    For each (lat,lon) in coords, hit /nearest and report the snapped distance.
    """
    results = []
    for lat, lon in coords:
        url = f"{OSRM}/nearest/v1/cycling/{lon},{lat}"
        params = {"number": 1, "radius": radius}
        r = requests.get(url, params=params, timeout=5)
        r.raise_for_status()
        wp = r.json()["waypoints"][0]
        # waypoint.distance is in meters
        results.append((lat, lon, wp["distance"], wp["name"]))
    return results


def read_osm_bounds(osm_path):
    """Parse the <bounds> element from an OSM XML file"""
    for even, elem in ET.iterparse(osm_path, events=("start,")):
        if elem.tag == "bounds":
            minlat = float(elem.attrib["minlat"])
            minlon = float(elem.attrib["minlon"])
            maxlat = float(elem.attrib["maxlat"])
            maxlon = float(elem.attrib["maxlon"])
            return minlat, minlon, maxlat, maxlon
        # when we see first element we are done
        break
    raise ValueError("No <bounds> tag found in OSM file")
