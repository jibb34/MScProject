import argparse
import json
import math
from datetime import datetime

# Earth radius in meters
EARTH_R = 6371000


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compute speed, gradient, and heading for each track point in a JSON file"
    )
    parser.add_argument("input_json", help="Path to input JSON with track-point data")
    parser.add_argument("output_json", help="Path to write JSON with added metrics")
    return parser.parse_args()


def load_points(path):
    with open(path, "r") as f:
        return json.load(f)


def save_points(points, path):
    with open(path, "w") as f:
        json.dump(points, f, separators=(",", ":"), ensure_ascii=False)


def parse_time(timestr):
    # Remove Z
    if timestr.endswith("Z"):
        timestr = timestr[:-1]
    # Strip timezone offset if present (e.g., +01:00)
    if "+" in timestr:
        timestr = timestr.split("+")[0]
    if "-" in timestr[10:]:  # after date part
        # find timezone minus
        parts = timestr.split("-")
        # recombine date and time parts
        timestr = parts[0] + "-" + parts[1].split("T")[1]
    try:
        # Use fromisoformat for flexible parsing
        return datetime.fromisoformat(timestr)
    except Exception:
        # Fallback to manual formats
        for fmt in ("%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S"):
            try:
                return datetime.strptime(timestr, fmt)
            except ValueError:
                continue
        raise ValueError(f"Unrecognized time format: {timestr}")


def haversine(lat1, lon1, lat2, lon2):
    # coords in decimal degrees
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    )
    return 2 * EARTH_R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# Calculate the degrees from true north of a point given the previous point.


def bearing(lat1, lon1, lat2, lon2):
    # initial bearing from point1 to point2 in degrees from north
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(
        dlambda
    )
    theta = math.atan2(x, y)
    deg = math.degrees(theta)
    # Normalize to 0-360
    return (deg + 360) % 360


def compute_metrics(points):
    # Adds speed and heading to each track point.
    result = []
    prev_pt = None
    prev_time = None

    for pt in points:
        # Copy original data
        new_pt = pt.copy()
        # Initialize metrics
        new_pt["speed"] = None
        new_pt["heading"] = None

        lat = new_pt.get("lat")
        lon = new_pt.get("long")
        time_str = new_pt.get("time")

        if prev_pt is not None:
            # time delta
            try:
                t1 = parse_time(prev_time)
                t2 = parse_time(new_pt["time"])
                dt = (t2 - t1).total_seconds()
            except Exception:
                dt = None
            # distance
            dist = None
            try:
                dist = haversine(
                    prev_pt["lat"], prev_pt["long"], new_pt["lat"], new_pt["long"]
                )
            except Exception:
                dist = None
            # speed (km/hr)
            if dist is not None and dt is not None and dt > 0:
                # Convert from m/s to Km/hr
                new_pt["speed"] = (dist / dt) * 3.6

            # heading
            if dist is not None:
                new_pt["heading"] = bearing(prev_pt["lat"], prev_pt["long"], lat, lon)

        result.append(new_pt)
        prev_pt = new_pt
        prev_time = time_str

    return result


def main():
    args = parse_args()
    points = load_points(args.input_json)
    enriched = compute_metrics(points)
    save_points(enriched, args.output_json)


if __name__ == "__main__":
    main()
