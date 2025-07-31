import argparse
import os
import json
import numpy as np

EARTH_R = 6371000  # Radius of earth


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test outlier detection without interpolation"
    )
    parser.add_argument(
        "input_json", help="Path to the JSON file with track-point data"
    )
    parser.add_argument(
        "metric", help="Metric to test (e.g., 'geographic', 'power', 'hr', etc.)"
    )
    parser.add_argument(
        "threshold",
        type=float,
        nargs="?",
        default=2.0,
        help="SD-multiplier threshold (default: 2.0)",
    )
    return parser.parse_args()


# def parse_args():
#     parser = argparse.ArgumentParser(
#         description="Interactive normalizer for track-point JSON data, including geographic outlier detection"
#     )
#     parser.add_argument(
#         "input_json", help="Path to the normalized JSON file from GPX-to-JSON step"
#     )
#     parser.add_argument(
#         "output_json", help="Path to write the cleaned and interpolated JSON"
#     )
#     return parser.parse_args()


def load_points(path):
    with open(path, "r") as f:
        return json.load(f)


def list_metrics(points):
    # For a specific json file, list all the metrics for each trackpoint, and give user the option to either normalize or not normalize them

    # Define a set of all the keys in a trackpoint
    all_keys = set().union(*[set(p.keys()) for p in points])
    # keys to exclude
    exclude = {"time"}

    # Return a list of all keys that can be normalized
    custom = sorted(all_keys - exclude)
    return ["geographic"] + [k for k in custom if k not in ("lat", "long")]


def metric_prompt(metrics):
    # Print out the menu
    print("Available Normalizable Metrics:")
    # Iterate through and add index
    for idx, m in enumerate(metrics, start=1):
        print(f"  {idx}. {m}")
    choice = input("Enter numbers to normalize (space-separated):")
    select = []
    # Get array of normalizable metrics
    for part in choice.split(" "):
        # If the part is a number, get the number and select the metric that corresponds
        if part.strip().isdigit():
            i = int(part)
            if 1 <= i <= len(metrics):
                select.append(metrics[i - 1])
    return select


def prompt_threshold():
    # Print out the prompt to enter the standard deviation multiplier
    val = input("Enter SD-multiplier threshold (e.g. 2.0): ")
    try:
        return float(val)
    except ValueError:
        print("Invalid input, using default 2.0")
        return 2.0


# Haverine formula for calculating distance
def haversine(lat1, lon1, lat2, lon2):
    # convert to radians
    lat1, lon1, lat2, lon2 = map(np.radians, (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2.0) ** 2 + np.cos(lat1) * \
        np.cos(lat2) * np.sin(dlon / 2.0) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return EARTH_R * c


def compute_geodistances(points):
    # Get all lat and long points
    lats = np.array([p["lat"] for p in points], dtype=float)
    longs = np.array([p["long"] for p in points], dtype=float)
    # Calculate distance pair-wise
    return haversine(lats[:-1], longs[:-1], lats[1:], longs[1:])


def compute_deltas(points, metric):
    # If latitude, longitude, compute the geodistance
    if metric == "geographic":
        return np.abs(compute_geodistances(points))
    # Else computer abs difference
    vals = [float(p.get(metric, np.nan)) for p in points]
    return np.abs(np.diff(vals))


def find_missing(points, metric, zero_threshold=0.05):
    # Identifies missing values in the signal
    vals = []
    for p in points:
        v = p.get(metric)
        try:
            fv = float(v)
        except (TypeError, ValueError):
            fv = np.nan
        vals.append(fv)
    arr = np.array(vals)
    # find ur nan
    nan_idx = np.where(np.isnan(arr))[0].tolist()
    # find zeroes
    zero_idx = np.where(arr == 0)[0].tolist()
    # Decide if zeroes count as missing:
    missing_idx = nan_idx.copy()
    if len(zero_idx) / max(len(arr), 1) < zero_threshold:
        # treat zeroes as missing:
        missing_idx = sorted(set(missing_idx) | set(zero_idx))
    missing_vals = [arr[i] for i in missing_idx]
    return missing_idx, missing_vals


# def interpolate(points, metric, out_idx):
# TODO: for each point given a specific metric, remove, and interpolate that metric
#       on the previous and next index, if the value falls inside out_idx


def main():
    args = parse_args()
    points = load_points(args.input_json)
    diffs = compute_deltas(points, args.metric)
    missing_idx, missing_vals = find_missing(points, args.metric)
    print(f"Missing for '{args.metric}': indices={
          missing_idx}, values={missing_vals}")

    # out_idx, out_values, mu, sigma = find_outliers(diffs, args.threshold)


if __name__ == "__main__":
    main()
