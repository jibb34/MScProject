import os
import json
import requests
from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse
import polyline
from urllib.parse import quote

# Utility functions from project
from utils.gpx_utils import load_json
from utils.osrm_utils import compute_dynamic_radius, extract_chunk_idx, diagnose_points

OSRM_ROUTE_URL = "http://localhost:5000/route/v1/cycling"
OSRM_URL = "http://localhost:5000/match/v1/cycling"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in a GPX file"
    )
    parser.add_argument(
        "chunk_dir", help="Directory that contains all JSON chunks of the GPX file"
    )
    parser.add_argument(
        "dynamic_window", help="The length of the window used to calculate dynamic radius search")
    parser.add_argument(
        "--debug", action="store_true", help="Run in debug mode"
    )
    return parser.parse_args()


def format_list(values):
    """
    Convert a list of values to a semicolon-separated string.
    Example:
        [v1, v2] -> "v1;v2"
    """
    return ";".join(str(v) for v in values)


def route_fill_gaps(results_dir, output_dir, overview="full", geometries="geojson"):  # Unused stub
    """
    Scan through match_result_*.json in results_dir,
    and whenever a file has no matchings, look ahead to the next file
    with matchings and call the /route endpoint to fill the gap.
    Saves any "gap routes" as gap_<idx_prev>_<idx_next>.json in output_dir.

    Note: This function is currently a stub and not called anywhere.
    """
    # TODO: Implement gap filling logic
    pass


def get_osrm_match(file_path, radius_step=5, max_radius=100):
    """
    Send a match request to the OSRM server for a given JSON chunk.

    Args:
        file_path (str): Path to the JSON chunk file.
        radius_step (int): Step size for radius (unused; dynamic radius is used).
        max_radius (int): Maximum search radius (unused).

    Returns:
        tuple: (file_path, result_json)
    """

    # Load chunk data (coordinates, timestamps, radiuses, etc.)
    data = load_json(file_path)

    # Extract latitude/longitude pairs in (lat,lon) order
    coords = [(lat, lon) for lon, lat in data["coordinates"]]

    # Prepare output filename: chunk_xxxxxx.json -> result_xxxxxx.json
    base_name = os.path.basename(file_path)
    output_file = base_name.replace("chunk", "result")

    # Encode coordinate into polyline6 format to reduce URL length
    raw_poly = polyline.encode(coords, precision=6)
    poly = quote(raw_poly, safe='')
    match_url = f"{OSRM_URL}/polyline6({poly})"

    # Prepare match parameters
    params = {
        "geometries": data.get("geometries", "geojson"),
        "overview": data.get("overview", "full"),
        "steps": str(data.get("steps", False)).lower(),
        "gaps": "ignore",
        "tidy": "true",
    }
    # If timestamps are included in the dataset, add them here
    if "timestamps" in data:
        params["timestamps"] = format_list(data["timestamps"])

    # Computer dynamic radiuses based on noise factor
    # around a given window size
    radii = compute_dynamic_radius(coords,
                                   data["radiuses"][0],
                                   window=int(args.dynamic_window),
                                   noise_scale=2.0)
    params["radiuses"] = ";".join(str(radius)
                                  for radius in radii)
    try:
        # Send request to OSRM /match endpoint
        response = requests.get(match_url,
                                params=params,
                                timeout=30)
        response.raise_for_status()
        result = response.json()
        # If multiple matchings found (more than 1 continuous route),
        # diagnose poorly matched points
        if len(result.get("matchings", [])) > 1:
            diag = diagnose_points("http://localhost:5000", coords, radius=500)
            for lat, lon, dist, name in diag:
                if dist is None or dist > 100:
                    print(f"Point {(lat, lon)} snapped {
                          dist}m away - nearest road: {name}")

    except requests.RequestException as e:
        # If request fails, log a warning and continue
        # print(f"[WARN] OSRM request failed -> {e}")
        result = {}

    # Determine output directory based on map results.
    # If a chunk could not be processed,
    # an empty chunk is produced in results/gap
    # for later processing
    out_dir = "./data/results"
    gap_dir = os.path.join(out_dir, "gap")
    dest = (os.path.join(out_dir, output_file)
            if result.get("matchings")
            else os.path.join(gap_dir, output_file)
            )

    # Write result to file
    with open(dest, "w") as out:
        json.dump(result, out, indent=2)

    return (file_path, result)


def main(args):
    """
    Main entry point:
    - Read all chunk files from a directory
    - Send them to the get_osrm_match function in parallel
    - Collect and save results
    """
    CHUNK_DIR = args.chunk_dir
    # Prepare results directories
    results_dir = "./data/results"
    gap_dir = os.path.join(results_dir, "gap")
    os.makedirs(gap_dir, exist_ok=True)

    # Set max thread count to either the number of chunks to
    # process, or 32, whichever is less
    MAX_THREADS = min(32, sum(1 for name in os.listdir(CHUNK_DIR)))
    os.makedirs("./data/results/gap", exist_ok=True)

    # =========== pull all chunk files from directory sorted ===========
    json_files = sorted([
        os.path.join(CHUNK_DIR, f) for f in os.listdir(CHUNK_DIR)
        if f.endswith(".json")
    ])
    # ============== Multithreaded requests =====================
    results = []
    # Send request on each thread to server running OSRM software
    with ThreadPoolExecutor(max_workers=MAX_THREADS) as executor:
        future_to_file = {executor.submit(
            get_osrm_match, f): f for f in json_files}
    for future in as_completed(future_to_file):
        try:
            file, result = future.result()
            results.append((file, result))
        except Exception as e:
            print(f"[ERROR] Thread Failed: {e}")

    # ================= Merge into larger route ==============

    # TODO: Implement merge function


if __name__ == "__main__":
    args = parse_args()
    main(args)
