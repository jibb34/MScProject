import os
import json
import requests
from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse
import polyline
from utils.gpx_utils import load_json
from utils.osrm_utils import compute_dynamic_radius, extract_chunk_idx, diagnose_points
from urllib.parse import quote

OSRM_ROUTE_URL = "http://localhost:5000/route/v1/cycling"
OSRM_URL = "http://localhost:5000/match/v1/cycling"
HEADERS = {"Content-Type": "application/json"}


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
    """Convert list of values to string in format (value1,value2;value1,value2;...)"""
    return ";".join(str(v) for v in values)


def route_fill_gaps(results_dir, output_dir, overview="full", geometries="geojson"):
    """
    Scan through match_result_*.json in results_dir, and whenever you find
    a file with no matchings, look ahead to the next file with matchings
    and call /route between them.  Saves any "gap routes" as
    gap_<idx_prev>_<idx_next>.json in output_dir.
    """
    # gather and sort result files
    files = sorted(f for f in os.listdir(results_dir)
                   if f.startswith("match_result_") and f.endswith(".json"))
    # helper to load snapped locations


def get_osrm_match(file_path, radius_step=5, max_radius=100):
    data = load_json(file_path)
    coords = [(lat, lon) for lon, lat in data["coordinates"]]
    output_file = os.path.basename(file_path).replace("chunk", "result")

    # Convert to polyline to save space
    raw_poly = polyline.encode(coords, precision=6)

    # Percent-quote encode to stop issues
    poly = quote(raw_poly, safe='')

    match_url = f"{OSRM_URL}/polyline6({poly})"

    params = {
        "geometries": data.get("geometries", "geojson"),
        "overview": data.get("overview", "full"),
        "steps": str(data.get("steps", False)).lower(),
        "gaps": "ignore",
        "tidy": "true",
    }
    if "timestamps" in data:
        params["timestamps"] = format_list(data["timestamps"])

    radii = compute_dynamic_radius(coords,
                                   data["radiuses"][0],
                                   window=int(args.dynamic_window),
                                   noise_scale=2.0)
    # Increase radius until response returns a single segment
    params["radiuses"] = ";".join(str(radius)
                                  for radius in radii)
    try:
        response = requests.get(match_url,
                                params=params,
                                timeout=30)
        response.raise_for_status()
        result = response.json()
        if len(result.get("matchings", [])) > 1:
            tracepoints = result.get("tracepoints", [])
            dropped = sum(1 for tp in tracepoints if tp is None)
            total = len(tracepoints)
            diag = diagnose_points("http://localhost:5000", coords, radius=500)
            for lat, lon, dist, name in diag:
                if dist is None or dist > 100:
                    print(f"Point {(lat, lon)} snapped {
                          dist}m away - nearest road: {name}")
        # print(f"Dropped {dropped} out of {total} points")

    except requests.RequestException as e:
        # print(f"[WARN] OSRM request failed -> {e}")
        result = {}

    # Change filename from match_chunk_xxx.json to match_result_xxx.json
    out_dir = "./data/results"
    gap_dir = os.path.join(out_dir, "gap")
    dest = os.path.join(out_dir, output_file) if result.get("matchings") else \
        os.path.join(gap_dir, output_file)

    with open(dest, "w") as out:
        json.dump(result, out, indent=2)

    return (file_path, result)


def main(args):
    # Define Chunk file location
    CHUNK_DIR = args.chunk_dir
    # print(os.listdir(CHUNK_DIR))
    # Set max thread count to either the number of chunks to
    # process, or 32, whichever is less
    MAX_THREADS = min(32, sum(1 for name in os.listdir(CHUNK_DIR)))
    os.makedirs("./data/results/gap", exist_ok=True)

    # =========== pull all chunk files from directory sorted ===========
    json_files = sorted([
        os.path.join(CHUNK_DIR, f) for f in os.listdir(CHUNK_DIR)
        if f.endswith(".json")
    ])
    # print(json_files)

    # print(f"[DEBUG] Found {len(json_files)} JSON files")

    results = []
    # ============== Multithreaded requests =====================

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

    # print("[PYTHON] Merged all chunk matches into one")

    results_dir = "./data/results"
    os.makedirs(results_dir, exist_ok=True)


if __name__ == "__main__":
    args = parse_args()
    main(args)
