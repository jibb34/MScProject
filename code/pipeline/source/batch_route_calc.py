import os
import json
import requests
from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse
import polyline
from urllib.parse import quote
import sqlite3
from itertools import groupby

# Utility functions from project
from utils.gpx_utils import load_json
from utils.osrm_utils import compute_dynamic_radius, extract_chunk_idx, diagnose_points
from utils.osm_utils import _flatten_nodes, _edge_lookup

OSRM_ROUTE_URL = "http://localhost:5000/route/v1/cycling"
OSRM_URL = "http://localhost:5000/match/v1/cycling"


def parse_args():
    parser = argparse.ArgumentParser(description="Pass in a GPX file")
    parser.add_argument(
        "input_dir", help="Directory that contains all JSON chunks of the GPX file"
    )
    parser.add_argument(
        "output_dir", help="Directory that the match file is sent to")
    parser.add_argument(
        "dynamic_window",
        help="The length of the window used to calculate dynamic radius search",
    )
    parser.add_argument("--waydb", default="way_index.sqlite",
                        help="Path to way_index.sqlite")
    parser.add_argument("--with-metrics", action="store_true",
                        help="Sum distance into way runs if present")

    return parser.parse_args()


def chunk_nodes_to_ways(osrm_obj, db="way_index.sqlite", with_metrics=False):
    # Load OSRM JSON and flatten nodes
    legs = osrm_obj.get("matchings", [{}])[0].get("legs", [])
    nodes = _flatten_nodes(legs)
    # Open edge index database
    con = sqlite3.connect(db)
    cur = con.cursor()

    # optional align distance per edge
    distances = []
    if with_metrics:
        for leg in osrm_obj["matchings"][0].get("legs", []):
            ann = leg.get("annotation", {})
            distances.extend(ann.get("distance") or [])

    # Each node pair is mapped to a directional way from the SQLite database

    edges = []  # list of (way_id, dir, dist)
    # way_id = numeric ID of OSM way
    # dir = 1, -1 or 0.
    # +1 means forward (u->v),
    # -1 means backwards (v->u),
    # and 0 means no way found
    for i, (u, v) in enumerate(zip(nodes[:-1], nodes[1:])):
        u = int(u)
        v = int(v)
        if u == v:
            continue
        wid, dir = _edge_lookup(cur, int(u), int(v))
        dist = distances[i] if with_metrics and i < len(distances) else None
        edges.append((wid, dir, dist))

    # compress by (way_id, dir) into "runs". node to node edges often form only part of a single way
    runs = []
    for (wid, d), group in groupby(edges, key=lambda e: (e[0], e[1])):
        g = list(group)
        # set edge count to be # of edges (essentially creaing hash map)
        edge_count = len(g)
        run = {"way_id": wid, "dir": d, "edge_count": edge_count}
        if with_metrics:
            length_m = sum(x[2] or 0.0 for x in g)
            run.update({"length_m": length_m})
        runs.append(run)

    con.close()
    return {
        "runs": runs,
        # keep these two tiny integers for merge math:
        "total_edges": max(0, len(nodes)-1)
    }


def format_list(values):
    """
    Convert a list of values to a semicolon-separated string.
    Example:
        [v1, v2] -> "v1;v2"
    """
    return ";".join(str(v) for v in values)


def route_fill_gaps(
    results_dir, output_dir, overview="full", geometries="geojson"
):  # Unused stub
    """
    Scan through match_result_*.json in results_dir,
    and whenever a file has no matchings, look ahead to the next file
    with matchings and call the /route endpoint to fill the gap.
    Saves any "gap routes" as gap_<idx_prev>_<idx_next>.json in output_dir.

    Note: This function is currently a stub and not called anywhere.
    """
    # TODO: Implement gap filling logic
    pass


def get_osrm_match(file_path, args):
    """
    Send a match request to the OSRM server for a given JSON chunk.

    Args:
        file_path (str): Path to the JSON chunk file.
        args: list of input arguments, used to get file outputs

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
    poly = quote(raw_poly, safe="")
    match_url = f"{OSRM_URL}/polyline6({poly})"

    # Prepare match parameters
    params = {
        "geometries": data.get("geometries", "geojson"),
        "overview": data.get("overview", "full"),
        "steps": str(data.get("steps", False)).lower(),
        "gaps": "ignore",
        "tidy": "true",
        "annotations": "nodes,distance"
    }
    # If timestamps are included in the dataset, add them here
    if "timestamps" in data:
        params["timestamps"] = format_list(data["timestamps"])

    # Static radius definition (testing)
    # radii = data["radiuses"]
    # Computer dynamic radiuses based on noise factor
    # around a given window size
    radii = compute_dynamic_radius(
        coords, data["radiuses"][0], window=int(args.dynamic_window), noise_scale=2.0
    )
    params["radiuses"] = ";".join(str(radius) for radius in radii)
    try:
        # Send request to OSRM /match endpoint
        response = requests.get(match_url, params=params, timeout=30)
        response.raise_for_status()
        # print(f"{base_name}: {response.status_code}, {
        #       response.json().get('code')}")
        result = response.json()

        # If multiple matchings found (more than 1 continuous route),
        # diagnose poorly matched points
        if len(result.get("matchings", [])) > 1:
            diag = diagnose_points("http://localhost:5000", coords, radius=500)
            for lat, lon, dist, name in diag:
                if dist is None or dist > 100:
                    print(
                        f"Point {(lat, lon)} snapped {dist}m away - nearest road: {
                            name
                        }"
                    )

    except requests.RequestException as e:
        # If request fails, log a warning and continue
        # print(f"[WARN] OSRM request failed -> {e}")
        result = {}

    # Determine output directory based on map results.
    # If a chunk could not be processed,
    # an empty chunk is produced in results/gap
    # for later processing
    out_dir = args.output_dir
    gap_dir = os.path.join(out_dir, "gap")
    dest = (
        os.path.join(out_dir, output_file)
        if result.get("matchings")
        else os.path.join(gap_dir, output_file)
    )
    # Convert Nodes to Ways:
    if result.get("matchings"):
        legs = result["matchings"][0].get("legs", [])
        route_nodes = _flatten_nodes(legs)
        result["route_nodes"] = route_nodes
        ways_obj = chunk_nodes_to_ways(result, args.waydb, args.with_metrics)
        result["ways"] = ways_obj["runs"]
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
    input_dir = args.input_dir
    # Prepare results directories
    results_dir = args.output_dir
    gap_dir = os.path.join(results_dir, "gap")
    os.makedirs(gap_dir, exist_ok=True)

    # Set max thread count to either the number of chunks to
    # process, or 32, whichever is less
    MAX_THREADS = min(32, sum(1 for name in os.listdir(input_dir)))
    print(f"Max threadcount: {MAX_THREADS}")
    os.makedirs(os.path.join(results_dir, "gap"), exist_ok=True)

    # =========== pull all chunk files from directory sorted ===========
    json_files = sorted(
        [
            os.path.join(input_dir, f)
            for f in os.listdir(input_dir)
            if f.endswith(".json")
        ]
    )
    # ============== Multithreaded requests =====================
    results = []
    # Send request on each thread to server running OSRM software
    with ThreadPoolExecutor(max_workers=MAX_THREADS) as executor:
        future_to_file = {
            executor.submit(get_osrm_match, f, args): f for f in json_files
        }
    print(
        f"[PYTHON] Matching complete for {
            os.path.basename(os.path.normpath(input_dir))
        }. {len(future_to_file)} files processed"
    )
    for future in as_completed(future_to_file):
        try:
            file, result = future.result()
            results.append((file, result))
        except Exception as e:
            print(f"[ERROR] Thread Failed: {e}")


if __name__ == "__main__":
    args = parse_args()
    main(args)
