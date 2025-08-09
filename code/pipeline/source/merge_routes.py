import os
import sqlite3
import argparse
import json
from concurrent.futures import ThreadPoolExecutor, as_completed
from utils.osm_utils import extract_coords, merge_runs, _overlap_len, _runs_from_chunk, _flatten_nodes


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in the input and output directory"
    )
    parser.add_argument(
        "input_dir", help="Directory that contains all JSON chunks of the GPX file"
    )
    parser.add_argument(
        "output_dir", help="Directory that the match file is sent to")
    parser.add_argument("--waydb", default="way_index.sqlite",
                        help="Path to way_index.sqlite (directed edge index)")
    parser.add_argument("--with-metrics", action="store_true",
                        help="Sum distance/duration into way runs if present")
    return parser.parse_args()


def merge_two(route_a, route_b):
    """Merge logic: Takes two routes, and attempts to merge them into 1"""

    if not route_a.get("matchings") or not route_b.get("matchings"):
        print("[ERROR] One of the matchings lists is empty")
        return {"code": "Error"}

    match_a = route_a["matchings"][0]
    match_b = route_b["matchings"][0]

    code = "Ok" \
        if route_a["code"] == "Ok" \
        and route_b["code"] == "Ok" \
        else "Error"

    if not route_a.get("matchings") or not route_b.get("matchings"):
        print("[ERROR] One of the matchings lists is empty")
        return {"code": "Error"}

    if not match_a.get("geometry") or not match_b.get("geometry"):
        print("[ERROR] One of the matchings has no geometry")
        return {"code": "Error"}

    coords_a = extract_coords(match_a, "route_a")
    coords_b = extract_coords(match_b, "route_b")

    # Avoid duplicates at endpoints:
    if coords_a and coords_b and coords_a[-1] == coords_b[0]:
        coords_b = coords_b[1:]

    # Merge coordinates and convert to geometry
    merged_coords = coords_a + coords_b
    merged_geometry = {
        "type": "LineString",
        "coordinates": [(lon, lat) for lat, lon in merged_coords]
    }

    # Merge all other values

    # Merge way runs per chunk:
    # --- Way runs per chunk ---
    # If 'ways' are already present (from previous round or batch precompute), use them.
    runs_a = route_a.get("ways")
    runs_b = route_b.get("ways")

    nodes_a_full = route_a.get("route_nodes") or _flatten_nodes(
        match_a.get("legs", []))
    nodes_b_full = route_b.get("route_nodes") or _flatten_nodes(
        match_b.get("legs", []))

    nodes_a_head = route_a.get("nodes_head") or []
    nodes_a_tail = route_a.get("nodes_tail") or []
    nodes_b_head = route_b.get("nodes_head") or []
    nodes_b_tail = route_b.get("nodes_tail") or []
    k = _overlap_len(nodes_a_full, nodes_b_head)
    if nodes_a_full and nodes_b_full:
        route_nodes_merged = nodes_a_full + nodes_b_full[k:]
    else:
        # last resort — we don’t have full spines; keep A’s head and B’s tail summaries
        route_nodes_merged = (nodes_a_head or []) + (nodes_b_tail or [])

    # If missing, compute from legs (first-round chunks)
    if runs_a is None or nodes_a_tail == [] or nodes_a_head == []:
        con = sqlite3.connect(WAY_DB)
        cur = con.cursor()
        runs_a, _total_edges_a, nodes_full_a = _runs_from_chunk(
            match_a, cur, WITH_METRICS)
        con.close()
        N = 50
        nodes_a_head = nodes_a_head or nodes_full_a[:N]
        nodes_a_tail = nodes_a_tail or nodes_full_a[-N:]
    if runs_b is None or nodes_b_head == [] or nodes_b_tail == []:
        con = sqlite3.connect(WAY_DB)
        cur = con.cursor()
        runs_b, _total_edges_b, nodes_full_b = _runs_from_chunk(
            match_b, cur, WITH_METRICS)
        con.close()
        N = 50
        nodes_b_head = nodes_b_head or nodes_full_b[:N]
        nodes_b_tail = nodes_b_tail or nodes_full_b[-N:]

    # If both match a and b have nodes, and if the last node of a is the same as first of b
    # then skip the last node (to avoid duplicates)

    k = _overlap_len(nodes_a_tail, nodes_b_head)  # seam based on summaries
    if k == 0:
        print("[WARN] No node overlap; assuming k=1 seam.")
        k = 1
    overlap_edges = max(0, k-1)

    merged_runs = merge_runs(runs_a or [], runs_b or [], overlap_edges)
    # merged node summaries for the next round:
    # head stays from A, tail stays from B (unless B fully overlapped)
    nodes_head_merged = nodes_a_head or []
    nodes_tail_merged = nodes_b_tail or []

    merged_dist = match_a["distance"] + match_b["distance"]
    merged_duration = match_a["duration"] + match_b["duration"]
    merged_weight = match_a["weight"] + match_b["weight"]
    merged_confidence = (match_a["confidence"] + match_b["confidence"]) / 2

    # New unified match:
    unified = {
        "code": code,
        "matchings": [{
            "confidence": merged_confidence,
            "geometry": merged_geometry,
            "legs": [],  # No legs since we are storing ways
            "weight_name": "duration",
            "weight": merged_weight,
            "duration": merged_duration,
            "distance": merged_dist
        }],
        "tracepoints": route_a["tracepoints"] + route_b["tracepoints"],
        "route_nodes": route_nodes_merged,
        "ways": merged_runs,
        "nodes_head": nodes_head_merged,
        "nodes_tail": nodes_tail_merged
    }
    return unified


def tree_merge_routes(matches, executor):
    """Takes a sorted array of json objects, and recursively merges the objects
    until there is only 1
    - sorted_input: A list of json objects, in order of occurance in the route
    - executor: ThreadPoolExecutor instance to submit work to
      """
    # === BASE CASE ===
    if len(matches) == 1:
        return matches[0]
    # list of future async computation results
    future_to_index = {}
    # initialize to half the size of the current matches array (rounded up)
    # one for each pair.
    merged_results = [None] * ((len(matches) + 1) // 2)

    # For every 2 values... multithreaded match
    for i in range(0, len(matches), 2):
        if i + 1 < len(matches):
            # if the value exists in the index range:
            future = executor.submit(merge_two, matches[i], matches[i+1])
            # Associate each future merge pair with an index
            # for sorting after resync
            future_to_index[future] = i // 2
            # future:index pair where each future retains it's index
        else:
            # There is only 1 value left at the end of index (odd one out)
            merged_results[i // 2] = matches[i]

    # Thread ReSync
    # make sure list is still ordered
    for future in as_completed(future_to_index):
        # For every thead result calculated, merge into
        index = future_to_index[future]
        merged_results[index] = future.result()
    # recursive call
    # print(f"Merging {len(merged_results)} chunks")
    return tree_merge_routes(merged_results, executor)


def get_json_data(file_path):
    """ Get the code, matchings and tracepoints from the json objects"""
    with open(file_path, "r") as f:
        data = json.load(f)

    # pick top-level route_nodes; if absent, check matchings[0]
    route_nodes = data.get("route_nodes")
    if not route_nodes:
        ms = data.get("matchings", [])
        if ms and isinstance(ms[0], dict):
            route_nodes = ms[0].get("route_nodes")
    return {
        "code": data.get("code", "Error"),
        "matchings": data.get("matchings", []),
        "tracepoints": data.get("tracepoints", []),
        "route_nodes": route_nodes,
        "ways": data.get("ways", []),
        "nodes_head": [],
        "nodes_tail": []

    }


def main(input_dir, output_dir, output_name):
    # Threads is the lesser of 32 or the number of files to parse
    INPUT_DIR = input_dir
    OUTPUT_DIR = output_dir
    output_json = os.path.join(OUTPUT_DIR, (output_name + ".json"))
    num_threads = min(32, sum(1 for name in os.listdir(INPUT_DIR)))
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    json_files = sorted([
        os.path.join(INPUT_DIR, f) for f in os.listdir(INPUT_DIR)
        if f.endswith(".json")
    ])
    # Start thread executor
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        print(f"[INFO] Loading {len(json_files)} match files...")
        match_chunks = list(executor.map(get_json_data, json_files))
        # print(f"[INFO] Recursively merging using {num_threads} threads...")
        merged = tree_merge_routes(match_chunks, executor)

    print(f"[CHECK] Total matchings in final merge: {
          len(merged['matchings'])}")
    print(f"[CHECK] Total tracepoints in final merge: {
          len(merged['tracepoints'])}")
    with open(output_json, "w") as f:
        json.dump(merged, f, indent=2)

    return 0


if __name__ == "__main__":
    args = parse_args()
    global WAY_DB, WITH_METRICS
    WAY_DB = args.waydb
    WITH_METRICS = args.with_metrics

    input_directory = args.input_dir
    output_dir = args.output_dir
    # name of directory for naming the merge
    output_name = os.path.basename(os.path.normpath(input_directory))
    main(input_directory, output_dir, output_name)
