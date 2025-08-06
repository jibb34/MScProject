import os
import argparse
import json
from concurrent.futures import ThreadPoolExecutor, as_completed
import polyline


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in the input and output directory"
    )
    parser.add_argument(
        "input_dir", help="Directory that contains all JSON chunks of the GPX file"
    )
    parser.add_argument(
        "output_dir", help="Directory that the match file is sent to")
    return parser.parse_args()


def extract_coords(matching, label=""):
    geometry = matching.get("geometry")
    # If polyline:
    if isinstance(geometry, str):
        try:
            return polyline.decode(geometry, precision=6)
        except Exception as e:
            print(f"[ERROR] Failed to decode poylyline from {label}")
            return []
    elif isinstance(geometry, dict) and geometry.get("type") == "LineString":
        return [(lat, lon) for lon, lat in geometry["coordinates"]]
    else:
        print(f"[ERROR] Unsupported geometry format in {label}")
        return []


def merge_two(route_a, route_b):
    """Merge logic: Takes two routes, and attempts to merge them into 1"""
    match_a = route_a["matchings"][0]
    match_b = route_b["matchings"][0]

    code = "Ok" \
        if route_a["code"] == "Ok" \
        and route_b["code"] == "Ok" \
        else "Error"
    # Decode polylines of routes:
    if not route_a.get("matchings") or not route_b.get("matchings"):
        print("[ERROR] One of the matchings lists is empty")
        return {"code": "Error"}

    if not match_a.get("geometry") or not match_b.get("geometry"):
        print("[ERROR] One of the matchings has no geometry")
        return {"code": "Error"}

    coords_a = []
    coords_b = []
    try:
        coords_a = extract_coords(match_a, "route_a")
        coords_b = extract_coords(match_b, "route_b")
    except Exception as e:
        print(f"[ERROR] Failed to decode Geometry: {e}")
        return {"code": "Error"}
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

    merged_legs = match_a["legs"] + match_b["legs"]
    merged_dist = match_a["distance"] + match_b["distance"]
    merged_duration = match_a["duration"] + match_b["duration"]
    merged_weight = match_a["weight"] + match_b["weight"]
    merged_confidence = (match_a["confidence"] + match_b["confidence"]) / 2

    # New unified match:
    unified = {
        "code": "Ok",
        "matchings": [{
            "confidence": merged_confidence,
            "geometry": merged_geometry,
            "legs": merged_legs,
            "weight_name": "duration",
            "weight": merged_weight,
            "duration": merged_duration,
            "distance": merged_dist
        }],
        "tracepoints": route_a["tracepoints"] + route_b["tracepoints"]
    }
    return unified

    return {
        "code": code,
        "matchings": route_a["matchings"] + route_b["matchings"],
        "tracepoints": route_a["tracepoints"] + route_b["tracepoints"]
    }


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
    return {
        "code": data.get("code", "Error"),
        "matchings": data.get("matchings", []),
        "tracepoints": data.get("tracepoints", [])
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
    # print(json_files)
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
        json.dump(merged, f)

    return 0


if __name__ == "__main__":
    args = parse_args()
    input_directory = args.input_dir
    output_dir = args.output_dir
    # name of directory for naming the merge
    output_name = os.path.basename(os.path.normpath(input_directory))
    main(input_directory, output_dir, output_name)
