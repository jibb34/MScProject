import os
import sqlite3
import argparse
import json
from concurrent.futures import ThreadPoolExecutor, as_completed
import copy
from itertools import groupby
from utils.osm_utils import (
    _overlap_len,
    _flatten_nodes,
    shortest_connector_nodes_db,
    _edge_lookup)


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


N_OVERLAP = 200  # nodes to check


def _max_waypoint_index(tps):
    mx = -1
    for tp in tps or []:
        if isinstance(tp, dict):
            w = tp.get("waypoint_index")
            if w is not None and w > mx:
                mx = w
    return mx


def _normalize_tracepoints_single_matching(tps, waypoint_offset=0):
    """Return a copy where all non-null tracepoints have matchings_index=0 and
    waypoint_index += waypoint_offset. Nulls (None) are preserved."""
    out = []
    for tp in tps or []:
        if isinstance(tp, dict):
            tp2 = dict(tp)  # shallow copy
            tp2["matchings_index"] = 0
            if tp2.get("waypoint_index") is not None:
                tp2["waypoint_index"] = int(
                    tp2["waypoint_index"]) + int(waypoint_offset)
            out.append(tp2)
        else:
            out.append(None)
    return out


def _nodes_to_ways(nodes, cur, with_metrics=False, distances=None):
    edges = []
    di = 0
    for u, v in zip(nodes[:-1], nodes[1:]):
        if u == v:
            continue
        wid, d, db_len = _edge_lookup(cur, int(u), int(v))
        dist = db_len if with_metrics and db_len is not None else (
            float(distances[di]) if with_metrics and distances and di < len(
                distances) else None
        )
        edges.append((wid, d, dist))
        di += 1
    runs = []
    for (wid, d), group in groupby(edges, key=lambda e: (e[0], e[1])):
        g = list(group)
        run = {"way_id": wid, "dir": d, "edge_count": len(g)}
        if with_metrics:
            run["length_m"] = sum((x[2] or 0.0) for x in g)
        runs.append(run)
    return runs, max(0, len(nodes) - 1)


def _trim_B_legs_by_nodes(legs_b, k_nodes, cur, with_metrics=False):
    """Drop the first k_nodes from B across its legs; recompute runs/total_edges per affected leg."""
    out = []
    remaining = int(k_nodes)
    for leg in legs_b:
        ann = (leg.get("annotation") or {})
        nodes = [int(x) for x in (ann.get("nodes") or [])]
        if not nodes:
            continue
        if remaining <= 0:
            out.append(leg)
            continue

        # how many nodes to drop from THIS leg
        drop = min(remaining, len(nodes))
        remaining -= drop

        if drop == len(nodes):
            # this leg is entirely overlapped — skip it
            continue

        # partially trim this leg
        new_nodes = nodes[drop:]
        new_distances = ann.get("distance") or []
        # drop exactly 'drop' edge distances from the front
        new_distances = new_distances[drop:] if new_distances else []

        new_leg = copy.deepcopy(leg)
        new_leg["annotation"] = dict(
            ann, nodes=new_nodes, distance=new_distances)

        # recompute runs + edge count for this leg
        runs, k = _nodes_to_ways(
            new_nodes, cur, with_metrics=with_metrics, distances=new_distances)
        new_leg["runs"] = runs
        new_leg["total_edges"] = k

        # (optional) adjust leg.distance from distances array if present
        if new_distances:
            new_leg["distance"] = float(sum(new_distances))

        out.append(new_leg)

        # once we’ve trimmed into the first leg, remaining will be 0; rest append as-is
    # if overlap spilled across many legs, we already skipped them above
    return out


def _merge_coords(coords_a, coords_b):
    max_check = min(64, len(coords_a), len(coords_b))
    k = 0
    # try to find the largest exact overlap (same coordinate pairs)
    for t in range(max_check, 0, -1):
        if coords_a[-t:] == coords_b[:t]:
            k = t
            break
    return coords_a + coords_b[k:]


def merge_two(a, b, waydb_path, with_metrics=False, stitch_if_needed=False):
    # pull matchings & legs
    ma = (a.get("matchings") or [])[0]
    mb = (b.get("matchings") or [])[0]
    legs_a = ma.get("legs") or []
    legs_b = mb.get("legs") or []

    # flatten node spines to detect overlap
    nodes_a_full = [int(x) for x in _flatten_nodes(legs_a)]
    nodes_b_full = [int(x) for x in _flatten_nodes(legs_b)]

    k_nodes = _overlap_len(nodes_a_full[-N_OVERLAP:], nodes_b_full[:N_OVERLAP])
    con = sqlite3.connect(waydb_path)
    cur = con.cursor()

    if k_nodes == 0 and stitch_if_needed:
        # optional: create a connector leg between A tail and B head
        u_last = nodes_a_full[-1]
        v_first = nodes_b_full[0]
        connector_nodes = shortest_connector_nodes_db(
            cur, u_last, v_first, max_expansions=200000)
        if connector_nodes and len(connector_nodes) >= 2:
            runs, k = _nodes_to_ways(connector_nodes, cur, with_metrics=False)
            connector_leg = {
                # distance optional
                "annotation": {"nodes": connector_nodes, "distance": []},
                "steps": [],
                "distance": 0.0,
                "duration": 0.0,
                "summary": "",
                "weight": 0.0,
                "runs": runs,
                "total_edges": k,
            }
            legs_merged = legs_a + [connector_leg] + legs_b
        else:
            legs_merged = legs_a + legs_b
    else:
        # trim B by the exact node overlap
        legs_b_trimmed = _trim_B_legs_by_nodes(
            legs_b, k_nodes, cur, with_metrics=with_metrics)
        legs_merged = legs_a + legs_b_trimmed

    con.close()

    # merge geometry (LineString coords)
    coords_a = (ma.get("geometry") or {}).get("coordinates") or []
    coords_b = (mb.get("geometry") or {}).get("coordinates") or []
    merged_coords = _merge_coords(coords_a, coords_b)

    # recompute top-level metrics from legs
    tot_dist = sum(float(leg.get("distance", 0.0)) for leg in legs_merged)
    tot_dur = sum(float(leg.get("duration", 0.0)) for leg in legs_merged)
    tot_wt = sum(float(leg.get("weight", 0.0)) for leg in legs_merged)

    a_tps = a.get("tracepoints") or []
    b_tps = b.get("tracepoints") or []

    a_wmax = _max_waypoint_index(a_tps)          # -1 if A has no non-null
    offset = (a_wmax + 1) if a_wmax is not None else 0

    tps_a_norm = _normalize_tracepoints_single_matching(
        a_tps, waypoint_offset=0)
    tps_b_norm = _normalize_tracepoints_single_matching(
        b_tps, waypoint_offset=offset)

    tracepoints_merged = tps_a_norm + tps_b_norm

    merged = {
        "code": "Ok" if a.get("code") == "Ok" and b.get("code") == "Ok" else "Error",
        "matchings": [{
            "confidence": (ma.get("confidence", 0.0) + mb.get("confidence", 0.0)) / 2.0,
            "geometry": {"type": "LineString", "coordinates": merged_coords},
            "legs": legs_merged,
            "distance": tot_dist,
            "duration": tot_dur,
            "weight_name": ma.get("weight_name", "duration"),
            "weight": tot_wt
        }],
        "tracepoints": tracepoints_merged
    }
    return merged


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
            future = executor.submit(
                merge_two, matches[i], matches[i+1], WAY_DB, True, True)
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
        json.dump(merged, f)

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
