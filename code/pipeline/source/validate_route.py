import json
import sqlite3
import argparse
import math
from typing import List, Optional
import osmium

from utils.gpx_utils import haversine_m


def load_route_nodes(d: dict) -> List[int]:
    nodes = d.get("route_nodes")
    if nodes:
        return [int(x) for x in nodes]
    # fallback: flatten legs if present
    matchings = d.get("matchings") or []
    legs = (matchings[0].get("legs") if matchings else None) or []
    out = []
    for leg in legs:
        seq = ((leg or {}).get("annotation") or {}).get("nodes") or []
        seq = [int(n) for n in seq]
        if not seq:
            continue
        if not out:
            out.extend(seq)
        else:
            # conservative 1-node overlap drop
            if out[-1] == seq[0]:
                out.extend(seq[1:])
            else:
                out.extend(seq)  # last resort
    return out


class NodeCoords(osmium.SimpleHandler):
    def __init__(self, ids: List[int]):
        super().__init__()
        self.want = set(int(x) for x in ids)
        self.coords = {}

    def node(self, n):
        i = int(n.id)
        if i in self.want and n.location.valid():
            self.coords[i] = (n.location.lat, n.location.lon)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "merged_json", help="Merged OSRM JSON (must contain route_nodes + ways)")
    ap.add_argument("waydb", help="way_index.sqlite path")
    ap.add_argument("--pbf", help="(optional) PBF for geo sanity checks")
    ap.add_argument("--jump_thresh_m", type=float, default=80.0,
                    help="Flag consecutive-node jumps above this (default 80 m)")
    args = ap.parse_args()

    data = json.load(open(args.merged_json))
    nodes = load_route_nodes(data)
    runs = data.get("ways") or []

    if len(nodes) < 2:
        print("[FAIL] No route_nodes; cannot validate.")
        return

    # 1) Edge existence
    con = sqlite3.connect(args.waydb)
    cur = con.cursor()
    missing = []
    for i, (u, v) in enumerate(zip(nodes[:-1], nodes[1:])):
        row = cur.execute(
            "SELECT 1 FROM edges WHERE u=? AND v=? LIMIT 1", (int(u), int(v))).fetchone()
        if not row:
            missing.append((i, int(u), int(v)))
    con.close()

    # 2) Runs consistency
    sum_edges = sum(int(r.get("edge_count", 0)) for r in runs)
    spine_edges = len(nodes) - 1
    runs_ok = (sum_edges == spine_edges)

    # 3) Geo sanity (optional)
    geo_issues = []
    if args.pbf:
        h = NodeCoords(nodes)
        h.apply_file(args.pbf, locations=True)
        coords = h.coords
        for i, (u, v) in enumerate(zip(nodes[:-1], nodes[1:])):
            pu = coords.get(int(u))
            pv = coords.get(int(v))
            if not pu or not pv:
                continue
            d = haversine_m(pu[0], pu[1], pv[0], pv[1])
            if d > args.jump_thresh_m:
                geo_issues.append((i, int(u), int(v), round(d, 1)))

    # Report
    print("=== Route integrity check ===")
    print(f"nodes: {len(nodes)}  edges(spine): {spine_edges}")
    print(f"ways(runs): {len(runs)}  sum(edge_count): {
          sum_edges}  CONSISTENT: {runs_ok}")
    print(f"missing directed edges in DB: {len(missing)}")
    if missing:
        print("  sample:", missing[:5])
    if args.pbf:
        print(f"geo jumps > {args.jump_thresh_m} m: {len(geo_issues)}")
        if geo_issues:
            print("  sample:", geo_issues[:5])

    # Exit code hint (0 good, 1 warnings)
    if missing or (args.pbf and geo_issues) or not runs_ok:
        print("[WARN] Route has issues (see above).")
    else:
        print("[OK] Route is gap-free and consistent.")


if __name__ == "__main__":
    main()
