"""
Plot 'ways' runs from a single OSRM JSON (chunk or merged), alternating colours.

Usage:
  python plot_ways_from_json.py route.json map.pbf out.html

Requires:
  pip install osmium folium polyline
"""

import argparse
import json
from typing import List, Dict, Tuple, Optional
import folium
import osmium
import polyline


# ---------- helpers ----------
def flatten_nodes_robust(legs, max_overlap_check=8) -> List[int]:
    """Flatten legs' annotation.nodes into one list without seam duplicates."""
    if isinstance(legs, dict):
        legs = [legs]
    out: List[int] = []
    for leg in (legs or []):
        ann = (leg or {}).get("annotation") or {}
        seq = ann.get("nodes") or []
        if not seq:
            continue
        seq = [int(n) for n in seq]
        if not out:
            out.extend(seq)
            continue
        # find largest K s.t. tail(out, K) == head(seq, K)
        max_k = min(len(out), len(seq), max_overlap_check)
        K = 0
        for k in range(max_k, 0, -1):
            if out[-k:] == seq[:k]:
                K = k
                break
        # append remainder, guarding against consecutive dupes
        for n in seq[K:]:
            if not out or n != out[-1]:
                out.append(n)
    # paranoia sweep
    dedup: List[int] = []
    for n in out:
        if not dedup or n != dedup[-1]:
            dedup.append(n)
    return dedup


class NodeCollector(osmium.SimpleHandler):
    """Collect lat/lon for a target set of node IDs from a PBF."""

    def __init__(self, target_ids: List[int]):
        super().__init__()
        self.target = set(target_ids)
        self.coords: Dict[int, Tuple[float, float]] = {}

    def node(self, n):
        nid = int(n.id)
        if nid in self.target and n.location.valid():
            self.coords[nid] = (n.location.lat, n.location.lon)


def slice_nodes_by_runs(nodes: List[int], runs: List[dict]) -> List[List[int]]:
    """Convert runs (with edge_count) into contiguous node slices (each has k+1 nodes)."""
    slices: List[List[int]] = []
    pos = 0  # edge index along nodes
    total_edges = max(0, len(nodes) - 1)
    for r in runs:
        k = int(r.get("edge_count", 0))
        k = max(0, min(k, max(0, total_edges - pos)))  # clamp
        start = pos
        end = pos + k
        node_slice = nodes[start: end + 1] if k > 0 else []
        slices.append(node_slice)
        pos += k
    return slices


def decode_osrm_geometry(osrm: dict) -> List[Tuple[float, float]]:
    """Return [(lat,lon), ...] from OSRM geometry if present (for reference overlay)."""
    m = (osrm.get("matchings") or [{}])[0]
    geom = m.get("geometry")
    if isinstance(geom, str):
        try:
            return polyline.decode(geom, precision=6)  # [(lat,lon)]
        except Exception:
            return []
    if isinstance(geom, dict) and geom.get("type") == "LineString":
        # GeoJSON: [lon,lat]
        return [(lat, lon) for lon, lat in geom.get("coordinates", [])]
    return []


# ---------- main ----------
PALETTE = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "osrm_json", help="OSRM JSON with 'ways' and leg annotations")
    ap.add_argument("pbf", help="PBF used for the map")
    ap.add_argument("out_html", help="Output HTML map path")
    args = ap.parse_args()

    osrm = json.load(open(args.osrm_json))
    runs = osrm.get("ways") or []
    if not runs:
        raise SystemExit(
            "This JSON has no 'ways' array. Generate ways first, then re-run.")

    # We need the node path to slice by edge_count
    matchings = osrm.get("matchings") or []
    legs = (matchings[0].get("legs") if matchings else None) or []
    nodes = flatten_nodes_robust(legs)
    if len(nodes) < 2:
        raise SystemExit(
            "No nodes found in annotations. Did you include annotations=nodes when matching/merging?")

    # Resolve node coords from the PBF
    collector = NodeCollector(nodes)
    collector.apply_file(args.pbf, locations=True)
    coords = collector.coords  # {node_id: (lat,lon)}
    resolved = sum(1 for n in nodes if n in coords)
    print(f"[INFO] nodes={len(nodes)}  resolved={
          resolved}  missing={len(nodes)-resolved}")

    # Build node-slices per run
    node_slices = slice_nodes_by_runs(nodes, runs)

    # Base map centered on first available point
    center = None
    for nid in nodes:
        if nid in coords:
            center = coords[nid]
            break
    if not center:
        raise SystemExit(
            "Could not resolve any node coordinates from the PBF.")

    m = folium.Map(location=center, zoom_start=13, tiles="OpenStreetMap")

    # Optional: overlay the full OSRM geometry (faint grey)
    geom = decode_osrm_geometry(osrm)
    if len(geom) >= 2:
        folium.PolyLine(geom, weight=2, opacity=0.35).add_to(m)

    # Draw each run in alternating colours
    for i, (r, ns) in enumerate(zip(runs, node_slices)):
        if len(ns) < 2:
            continue
        seg = [(coords[n][0], coords[n][1]) for n in ns if n in coords]
        if len(seg) < 2:
            continue
        color = PALETTE[i % len(PALETTE)]
        meta = f"way_id={r.get('way_id')} dir={r.get('dir')} edges={
            r.get('edge_count')}"
        folium.PolyLine(seg, weight=5, opacity=0.85,
                        color=color, popup=meta).add_to(m)

    m.save(args.out_html)
    print(f"[OK] saved → {args.out_html}")


if __name__ == "__main__":
    main()
