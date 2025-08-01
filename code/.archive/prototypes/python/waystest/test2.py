"""
Script: gpx_snap_and_pathfind.py

Reads a GPX file of trackpoints, snaps each GPS point to the nearest road segment using a map-matching approach,
then determines the sequence of OSM ways connecting those snapped nodes, and generates an HTML map overlaying the
matched path.

Uses:
- Map matching via OSM Overpass + a shortest-path on the OSM graph between snapped points.
- Folium for interactive map output.
- NetworkX for graph traversal.
- references: methods inspired by “Map Matching with Hidden Markov Models” (Arnaud de La... etc)
"""

import sys
import requests
import gpxpy
import folium
import networkx as nx
from shapely.geometry import LineString, Point
from scipy.spatial import cKDTree
from tqdm import tqdm

OVERPASS_URL = "https://overpass-api.de/api/interpreter"


def load_trackpoints(gpx_path):
    with open(gpx_path, "r") as f:
        gpx = gpxpy.parse(f)
    return [
        (pt.latitude, pt.longitude)
        for tr in gpx.tracks
        for seg in tr.segments
        for pt in seg.points
    ]


def fetch_osm_graph(bbox, highway_filter="[highway]"):
    # Download all ways and nodes in bbox
    query = f"""
[out:json][timeout:120];
(
  way{highway_filter}({bbox[1]},{bbox[0]},{bbox[3]},{bbox[2]});
  >;
);
out body;
"""
    r = requests.post(OVERPASS_URL, data={"data": query})
    r.raise_for_status()
    data = r.json()["elements"]

    # Build node dict and graph
    nodes = {e["id"]: (e["lat"], e["lon"])
             for e in data if e["type"] == "node"}
    G = nx.DiGraph()
    for e in data:
        if e["type"] == "way":
            nds = e["nodes"]
            for u, v in zip(nds, nds[1:]):
                G.add_edge(
                    u, v, id=e["id"], weight=Point(
                        nodes[u]).distance(Point(nodes[v]))
                )
                G.add_edge(
                    v, u, id=e["id"], weight=Point(
                        nodes[v]).distance(Point(nodes[u]))
                )
    return G, nodes


def map_match_points(pts, nodes):
    # Build KD-tree on node coords
    coords = [nodes[n] for n in nodes]
    ids = list(nodes.keys())
    tree = cKDTree(coords)
    matched = []
    for lat, lon in pts:
        dist, idx = tree.query((lat, lon))
        matched.append(ids[idx])
    return matched


def find_way_sequence(G, matched_nodes, nodes, min_way_length=20.0):
    """
    Build full node path via shortest paths, segment by way IDs, filter out short segments,
    and return a smooth sequence of nodes.
    """
    # 1) Build full node path by concatenating shortest paths
    full_path = []
    for u, v in zip(matched_nodes, matched_nodes[1:]):
        try:
            sub = nx.shortest_path(G, u, v, weight="weight")
        except nx.NetworkXNoPath:
            continue
        if full_path and full_path[-1] == sub[0]:
            full_path.extend(sub[1:])
        else:
            full_path.extend(sub)

    # 2) Extract (node, node, way_id) edges
    edge_ways = []
    for u, v in zip(full_path, full_path[1:]):
        data = G.get_edge_data(u, v, default=None)
        way_id = data["id"] if data and "id" in data else None
        edge_ways.append((u, v, way_id))

    # 3) Group into segments by way_id
    segments = []
    if edge_ways:
        curr_way = edge_ways[0][2]
        curr_nodes = [edge_ways[0][0], edge_ways[0][1]]
        for u, v, wid in edge_ways[1:]:
            if wid == curr_way and wid is not None:
                curr_nodes.append(v)
            else:
                segments.append((curr_way, curr_nodes))
                curr_way = wid
                curr_nodes = [u, v]
        segments.append((curr_way, curr_nodes))

    # 4) Filter out only truly short segments (except first/last)
    filtered_seqs = []
    total = len(segments)
    for idx, (wid, seq) in enumerate(segments):
        if idx == 0 or idx == total - 1:
            filtered_seqs.append(seq)
            continue
        # compute segment length
        seg_len = 0.0
        for a, b in zip(seq, seq[1:]):
            seg_len += Point(nodes[a]).distance(Point(nodes[b]))
        if seg_len >= min_way_length:
            filtered_seqs.append(seq)
        else:
            # merge small segment into neighbors by skipping adding it
            pass

    # 5) Flatten filtered sequences preserving continuity
    smoothed = []
    for seq in filtered_seqs:
        if not smoothed:
            smoothed.extend(seq)
        else:
            if smoothed[-1] == seq[0]:
                smoothed.extend(seq[1:])
            else:
                smoothed.extend(seq)
    return smoothed


def generate_map(pts, node_path, nodes):
    # convert to latlon
    track_coords = [(pt[0], pt[1]) for pt in pts]
    path_coords = [(nodes[n][0], nodes[n][1]) for n in node_path]
    center = track_coords[len(track_coords) // 2]
    m = folium.Map(location=center, zoom_start=14)
    folium.PolyLine(
        track_coords, color="blue", weight=3, opacity=0.6, tooltip="Original Track"
    ).add_to(m)
    folium.PolyLine(
        path_coords, color="red", weight=3, opacity=0.8, tooltip="Matched Path"
    ).add_to(m)
    out = "matched_map.html"
    m.save(out)
    print(f"Map saved to {out}")


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} path/to/track.gpx")
        sys.exit(1)
    gpx_path = sys.argv[1]
    pts = load_trackpoints(gpx_path)
    lats = [p[0] for p in pts]
    lons = [p[1] for p in pts]
    bbox = (min(lons), min(lats), max(lons), max(lats))
    print("Fetching OSM graph...")
    G, nodes = fetch_osm_graph(bbox)
    print("Map matching via OSRM API...")
    # Build coordinate string for OSRM match service
    coords = ";".join(
        f"{lon},{lat}"
        for lat, latlon in [(p[0], p[1]) for p in pts]
        for lon, lat in [(latlon, p[0]) for p in pts]
    )
    osrm_url = f"http://router.project-osrm.org/match/v1/driving/{{coords}}?geometries=geojson&overview=full&steps=false"
    r = requests.get(osrm_url)
    r.raise_for_status()
    match = r.json()
    # Extract matched node IDs by finding nearest OSM nodes from matched coordinates
    matched_nodes = []
    for mpt in match.get("matchings", []):
        for coord in mpt["geometry"]["coordinates"]:
            # OSRM returns [lon, lat]
            point = (coord[1], coord[0])
            # Find nearest node in our OSM graph
            _, idx = cKDTree(list(nodes.values())).query(point)
            node_id = list(nodes.keys())[idx]
            matched_nodes.append(node_id)
    print(f"Matched {len(matched_nodes)} points via OSRM")
    print("Computing path sequence...")
    node_path = find_way_sequence(G, matched_nodes, nodes)
    print("Generating map...")
    generate_map(pts, node_path, nodes)


if __name__ == "__main__":
    main()

# Requirements:
# pip install gpxpy requests folium networkx shapely scipy tqdm
