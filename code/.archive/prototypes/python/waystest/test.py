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


def find_way_sequence(G, matched_nodes):
    # Find shortest path segments between consecutive matched nodes
    path = []
    for u, v in zip(matched_nodes, matched_nodes[1:]):
        try:
            sub = nx.shortest_path(G, u, v, weight="weight")
        except nx.NetworkXNoPath:
            continue
        path.extend(sub[:-1])
    path.append(matched_nodes[-1])
    return path


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
    print("Matching points to nodes...")
    matched_nodes = map_match_points(pts, nodes)
    print("Computing path sequence...")
    node_path = find_way_sequence(G, matched_nodes)
    print("Generating map...")
    generate_map(pts, node_path, nodes)


if __name__ == "__main__":
    main()

# Requirements:
# pip install gpxpy requests folium networkx shapely scipy tqdm

