import osmnx as ox
import os
import numpy as np
import json
import argparse

EARTH_R = 6371000  # Radius of earth


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in a JSON data file and convert it to a network of OSM street data"
    )
    parser.add_argument(
        "json_file", help="The JSON file that the GPS Observations are being read from"
    )
    parser.add_argument(
        "--graphml",
        help="Optional: Path to a GraphML file containing saved map data. If not provided, map data will be downloaded using GPX bounding box.",
    )
    return parser.parse_args()


def open_json(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)
    return data


def bounding_box_from_json(data):
    # TODO: Possibly may need to expand the bbox slightly if error matching causes issues, route could go outside the box even if gpx points don't
    # Get all lat/long values
    lats = [p["lat"] for p in data if "lat" in p and p["lat"] is not None]
    longs = [p["long"] for p in data if "long" in p and p["long"] is not None]
    # get the extrema
    north = max(lats)
    south = min(lats)
    east = max(longs)
    west = min(longs)

    # return a quaduple of the extrema in bbox format (left, bottom, right, top)
    return (west, south, east, north)


def load_road_graph(bbox):
    ox.settings.use_cache = True
    os.makedirs("./cache", exist_ok=True)
    ox.settings.cache_folder = "./cache"
    G = ox.graph_from_bbox(bbox, network_type="bike")
    ox.simplification.consolidate_intersections(
        G, tolerance=10, reconnect_edges=True)

    # Filter for road types bikes can use
    return G


# Haverine formula for calculating distance
def haversine(lat1, lon1, lat2, lon2):
    # convert to radians
    lat1, lon1, lat2, lon2 = map(np.radians, (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2.0) ** 2 + np.cos(lat1) * \
        np.cos(lat2) * np.sin(dlon / 2.0) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return EARTH_R * c


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    args = parse_args()
    print(f"type(args.json_file): {type(args.json_file)}")
    json_path = args.json_file
    points = open_json(json_path)
    bbox = bounding_box_from_json(points)
    if args.graphml and os.path.isfile(args.graphml):
        print(f"Loading graph from {args.graphml}")
        G = ox.load_graphml(args.graphml)
    else:
        print("No GraphML file provided — generating graph from bounding box.")
        G = load_road_graph(bbox)
        map_file_path, _ = os.path.splitext(args.json_file)
        ox.io.save_graphml(G, map_file_path + ".graphml")

    # Convert to GeoDataFrame
    # Filter points

    xs = [pt["long"] for pt in points if "lat" in pt and "long" in pt]
    ys = [pt["lat"] for pt in points if "lat" in pt and "long" in pt]
    # Plot the bike-accessible road graph
    fig, ax = ox.plot_graph(
        G, node_size=1, edge_color="blue", show=False, close=False
    )  # Replace with real invocation
    ax.scatter(xs, ys, c="red", s=30, label="GPS Points", zorder=3)

    plt.show()
