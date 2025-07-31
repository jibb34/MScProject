import osmnx as ox
import json
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in a JSON data file and convert it to a network of OSM street data"
    )
    parser.add_argument(
        "json_file", help="The JSON file that the GPS Observations are being read from"
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
    G = ox.graph_from_bbox(bbox, network_type="all")

    # Filter for road types bikes can use
    allowed = {
        "cycleway",
        "primary",
        "secondary",
        "tertiary",
        "residential",
        "unclassified",
        "service",
    }
    to_remove = []
    for u, v, k, data in G.edges(keys=True, data=True):
        hw = data.get("highway")
        tags = set(hw) if isinstance(hw, list) else {hw}
        if not (tags & allowed):
            to_remove.append((u, v, k))

    for u, v, k in to_remove:
        G.remove_edge(u, v, k)

    if not G.graph.get("simplified", False):
        G = ox.simplify_graph(G)
    return G


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    args = parse_args()
    print(f"type(args.json_file): {type(args.json_file)}")
    json_path = args.json_file
    points = open_json(json_path)
    bbox = bounding_box_from_json(points)
    print(bbox)
    G = load_road_graph(bbox)

    xs = [pt["long"] for pt in points if "lat" in pt and "long" in pt]
    ys = [pt["lat"] for pt in points if "lat" in pt and "long" in pt]
    print(xs)
    print(ys)
    # Plot the bike-accessible road graph
    fig, ax = ox.plot_graph(
        G, node_size=5, edge_color="blue", show=False, close=False
    )  # Replace with real invocation
    ax.scatter(xs, ys, c="red", s=30, label="GPS Points", zorder=3)

    plt.show()
