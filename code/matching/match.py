import osmnx as ox
import networkx as nx
from math import cos, radians
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Any
import argparse
import json


@dataclass
class CandidateState:
    location: Tuple[float, float]
    candidate_edges: List[Any]
    candidate_distances: List[float]


"""
For each point P in points, query spatial index search within radius, and pull a list of graph edges within the radius
Keep closest K values
Create an edge-emmision-transition tuple (point, edge,em,tr)
Calculate, using paper formulas the values for each tuple
use Viterbi algorithm to generate most probable path


"""


# NOTE: Haversine formula for calculating euclidian distance
def haversine(lat1, lon1, lat2, lon2):
    EARTH_R = 6371000  # Radius of earth
    # convert to radians
    lat1, lon1, lat2, lon2 = map(np.radians, (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2.0) ** 2 + np.cos(lat1) * \
        np.cos(lat2) * np.sin(dlon / 2.0) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return EARTH_R * c


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in a JSON data file and convert it to a network of OSM street data"
    )
    parser.add_argument(
        "json_file", help="The JSON file that the GPS Observations are being read from"
    )
    parser.add_argument(
        "graphml", help="Path to a GraphML file containing saved map data."
    )
    return parser.parse_args()


# NOTE: Given a series of GPX points, gets the N closest edges in a graph G to each location point, within a given radius
def query_spatial_index(G: nx.MultiDiGraph, points, N, radius):
    # Get N closest edges within the radius specificed
    # Approach: use osmnx.distance.nearest_edges to get the nearest edge to each point,
    # and iterate N times, deleting the nearest edge each time, so the next iteration gets the next closest
    # should return the distance, and skip if outside of radius range
    # TEST: figuring out the average distance between point and road of each iteration of loops

    candidate_states = []
    print(G.graph["crs"])
    # create copy to destroy
    G_copy = G.copy()
    # get N closest edges for each point
    for n in range(N):
        nearest_edges, distance = ox.nearest_edges(
            G_copy,
            [p["long"] for p in points],
            [p["lat"] for p in points],
            return_dist=True,
        )

        for idx, p in enumerate(points):
            if (  # If the distance in meters between the first point or the second point of an edge is within the radius
                haversine(
                    p["lat"],
                    p["long"],
                    G.nodes[nearest_edges[idx][0]]["y"],
                    G.nodes[nearest_edges[idx][0]]["x"],
                )
                <= radius
                or haversine(
                    p["lat"],
                    p["long"],
                    G.nodes[nearest_edges[idx][1]]["y"],
                    G.nodes[nearest_edges[idx][1]]["x"],
                )
                <= radius
            ):
                # add edge and distance to location if location exists
                try:
                    candidate_states[idx].candidate_edges.append(
                        nearest_edges[idx])
                    candidate_states[idx].candidate_distances.append(
                        distance[idx])
                    candidate_states[idx].rank = n
                except Exception:
                    # if location doesn't exist, add the new candidate state
                    candidate_states.append(
                        CandidateState(
                            location=(p["long"], p["lat"]),
                            candidate_edges=[nearest_edges[idx]],
                            candidate_distances=[distance[idx]]
                        )
                    )
                # delete closest edge to find next nearest through the next iteration
                if G_copy.has_edge(
                    nearest_edges[idx][0], nearest_edges[idx][1], nearest_edges[idx][2]
                ):
                    G_copy.remove_edge(
                        nearest_edges[idx][0],
                        nearest_edges[idx][1],
                        nearest_edges[idx][2],
                    )
        # Calculate average candidate distance after this iteration
    return candidate_states


def calculate_emmision_prob(candidate_state: CandidateState, sigma: float):
    # Calculate the emission probability (sigma) of a point compared to it's edge, given a noise coefficient sigma:
    return 0


def calculate_transition_probability(edge_prev, edge_curr, beta):
    # Calculate transition probability of two edges occuring, given a distance tolerance beta
    transition: float = 0.0
    return transition


def viterbi_calculate_route(points, candidate_states, emmision, transition, init):
    path = np.zeros(len(points), dtype=int)
    return path


# NOTE: Opens JSON file for loading GPX Data
def open_json(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)
    return data


# NOTE: Takes a graph, and filters the edges down to just what exists in candidate edges, mostly for testing purposes
def filter_graph_from_candidate_states(G, states):
    G2 = nx.MultiDiGraph()
    G2.graph.update(G.graph)
    used_nodes = set()
    for state in states:
        for u, v, k in state.candidate_edges:
            if G.has_edge(u, v, k):
                G2.add_edge(u, v, key=k, **G.get_edge_data(u, v, k))
                used_nodes.add(u)
                used_nodes.add(v)

    for node in used_nodes:
        G2.add_node(node, **G.nodes[node])
    return G2


# NOTE: converts degrees to meters to help with parsing distance
def degrees_to_meters(deg, lat):
    # longitude scaling at given lat
    meters_per_degree = 111_320 * cos(radians(lat))
    return deg * meters_per_degree


def main():
    args = parse_args()
    json_path = args.json_file
    points = open_json(json_path)
    SIGMA = 1.0
    DELTA = 1.0

    # NOTE: TESTING: WE ARE PLOTTING THE GRAPH HERE
    import matplotlib.pyplot as plt

    G = ox.load_graphml(args.graphml)
    N = 7
    states = query_spatial_index(G, points, N, 200)
    for state in states:
        for i, d in enumerate(state.candidate_distances):
            state.candidate_distances[i] = degrees_to_meters(
                d, state.location[1])

    from statistics import mean
    average_distances = {n: [] for n in range(N)}
    for state in states:
        for i, dist in enumerate(state.candidate_distances):
            if i < N:
                average_distances[i].append(state.candidate_distances[i])
    for i in range(N):
        values = average_distances[i]
        avg = mean(values) if values else 0
        print(f"Rank {i}: Average candidate distance = {avg:.2f} meters")
    G2 = filter_graph_from_candidate_states(G, states)
    print(len(states))
    fig2, ax = ox.plot_graph(
        G2, node_size=1, edge_color="blue", show=False, close=False
    )  # Replace with real invocation
    plt.show()


# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


if __name__ == "__main__":
    main()
