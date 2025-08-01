import matplotlib.pyplot as plt
import osmnx as ox
import networkx as nx
from math import cos, radians
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Any
import pickle
import os
import argparse
import json
import matplotlib.cm as cm
import matplotlib.colors as mcolors


@dataclass
class CandidateState:
    location: Tuple[float, float]
    candidate_edges: List[Any]
    candidate_distances: List[float]
    emission: List[float]
    transition: List[List[float]]
    order: int
    # list of lists: each edge has a
    # list of probabilities refering to likelyhood
    # that the corresponding edge in p+1 is a transition edge


"""
For each point P in points, query spatial index search within radius,
and pull a list of graph edges within the radius
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
def query_spatial_index(G: nx.MultiDiGraph, points, N):
    # Get N closest edges within the radius specificed
    # Approach: use osmnx.distance.nearest_edges to get the nearest edge to each point,
    # and iterate N times, deleting the nearest edge each time, so the next iteration gets the next closest
    # should return the distance, and skip if outside of radius range
    # TEST: figuring out the average distance between point and road of each iteration of loops

    candidate_states = [
        # Initialize array with empty states for each point
        CandidateState(location=(p["long"], p["lat"]),
                       candidate_edges=[],
                       candidate_distances=[],
                       emission=[], transition=[], order=N)
        for p in points]
    # create copy to destroy and extract points lat/longs
    G_copy = G.copy()
    longs = [p["long"] for p in points]
    lats = [p["lat"] for p in points]

    # get N closest edges for each point
    for _ in range(N):
        nearest_edges, deg_dists = ox.nearest_edges(
            G_copy, longs, lats, return_dist=True
        )

        for idx, (edge, deg_dist) in enumerate(zip(nearest_edges, deg_dists)):
            dist_m = degrees_to_meters(
                deg_dist, lats[idx])  # Convert to meters
            if True:  # dist_m <= radius:  # if distance is within bounds

                # add edge and distance to location if location exists
                cs = candidate_states[idx]
                cs.candidate_edges.append(edge)
                cs.candidate_distances.append(dist_m)

            else:  # if values fall outside range, mark them as null and set distance to infinity
                cs.candidate_edges.append(None)
                cs.candidate_distances.append(float("inf"))

            # delete closest edge to find next nearest through the next iteration
            u, v, k = edge
            if G_copy.has_edge(u, v, k):
                G_copy.remove_edge(u, v, k)
    return candidate_states


def calculate_emmision_prob(candidate_states, sigma):
    # Calculate the emission probability (sigma) of a point compared to it's edge, given a noise coefficient sigma:
    coefficient = 1 / (np.sqrt(2 * np.pi) * sigma)
    for state in candidate_states:
        for i, d in enumerate(state.candidate_distances):
            if d == float("inf"):
                state.emission.append(0.0)
            else:
                power = -0.5 * (d/sigma) ** 2
                result = coefficient * np.e ** power
                state.emission.append(result)


def calculate_transition_probability(G, candidate_states, beta):
    # NOTE:  Calculate transition probability of two edges occuring,
    #        given a distance tolerance beta
    for i, _ in enumerate(candidate_states):
        # If last entry in state list, probability is zero
        if (i == len(candidate_states)-1):
            # Return empty list
            return

        # For each Candidate edge in Candidate State
        for edge in candidate_states[i].candidate_edges:
            for next_edge in candidate_states[i+1].candidate_edges:
                if ((edge is None) or (next_edge is None)):
                    prob = 0.0
                    continue
                # Get route distance
                route_distance = shortest_route_distance_between_edges(
                    G, candidate_states[i].location, candidate_states[i+1].location, edge, next_edge)
                # Get Haversine distance:
                haversine_distance = haversine(candidate_states[i].location[1], candidate_states[i].location[0],
                                               candidate_states[i+1].location[1], candidate_states[i+1].location[0])
                # Difference between the two: value is infinity, set prob to small numbere
                if (route_distance == float("inf")):
                    prob = 1e-10  # Small probability for fallback
                else:
                    delta = np.abs(haversine_distance - route_distance)

                    prob = (1/beta) * np.e ** -(delta/beta)
                candidate_states[i].transition.append(prob)

        print(candidate_states[i].transition)


# Just sets the initial state array
def get_init_state_dist(candidate_state):
    init = np.zeros(len(candidate_state.candidate_edges))
    min_index = np.argmin(candidate_state.candidate_distances)
    init[min_index] = 1.0
    return init


def viterbi_calculate_route(candidate_states):
    print("t=0 emissions:", candidate_states[0].emission)

    T = len(candidate_states)           # number of GPS points
    # number of candidates per point (constant)
    N = candidate_states[0].order

    # 1) Allocate the DP tables
    #    delta[t][j] = best probability of any path ending in candidate j at time t
    #    psi[t][j]   = index i of the best predecessor for state j at time t
    delta = [[0.0] * N for _ in range(T)]
    psi = [[0] * N for _ in range(T)]

    # 2) Initialization (t = 0)
    #    We simply start with the emission probs for the first GPS point
    for j in range(N):
        delta[0][j] = candidate_states[0].emission[j]
        psi[0][j] = -1   # no predecessor

    # 3) Recursion (t = 1 … T-1)
    for t in range(1, T):
        prev_cs = candidate_states[t-1]
        curr_cs = candidate_states[t]

        for j in range(N):
            best_prob = 0.0
            best_i = 0

            # Try every possible predecessor i → j
            for i in range(N):
                trans_p = prev_cs.transition[i][j]  # P(s_t=j | s_{t-1}=i)
                prob = delta[t-1][i] * trans_p

                if prob > best_prob:
                    best_prob = prob
                    best_i = i

            # Multiply by how well candidate j explains the observation
            delta[t][j] = best_prob * curr_cs.emission[j]
            psi[t][j] = best_i

    # 4) Termination: pick the best final state
    last_index = max(range(N), key=lambda j: delta[T-1][j])
    path = [0] * T
    path[T-1] = last_index

    # 5) Backtrace: walk psi backwards
    for t in range(T-1, 0, -1):
        path[t-1] = psi[t][path[t]]

    return path


def shortest_route_distance_between_edges(G, point1, point2, edge1, edge2):
    node1 = edge_to_node(edge1, point1, G)
    node2 = edge_to_node(edge2, point2, G)
    try:
        distance = nx.shortest_path_length(G, node1, node2, weight="length")
        return distance
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        # Return infinity if no connection found
        return float("inf")


def edge_to_node(edge, gps_point, G):
    # return edge[0]
    # NOTE: Converts an edge to the closest node to a gps point
    u, v, _ = edge
    # Get lat/long from graph
    lat1, lon1 = G.nodes[u]["y"], G.nodes[u]["x"]
    lat2, lon2 = G.nodes[v]["y"], G.nodes[v]["x"]
    # finds closest point
    d_u = haversine(gps_point[1], gps_point[0], lat1, lon1)
    d_v = haversine(gps_point[1], gps_point[0], lat2, lon2)
    # returns closest node
    return u if d_u < d_v else v


# NOTE: Opens JSON file for loading GPX Data
def open_json(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)
    return data


# NOTE: Takes a graph, and filters the edges down to just what
#       exists in candidate edges, mostly for testing purposes
def filter_graph_from_candidate_states(G, states):
    G2 = nx.MultiDiGraph()
    G2.graph.update(G.graph)
    used_nodes = set()
    for state in states:
        for edge in state.candidate_edges:
            try:
                u, v, k = edge
            except TypeError:
                continue
            if G.has_edge(u, v, k):
                G2.add_edge(u, v, key=k, **G.get_edge_data(u, v, k))
                used_nodes.add(u)
                used_nodes.add(v)

    for node in used_nodes:
        G2.add_node(node, **G.nodes[node])
    return G2


# NOTE: converts degrees to meters to help with parsing distance
def degrees_to_meters(deg, lat):
    # longitude scaling at given lat, approximate, and works well for smaller distances
    # (which nearest candidates should be)
    meters_per_degree = 111_320 * cos(radians(lat))
    return deg * meters_per_degree


def plot_emissions_on_map(points, G2, candidate_states):
    # 1) Compute max emission per edge
    edge_emission = {}
    for cs in candidate_states:
        for edge, e in zip(cs.candidate_edges, cs.emission):
            edge_emission[edge] = max(edge_emission.get(edge, 0), e)

    # 2) Normalize and colormap
    probs = list(edge_emission.values())
    norm = mcolors.Normalize(vmin=min(probs), vmax=max(probs))
    cmap = cm.get_cmap("coolwarm")

    # 3) Build a color for every edge in G2
    color_list = []
    for u, v, k in G2.edges(keys=True):
        p = edge_emission.get((u, v, k), 0.0)
        color_list.append(mcolors.to_hex(cmap(norm(p))))

    # 4) Plot
    fig, ax = ox.plot_graph(
        G2,
        edge_color=color_list,
        node_size=1,
        edge_linewidth=2,
        show=False,
        close=False
    )

    xs = [pt["long"] for pt in points if "lat" in pt and "long" in pt]
    ys = [pt["lat"] for pt in points if "lat" in pt and "long" in pt]
    ax.scatter(xs, ys, c="red", s=30, label="GPS Points", zorder=3)
    ax.set_title("Emission probabilities (red→blue)")
    plt.show()


def main():
    args = parse_args()
    json_path = args.json_file
    points = open_json(json_path)
    SIGMA = 1.0
    BETA = 1.0

    G = ox.load_graphml(args.graphml)
    N = 15
    # Check for cache file, if exists, load from there, else re-calculate
    CACHE_FILE = "states.pkl"
    if os.path.exists(CACHE_FILE):
        with open(CACHE_FILE, "rb") as f:
            states = pickle.load(f)
    else:
        states = query_spatial_index(G, points, N)
        import random
        calculate_emmision_prob(states, SIGMA)
        # print(states[random.randint(0, len(points)-1)].emission)

        # TEST:
        # from statistics import mean
        # average_distances = {n: [] for n in range(N)}
        # for state in states:
        #     for i, dist in enumerate(state.candidate_distances):
        #         if i < N:
        #             average_distances[i].append(state.candidate_distances[i])
        # for i in range(N):
        #     values = average_distances[i]
        #     avg = mean(values) if values else 0
        #     print(f"Rank {i}: Average candidate distance = {avg:.2f} meters")
        # NOTE: ~~ MOST TIME CONSUMING PART ~~
        G2 = filter_graph_from_candidate_states(G, states)
        # calculate_transition_probability(G2, states, BETA)

    with open(CACHE_FILE, "wb") as f:
        pickle.dump(states, f)
    c = 0
    v = 0
    for state in states:
        if (len(state.transition) != N):
            v += 0
        for t in state.transition:
            if (len(t) != N):
                c += 1
    # print("transition[0] row-0:", states[0].transition[0])
    # path = viterbi_calculate_route(states)
    import matplotlib.pyplot as plt
    G2 = filter_graph_from_candidate_states(G, states)

    plot_emissions_on_map(points, G2, states)


# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

if __name__ == "__main__":
    main()
