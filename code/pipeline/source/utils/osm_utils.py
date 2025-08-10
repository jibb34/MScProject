import sqlite3
from itertools import groupby
import polyline
import heapq


def extract_coords(matching, label=""):
    geometry = matching.get("geometry")
    # If polyline:
    if isinstance(geometry, str):
        try:
            return polyline.decode(geometry, precision=6)
        except Exception as e:
            print(f"[ERROR] Failed to decode polyline from {label}")
            return []
    elif isinstance(geometry, dict) and geometry.get("type") == "LineString":
        return [(lat, lon) for lon, lat in geometry["coordinates"]]
    else:
        print(f"[ERROR] Unsupported geometry format in {label}")
        return []


def _overlap_len(a_nodes, b_nodes, max_check=50):
    """Largest k where last k of A == first k of B. Basically check the amount of nodes overlapping with previous
    node"""
    max_k = min(len(a_nodes), len(b_nodes), max_check)
    for k in range(max_k, 0, -1):
        if a_nodes[-k:] == b_nodes[:k]:
            return k
    return 0


def _flatten_nodes(legs, max_overlap_check=16):
    """Flatten legs' annotation.nodes into one list, removing exact seam overlap."""
    if isinstance(legs, dict):
        legs = [legs]
    out = []
    for leg in legs or []:
        ann = (leg or {}).get("annotation") or {}
        seq = [int(n) for n in (ann.get("nodes") or [])]
        if not seq:
            continue
        if not out:
            out.extend(seq)
            continue
        # find largest K such that out[-K:] == seq[:K]
        max_k = min(len(out), len(seq), max_overlap_check)
        K = 0
        for k in range(max_k, 0, -1):
            if out[-k:] == seq[:k]:
                K = k
                break
        for n in seq[K:]:
            if not out or n != out[-1]:  # guard against accidental dup
                out.append(n)
    # paranoia sweep
    dedup = []
    for n in out:
        if not dedup or n != dedup[-1]:
            dedup.append(n)
    return dedup


def _load_dist(legs):
    """Concatenate per-edge metrics (distance) across legs if present."""
    dist = []
    for leg in legs or []:
        ann = (leg or {}).get("annotation") or {}
        dist.extend(ann.get("distance") or [])
    return dist


def _edge_lookup(cur, u, v):
    """
    Return (way_id, dir, dist_m)
      dir = +1 if u->v in DB, -1 if only v->u exists, 0 if unresolved
      dist_m comes from edges.len_m if present, else None
    """
    u = int(u)
    v = int(v)
    # try forward
    row = cur.execute(
        "SELECT way_id, len_m FROM edges WHERE u=? AND v=? LIMIT 1", (u, v)).fetchone()
    if row:
        wid, dlen = int(row[0]), (None if row[1] is None else float(row[1]))
        return wid, +1, dlen
    # try reverse
    row = cur.execute(
        "SELECT way_id, len_m FROM edges WHERE u=? AND v=? LIMIT 1", (v, u)).fetchone()
    if row:
        wid, dlen = int(row[0]), (None if row[1] is None else float(row[1]))
        return wid, -1, dlen
    return None, 0, None


def _runs_from_chunk(match_obj, cur, with_metrics=False):
    """
    Convert one matching's legs into directional way runs.
    Returns: (runs, total_edges)
    """
    legs = match_obj.get("legs", [])
    nodes = _flatten_nodes(legs)
    total_edges = max(0, len(nodes) - 1)

    # Optional metrics aligned per edge
    distances = []
    if with_metrics:
        distances = _load_dist(legs)

    edges = []  # list of (way_id, dir, dist)
    for i, (u, v) in enumerate(zip(nodes[:-1], nodes[1:])):
        u, v = int(u), int(v)
        if u == v:
            continue
        wid, dir, dist = _edge_lookup(cur, u, v)
        edges.append((wid, dir, dist))

    # Compress consecutive identical (way_id, dir)
    runs = []
    for (wid, d), group in groupby(edges, key=lambda e: (e[0], e[1])):
        g = list(group)
        run = {"way_id": wid, "dir": d, "edge_count": len(g)}
        if with_metrics:
            length_m = sum((x[2] or 0.0) for x in g)
            run.update({
                "length_m": length_m,
            })
        runs.append(run)

    return runs, total_edges, nodes  # nodes returned for overlap calc


def _trim_runs_prefix(runs, drop_edges):
    """Trim the first 'drop_edges' edges from a run list in-place and return the remaining list."""
    i = 0
    while i < len(runs) and drop_edges > 0:
        take = min(runs[i]["edge_count"], drop_edges)
        # proportionally trim metrics if present (approximate)
        if "length_m" in runs[i] and runs[i]["edge_count"] > 0:
            before = runs[i]["edge_count"]
            keep = before - take
            factor = (keep / before) if before else 0.0
            for k in ("length_m",):
                if k in runs[i] and runs[i][k] is not None:
                    runs[i][k] *= factor
        runs[i]["edge_count"] -= take
        if runs[i]["edge_count"] == 0:
            i += 1
        drop_edges -= take
    return runs[i:]


def merge_runs(A, B, overlap_edges):
    """Merge two directional run lists, trimming B by 'overlap_edges' (k-1)."""
    if overlap_edges > 0:
        B = _trim_runs_prefix(B, overlap_edges)

    if not A:
        return B
    if not B:
        return A

    # Coalesce boundary if same (way_id, dir)
    last, first = A[-1], B[0]
    if last["way_id"] == first["way_id"] and last["dir"] == first["dir"]:
        last["edge_count"] += first["edge_count"]
        if "length_m" in last and "length_m" in first:
            last["length_m"] = (last.get("length_m") or 0) + \
                (first.get("length_m") or 0)
        B = B[1:]

    return A + B


def _neighbors(cur, u, use_len):
    """
    Always return a list (possibly empty) of (v, way_id, weight).
    """
    u = int(u)
    try:
        if use_len:
            rows = cur.execute(
                "SELECT v, way_id, COALESCE(len_m,1.0) FROM edges WHERE u=?",
                (u,)
            ).fetchall()
            return [(int(v), int(wid), float(w)) for (v, wid, w) in rows]
        else:
            rows = cur.execute(
                "SELECT v, way_id FROM edges WHERE u=?",
                (u,)
            ).fetchall()
            return [(int(v), int(wid), 1.0) for (v, wid) in rows]
    except sqlite3.Error:
        # Graceful fallback: no neighbors rather than None
        return []


def shortest_connector_nodes_db(cur, start_u, goal_v, max_expansions=50000):
    """
    Dijkstra on directed edges table. Weight = len_m if present, else 1 per edge.
    Returns a node list [u,...,v] or [] if not found.
    """
    start_u = int(start_u)
    goal_v = int(goal_v)
    use_len = _table_has_column(cur, "edges", "len_m")
    Q = [(0.0, start_u)]
    dist = {start_u: 0.0}
    prev = {}
    expanded = 0

    while Q:
        cost, u = heapq.heappop(Q)
        if u == goal_v:
            break
        if cost > dist.get(u, float("inf")):
            continue
        for v, _wid, w in _neighbors(cur, u, use_len):
            nc = cost + (w if use_len else 1.0)
            if nc < dist.get(v, float("inf")):
                dist[v] = nc
                prev[v] = u
                heapq.heappush(Q, (nc, v))
        expanded += 1
        if expanded > max_expansions:
            return []

    if goal_v not in dist:
        return []
    # reconstruct
    path = [goal_v]
    while path[-1] != start_u:
        path.append(prev[path[-1]])
    path.reverse()
    return path


def nodes_to_runs_for_slice(
    nodes, cur, edge_lookup_fn,
    with_metrics=False, distances=None, prefer_db_len=True
):
    """
    Map a small connector node list to (way_id, dir) runs.
    - with_metrics: include length_m by summing per-edge distances
    - distances: optional OSRM per-edge list to use when DB len_m is absent
    - prefer_db_len: if True, use DB len_m when available
    """
    edges = []
    di = 0  # index into optional OSRM distances

    for u, v in zip(nodes[:-1], nodes[1:]):
        u = int(u)
        v = int(v)
        if u == v:
            continue  # skip degenerate self-loops

        info = edge_lookup_fn(cur, u, v)
        # tolerate both 2- and 3-tuple return shapes
        if isinstance(info, tuple) and len(info) == 3:
            wid, d, db_len = info
        else:
            wid, d = info
            db_len = None

        dist = None
        if with_metrics:
            if prefer_db_len and db_len is not None:
                dist = float(db_len)
            elif distances is not None and di < len(distances):
                dist = float(distances[di])

        edges.append((wid, d, dist))
        di += 1

    runs = []
    for (wid, d), group in groupby(edges, key=lambda e: (e[0], e[1])):
        g = list(group)
        run = {"way_id": wid, "dir": d, "edge_count": len(g)}
        if with_metrics:
            run["length_m"] = sum((x[2] or 0.0) for x in g)
        runs.append(run)

    return runs


def _table_has_column(cur, table, col):
    return col in [r[1] for r in cur.execute(f"PRAGMA table_info({table})").fetchall()]
