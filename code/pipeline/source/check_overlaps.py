import os
import json
import polyline
import math
from collections import Counter


def flatten_nodes_robust(legs, max_overlap_check=16):
    if isinstance(legs, dict):
        legs = [legs]
    out = []
    for leg in legs or []:
        seq = [int(n) for n in ((leg or {}).get(
            "annotation") or {}).get("nodes", [])]
        if not seq:
            continue
        if not out:
            out.extend(seq)
            continue
        max_k = min(len(out), len(seq), max_overlap_check)
        K = 0
        for k in range(max_k, 0, -1):
            if out[-k:] == seq[:k]:
                K = k
                break
        for n in seq[K:]:
            if not out or n != out[-1]:
                out.append(n)
    dedup = []
    for n in out:
        if not dedup or n != dedup[-1]:
            dedup.append(n)
    return dedup


def overlap_len(a, b, max_check=200):
    m = min(len(a), len(b), max_check)
    for k in range(m, 0, -1):
        if a[-k:] == b[:k]:
            return k
    return 0


def decode_coords(match):
    g = match.get("geometry")
    if isinstance(g, str):
        try:
            return polyline.decode(g, precision=6)  # [(lat,lon)]
        except Exception:
            return []
    if isinstance(g, dict) and g.get("type") == "LineString":
        return [(lat, lon) for lon, lat in g.get("coordinates", [])]
    return []


def haversine(p, q):
    if not p or not q:
        return None
    R = 6371000
    lat1, lon1 = map(math.radians, p)
    lat2, lon2 = map(math.radians, q)
    dlat = lat2-lat1
    dlon = lon2-lon1
    a = math.sin(dlat/2)**2+math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(a))


def main(chunks_dir):
    files = sorted([os.path.join(chunks_dir, f)
                   for f in os.listdir(chunks_dir) if f.endswith(".json")])
    spines = []
    geoms = []
    for fp in files:
        data = json.load(open(fp))
        m = (data.get("matchings") or [{}])[0]
        legs = m.get("legs") or []
        nodes = data.get("route_nodes") or flatten_nodes_robust(legs)
        spines.append(nodes)
        geoms.append(decode_coords(m))

    ks = []
    zero_cases = []
    for i in range(len(files)-1):
        a, b = spines[i], spines[i+1]
        k = overlap_len(a, b, max_check=200)
        ks.append(k)
        if k == 0:
            # collect a small sample
            tail = a[-10:] if a else []
            head = b[:10] if b else []
            # optional geometry sanity
            pa = geoms[i][-1] if geoms[i] else None
            pb = geoms[i+1][0] if geoms[i+1] else None
            d = haversine(pa, pb) if (pa and pb) else None
            zero_cases.append(
                (files[i], files[i+1], len(a), len(b), tail, head, d))

    from statistics import mean
    print(f"pairs={len(files)-1}")
    print("k histogram:", Counter([min(10, k)
          for k in ks]))  # bucket 10+ together
    if ks:
        print("mean k:", round(mean(ks), 2), "min:", min(ks), "max:", max(ks))

    print("\n=== examples with k=0 (up to 5) ===")
    for row in zero_cases[:5]:
        fA, fB, la, lb, tail, head, d = row
        print(f"- {os.path.basename(fA)} -> {os.path.basename(fB)
                                             }  lenA={la} lenB={lb} seam_dist(m)={None if d is None else round(d, 1)}")
        print("  tailA:", tail)
        print("  headB:", head)


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python check_overlaps.py <chunks_dir>")
        raise SystemExit(2)
    main(sys.argv[1])
