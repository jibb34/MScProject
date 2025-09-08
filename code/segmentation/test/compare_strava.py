import argparse
import json
import math
import os
import urllib.request
import urllib.parse
from typing import Dict, List


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return distance in meters between two WGS84 coordinates."""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def fetch_access_token(client_id: str, client_secret: str, refresh_token: str) -> str:
    """Retrieve a Strava API access token using OAuth refresh workflow."""
    data = urllib.parse.urlencode(
        {
            "client_id": client_id,
            "client_secret": client_secret,
            "refresh_token": refresh_token,
            "grant_type": "refresh_token",
        }
    ).encode()
    req = urllib.request.Request(
        "https://www.strava.com/api/v3/oauth/token", data=data, method="POST"
    )
    with urllib.request.urlopen(req) as resp:
        payload = json.load(resp)
    token = payload.get("access_token")
    if not token:
        raise RuntimeError("Failed to obtain Strava access token")
    return token


def fetch_segment(segment_id: int, token: str) -> Dict[str, object]:
    """Fetch a segment definition from Strava's /segments endpoint."""
    url = f"https://www.strava.com/api/v3/segments/{segment_id}"
    headers = {"Authorization": f"Bearer {token}"}
    req = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(req) as resp:
        return json.load(resp)


def fetch_strava_segments(activity_id: str, token: str) -> List[Dict[str, object]]:
    """Fetch segment data for an activity via Strava's API.

    Each segment is retrieved from the dedicated ``/segments`` endpoint so the
    output mirrors the structure returned by Strava, including ``start_latlng``
    and ``end_latlng`` coordinates.
    """
    url = f"https://www.strava.com/api/v3/activities/{activity_id}?include_all_efforts=true"
    headers = {"Authorization": f"Bearer {token}"}
    req = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(req) as resp:
        data = json.load(resp)

    unique_ids = {
        effort.get("segment", {}).get("id")
        for effort in data.get("segment_efforts", [])
        if effort.get("segment")
    }
    segments: List[Dict[str, object]] = []
    for seg_id in unique_ids:
        if seg_id is None:
            continue
        seg = fetch_segment(seg_id, token)
        start = seg.get("start_latlng") or [None, None]
        end = seg.get("end_latlng") or [None, None]
        if None in start or None in end:
            continue
        segments.append(
            {
                "id": seg.get("id"),
                "name": seg.get("name"),
                "start_latlng": start,
                "end_latlng": end,
            }
        )
    return segments


def load_engine_segments(path: str) -> List[Dict[str, Dict[str, float]]]:
    """Load segment definitions from segmentation engine JSON output."""
    with open(path) as f:
        data = json.load(f)
    segments = []
    for s in data.get("segments", []):
        start = s.get("start") or {}
        end = s.get("end") or {}
        if "lat" in start and "lon" in start and "lat" in end and "lon" in end:
            segments.append(
                {
                    "start": {"lat": start["lat"], "lon": start["lon"]},
                    "end": {"lat": end["lat"], "lon": end["lon"]},
                }
            )
    return segments


def compare_segments(
    strava_segments: List[Dict[str, object]],
    engine_segments: List[Dict[str, Dict[str, float]]],
) -> List[Dict[str, object]]:
    """Compare Strava segments against engine segments by start/end proximity."""
    results: List[Dict[str, object]] = []
    for seg in strava_segments:
        start = seg.get("start_latlng") or [None, None]
        end = seg.get("end_latlng") or [None, None]
        if None in start or None in end:
            continue
        best: Dict[str, object] | None = None
        for cand in engine_segments:
            start_diff = haversine(
                start[0], start[1], cand["start"]["lat"], cand["start"]["lon"]
            )
            end_diff = haversine(end[0], end[1], cand["end"]["lat"], cand["end"]["lon"])
            total = start_diff + end_diff
            if best is None or total < best["total"]:
                best = {
                    "start_diff": start_diff,
                    "end_diff": end_diff,
                    "total": total,
                    "segment": cand,
                }
        if best is not None:
            results.append({"strava_segment": seg, "best_match": best})
    return results


def main():
    ap = argparse.ArgumentParser(
        description="Compare Strava segments to engine output."
    )
    ap.add_argument("activity_id", help="Strava activity ID")
    ap.add_argument("engine_json", help="Path to engine segment JSON")
    ap.add_argument(
        "--token",
        default=None,
        help="Strava API access token (or set STRAVA_TOKEN env variable)",
    )
    ap.add_argument("--client-id", default=None, help="Strava client ID")
    ap.add_argument("--client-secret", default=None, help="Strava client secret")
    ap.add_argument("--refresh-token", default=None, help="Strava refresh token")

    args = ap.parse_args()
    token = args.token or os.environ.get("STRAVA_TOKEN")
    if not token:
        client_id = args.client_id or os.environ.get("STRAVA_CLIENT_ID")
        client_secret = args.client_secret or os.environ.get("STRAVA_CLIENT_SECRET")
        refresh_token = args.refresh_token or os.environ.get("STRAVA_REFRESH_TOKEN")
        if client_id and client_secret and refresh_token:
            token = fetch_access_token(client_id, client_secret, refresh_token)
    if not token:
        raise SystemExit(
            "Strava API token required via --token, STRAVA_TOKEN env, or OAuth refresh args"
        )

    strava_segs = fetch_strava_segments(args.activity_id, token)
    engine_segs = load_engine_segments(args.engine_json)
    comparison = compare_segments(strava_segs, engine_segs)
    print(json.dumps(comparison, indent=2))


if __name__ == "__main__":
    main()
