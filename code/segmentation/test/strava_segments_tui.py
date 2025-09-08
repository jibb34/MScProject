#!/usr/bin/env python3
"""
strava_full_tui.py

1) Opens browser → Strava OAuth (read + activity:read + activity:read_all)
2) Captures `code` locally
3) Exchanges for access_token
4) Prompts for activity_id(s) or reads from CSV
5) Fetches segment_efforts → prints {"activity_id": {"segment_id": [start, end]}} JSON
"""
import http.server
import socketserver
import threading
import webbrowser
import urllib.parse
import requests
import json
import sys
import argparse
import csv

# === CONFIG ===
CLIENT_ID = 154594 # fill in or prompt
CLIENT_SECRET = "521507674aaf771f50b9263d61eec75066102f12"
REDIRECT_URI = "http://localhost:53682/callback"
PORT = 53682
SCOPES = "read,activity:read,activity:read_all"
TIMEOUT = 300  # seconds to wait for the OAuth callback

# === OAUTH FLOW ===
code_container = {"code": None, "error": None}

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        p = urllib.parse.urlparse(self.path)
        if p.path != "/callback":
            self.send_response(404); self.end_headers(); return
        qs = urllib.parse.parse_qs(p.query)
        if "error" in qs:
            code_container["error"] = qs["error"][0]
        else:
            code_container["code"] = qs.get("code", [None])[0]
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"<html><body><h2>Auth complete. You can close this.</h2></body></html>")

def run_server():
    with socketserver.TCPServer(("127.0.0.1", PORT), Handler) as httpd:
        httpd.timeout = TIMEOUT
        httpd.handle_request()

def get_authorization_code():
    params = {
        "client_id": CLIENT_ID,
        "redirect_uri": REDIRECT_URI,
        "response_type": "code",
        "approval_prompt": "force",
        "scope": SCOPES,
    }
    url = "https://www.strava.com/oauth/authorize?" + urllib.parse.urlencode(params)
    print("\n👉 Opening your browser for Strava authorization...\n")
    webbrowser.open(url, new=1)
    print("If the browser doesn’t open, paste this URL into it:\n")
    print(url + "\n")
    print(f"Waiting up to {TIMEOUT} seconds for the redirect with your code...\n")
    t = threading.Thread(target=run_server, daemon=True)
    t.start()
    t.join(TIMEOUT + 5)
    if code_container["error"]:
        sys.exit(f"OAuth error: {code_container['error']}")
    if not code_container["code"]:
        sys.exit("Timed out waiting for authorization code.")
    return code_container["code"]

def exchange_code_for_token(code):
    resp = requests.post("https://www.strava.com/oauth/token", data={
        "client_id": CLIENT_ID,
        "client_secret": CLIENT_SECRET,
        "code": code,
        "grant_type": "authorization_code",
    }, timeout=30)
    resp.raise_for_status()
    return resp.json()["access_token"]

# === SEGMENT FETCH ===
def fetch_segments(activity_id, token):
    headers = {"Authorization": f"Bearer {token}"}
    url = f"https://www.strava.com/api/v3/activities/{activity_id}"
    r = requests.get(url, headers=headers, params={"include_all_efforts": "true"}, timeout=30)
    if r.status_code == 404:
        r = requests.get(url, headers=headers, timeout=30)
        if r.status_code == 404:
            print(f"⚠️  Activity {activity_id} not found. Skipping.", file=sys.stderr)
            return None
    r.raise_for_status()
    data = r.json()
    segs = {}
    for e in data.get("segment_efforts", []):
        seg = e.get("segment", {})
        sid = seg.get("id")
        start = seg.get("start_latlng") or []
        end   = seg.get("end_latlng")   or []
        if sid and len(start) >= 2 and len(end) >= 2:
            segs[str(sid)] = [
                {"lat": start[0], "lon": start[1]},
                {"lat": end[0],   "lon": end[1]},
            ]
    return segs

# === CSV INPUT PARSE ===
import os

def read_activity_ids(csv_path, verbose=False):
    if not os.path.isfile(csv_path):
        print(f"❌ CSV file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)
    ids = []
    with open(csv_path, newline='') as f:
        reader = csv.reader(f)
        for row_num, row in enumerate(reader, start=1):
            if verbose:
                print(f"[debug] row {row_num}: {row}", file=sys.stderr)
            for v in row:
                val = v.strip()
                if val and val.lower() != 'activity_id':  # skip header
                    ids.append(val)
    if verbose:
        print(f"[debug] parsed activity IDs: {ids}", file=sys.stderr)
    return ids

# === MAIN ===


# === MAIN ===

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', help='Path to CSV file containing activity IDs')
    parser.add_argument('--verbose', action='store_true', help='Enable debug output')
    parser.add_argument('--output', '-o', help='Output JSON file path (defaults to stdout)')
    args = parser.parse_args()

    global CLIENT_ID, CLIENT_SECRET
    if not CLIENT_ID:
        CLIENT_ID = input("Client ID: ").strip()
        CLIENT_SECRET = input("Client Secret: ").strip()

    code = get_authorization_code()
    token = exchange_code_for_token(code)
    if args.verbose:
        print("✅ Got access token.", file=sys.stderr)

    if args.csv:
        activity_ids = read_activity_ids(args.csv, verbose=args.verbose)
    else:
        single = input("Activity ID(s) (comma-separated): ").strip()
        activity_ids = [x.strip() for x in single.split(',') if x.strip()]
        if args.verbose:
            print(f"[debug] parsed activity IDs: {activity_ids}", file=sys.stderr)

    all_segments = {}
    for aid in activity_ids:
        if args.verbose:
            print(f"🔍 Fetching segments for activity {aid}...", file=sys.stderr)
        segs = fetch_segments(aid, token)
        if segs is not None:
            all_segments[aid] = segs

    output_json = json.dumps(all_segments, indent=2)
    if args.output:
        with open(args.output, 'w') as f:
            f.write(output_json)
        print(f"✅ Wrote results to {args.output}")
    else:
        print(output_json)

if __name__ == "__main__":
    main()

