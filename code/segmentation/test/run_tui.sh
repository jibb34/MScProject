#!/usr/bin/env bash
# strava_min_setup.sh — minimal deps (bash + python3)
# 1) make venv
# 2) (first run) fetch refresh token via local OAuth
# 3) run TUI to export segments JSON
# 4) deactivate venv on exit
set -euo pipefail

APP_DIR="${APP_DIR:-$PWD}"
VENV_DIR="${VENV_DIR:-$APP_DIR/.venv}"
ENV_FILE="${ENV_FILE:-$APP_DIR/.strava.env}"
TUI_FILE="${TUI_FILE:-$APP_DIR/strava_segments_tui.py}"
OAUTH_HELPER="${OAUTH_HELPER:-$APP_DIR/strava_oauth_helper.py}"
REDIRECT_URI="${REDIRECT_URI:-http://localhost:53682/callback}"

deactivate_venv() { if [[ -n "${VIRTUAL_ENV-}" ]]; then deactivate || true; fi; }
trap deactivate_venv EXIT

need() { command -v "$1" >/dev/null 2>&1 || {
  echo "Missing: $1" >&2
  exit 1
}; }
need python3

# --- venv ---
if [[ ! -d "$VENV_DIR" ]]; then
  echo "[*] Creating venv at $VENV_DIR"
  python3 -m venv "$VENV_DIR"
fi
# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"
VENV_PY="$VENV_DIR/bin/python"
"$VENV_PY" - <<'PY' || { "$VENV_PY" -m pip install --upgrade pip && pip install requests; }
try:
    import requests  # noqa
except Exception:
    raise SystemExit(1)
PY

# --- load .strava.env if present ---
if [[ -f "$ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$ENV_FILE"
fi

# --- write TUI script if missing (very small, only needs requests) ---
if [[ ! -f "$TUI_FILE" ]]; then
  cat >"$TUI_FILE" <<'PY'
#!/usr/bin/env python3
# strava_segments_tui.py — exports: { "Segment": [ {"lat":..,"lon":..}, {"lat":..,"lon":..} ], ... }
import json, os, sys, time, getpass
from typing import Any, Dict, Optional, Tuple
import requests

API_BASE = "https://www.strava.com/api/v3"
OAUTH_TOKEN_URL = "https://www.strava.com/oauth/token"
ENV_FILE = ".strava.env"

def prompt_default(prompt: str, default: Optional[str]=None, secret=False) -> str:
    label = prompt + (f" [{default}]" if default else "") + ": "
    val = getpass.getpass(label) if secret else input(label)
    return default if (default is not None and not val) else val

def load_env(path: str) -> Dict[str,str]:
    d = {}
    if not os.path.exists(path): return d
    for line in open(path):
        line=line.strip()
        if not line or line.startswith("#") or "=" not in line: continue
        k,v=line.split("=",1); d[k.strip()]=v.strip()
    return d

def save_env(path: str, env: Dict[str,str]) -> None:
    with open(path,"w") as f:
        f.write("# Local Strava credentials (keep private!)\n")
        for k,v in env.items(): f.write(f"{k}={v}\n")
    print(f"Saved {path}")

def get_access_token(cid: str, csec: str, rtok: str) -> str:
    r = requests.post(OAUTH_TOKEN_URL, data={
        "client_id": cid, "client_secret": csec,
        "grant_type": "refresh_token", "refresh_token": rtok
    }, timeout=30)
    if r.status_code != 200:
        sys.exit(f"Failed to refresh token: {r.status_code} {r.text}")
    tok = r.json().get("access_token")
    if not tok: sys.exit("No access_token in response.")
    return tok

def get_activity(aid: int, tok: str) -> Dict[str,Any]:
    r = requests.get(f"{API_BASE}/activities/{aid}",
        headers={"Authorization": f"Bearer {tok}"},
        params={"include_all_efforts": "true"}, timeout=30)
    if r.status_code==404: sys.exit(f"Activity {aid} not found.")
    if r.status_code==401: sys.exit("Unauthorized (401). Check scopes.")
    r.raise_for_status(); return r.json()

def get_segment(sid: int, tok: str):
    r = requests.get(f"{API_BASE}/segments/{sid}",
        headers={"Authorization": f"Bearer {tok}"}, timeout=30)
    if r.status_code==404: return None
    try: r.raise_for_status()
    except Exception: return None
    return r.json()

def extract_latlng(seg: Dict[str,Any]):
    s = seg.get("start_latlng") or []; e = seg.get("end_latlng") or []
    return (s[0] if len(s)>0 else None, s[1] if len(s)>1 else None,
            e[0] if len(e)>0 else None, e[1] if len(e)>1 else None)

def build_output(efforts, tok: str, sleep_s=0.1):
    out = {}
    for ef in efforts or []:
        seg = (ef or {}).get("segment") or {}
        sid = seg.get("id"); name = seg.get("name", f"segment_{sid}")
        s_lat,s_lon,e_lat,e_lon = extract_latlng(seg)
        if sid and (None in (s_lat,s_lon,e_lat,e_lon)):
            full = get_segment(sid, tok)
            if full: s_lat,s_lon,e_lat,e_lon = extract_latlng(full)
            time.sleep(sleep_s)
        if None not in (s_lat,s_lon,e_lat,e_lon):
            out[name] = [ {"lat": float(s_lat), "lon": float(s_lon)},
                          {"lat": float(e_lat), "lon": float(e_lon)} ]
    return out

def main():
    print("== Strava Segment Export ==")
    env = load_env(ENV_FILE)
    cid  = prompt_default("Client ID", env.get("STRAVA_CLIENT_ID"))
    csec = prompt_default("Client Secret", env.get("STRAVA_CLIENT_SECRET"), secret=True)
    rtok = prompt_default("Refresh Token", env.get("STRAVA_REFRESH_TOKEN"), secret=True)
    act  = prompt_default("Activity ID (e.g., 14290917931)", env.get("STRAVA_ACTIVITY_ID"))
    outp = prompt_default("Output JSON path", env.get("STRAVA_OUTPUT_JSON", f"segments_{act or 'activity'}.json"))
    if prompt_default("Save credentials to .strava.env? (y/N)", "N").lower()=="y":
        save_env(ENV_FILE, {
            "STRAVA_CLIENT_ID": cid,
            "STRAVA_CLIENT_SECRET": csec,
            "STRAVA_REFRESH_TOKEN": rtok,
            "STRAVA_ACTIVITY_ID": act or "",
            "STRAVA_OUTPUT_JSON": outp or "",
        })
    try: aid = int(act)
    except: sys.exit("Activity ID must be an integer.")
    tok = get_access_token(cid, csec, rtok)
    activity = get_activity(aid, tok)
    efforts = activity.get("segment_efforts", []) or []
    data = build_output(efforts, tok)
    with open(outp,"w") as f: json.dump(data, f, indent=2)
    print(f"wrote {len(data)} segments -> {outp}")
if __name__=="__main__": main()
PY
  chmod +x "$TUI_FILE"
fi

# --- write OAuth helper if missing (prints ONLY refresh token) ---
if [[ ! -f "$OAUTH_HELPER" ]]; then
  cat >"$OAUTH_HELPER" <<'PY'
#!/usr/bin/env python3
# strava_oauth_helper.py — opens browser, listens locally, prints refresh token
import http.server, socketserver, threading, webbrowser, sys, os, urllib.parse, requests

REDIRECT_URI = os.environ.get("REDIRECT_URI", "http://localhost:53682/callback")
CLIENT_ID = os.environ["STRAVA_CLIENT_ID"]
CLIENT_SECRET = os.environ["STRAVA_CLIENT_SECRET"]

def _port_from_uri(uri: str) -> int:
    p = urllib.parse.urlparse(uri)
    if p.port: return p.port
    return 80

PORT = _port_from_uri(REDIRECT_URI)
CODE_HOLDER = {"code": None, "error": None}

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path != urllib.parse.urlparse(REDIRECT_URI).path:
            self.send_response(404); self.end_headers(); self.wfile.write(b"Not Found"); return
        qs = urllib.parse.parse_qs(parsed.query)
        if "error" in qs:
            CODE_HOLDER["error"] = qs["error"][0]
        else:
            CODE_HOLDER["code"] = qs.get("code", [None])[0]
        self.send_response(200); self.end_headers()
        self.wfile.write(b"<html><body><h2>Auth complete.</h2>You can close this window.</body></html>")

def serve_once():
    with socketserver.TCPServer(("127.0.0.1", PORT), Handler) as httpd:
        httpd.timeout = 180
        httpd.handle_request()

def main():
    params = {
        "client_id": CLIENT_ID,
        "redirect_uri": REDIRECT_URI,
        "response_type": "code",
        "approval_prompt": "auto",
        "scope": "read,activity:read_all",
    }
    url = "https://www.strava.com/oauth/authorize?" + urllib.parse.urlencode(params)
    try: webbrowser.open(url, new=1)
    except Exception: pass
    t = threading.Thread(target=serve_once, daemon=True); t.start(); t.join(timeout=190)
    if CODE_HOLDER["error"]:
        print("OAUTH_ERROR:" + CODE_HOLDER["error"], file=sys.stderr); sys.exit(2)
    if not CODE_HOLDER["code"]:
        print("OAUTH_TIMEOUT", file=sys.stderr); sys.exit(3)
    # exchange code
    r = requests.post("https://www.strava.com/oauth/token", data={
        "client_id": CLIENT_ID, "client_secret": CLIENT_SECRET,
        "code": CODE_HOLDER["code"], "grant_type": "authorization_code"
    }, timeout=30)
    if r.status_code != 200:
        print("TOKEN_EXCHANGE_FAILED:" + str(r.status_code), file=sys.stderr)
        print(r.text, file=sys.stderr); sys.exit(4)
    data = r.json()
    # Print ONLY the refresh token to stdout
    print(data.get("refresh_token",""))
if __name__ == "__main__": main()
PY
  chmod +x "$OAUTH_HELPER"
fi

# --- prompt for client creds (only if not already in env file) ---
if [[ -z "${STRAVA_CLIENT_ID-}" ]]; then read -r -p "Strava Client ID: " STRAVA_CLIENT_ID; fi
if [[ -z "${STRAVA_CLIENT_SECRET-}" ]]; then
  read -r -s -p "Strava Client Secret: " STRAVA_CLIENT_SECRET
  echo
fi
# make sure the OAuth helper sees these
export STRAVA_CLIENT_ID
export STRAVA_CLIENT_SECRET
export REDIRECT_URI

# --- acquire refresh token if missing ---
if [[ -z "${STRAVA_REFRESH_TOKEN-}" || "${STRAVA_REFRESH_TOKEN}" == "" ]]; then
  echo "[*] Opening browser to authorize (ensure Authorization Callback Domain is 'localhost')"
  REFRESH="$(NO_OPEN=1 TIMEOUT_SECS=600 "$VENV_PY" "$OAUTH_HELPER" 2> >(tee oauth_stderr.txt >&2) || true)"
  if [[ -z "$REFRESH" ]]; then
    echo "[!] OAuth did not return a refresh token."
    echo "    (see oauth_stderr.txt for details)"
    exit 1
  fi
  STRAVA_REFRESH_TOKEN="$REFRESH"
  cat >"$ENV_FILE" <<EOF
STRAVA_CLIENT_ID=$STRAVA_CLIENT_ID
STRAVA_CLIENT_SECRET=$STRAVA_CLIENT_SECRET
STRAVA_REFRESH_TOKEN=$STRAVA_REFRESH_TOKEN
# Optional conveniences:
# STRAVA_ACTIVITY_ID=
# STRAVA_OUTPUT_JSON=
EOF
  echo "[*] Saved credentials to $ENV_FILE"
fi

# --- run the TUI (uses .strava.env defaults) ---
echo "[*] Launching TUI…"
"$VENV_PY" "$TUI_FILE"
echo "[*] Done."
