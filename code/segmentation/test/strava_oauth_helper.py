#!/usr/bin/env python3
# strava_oauth_helper.py — open auth URL, capture redirect, print refresh token
import http.server, socketserver, threading, webbrowser, sys, os, urllib.parse, requests, platform, subprocess, time, socket
REDIRECT_URI = os.environ.get("REDIRECT_URI", "http://localhost:53682/callback")
CLIENT_ID = os.environ["STRAVA_CLIENT_ID"]
CLIENT_SECRET = os.environ["STRAVA_CLIENT_SECRET"]
AUTH_CODE = os.environ.get("AUTH_CODE")              # optional: skip server, exchange this code directly
NO_OPEN = os.environ.get("NO_OPEN", "1") == "1" 
TIMEOUT_SECS = int(os.environ.get("TIMEOUT_SECS", "180"))

def _port_from_uri(uri: str) -> int:
    p = urllib.parse.urlparse(uri)
    return p.port or 80

PORT = _port_from_uri(REDIRECT_URI)
CODE = {"code": None, "error": None}

class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        # quiet server logs
        pass

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        target_path = urllib.parse.urlparse(REDIRECT_URI).path or "/callback"
        if parsed.path != target_path:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not Found")
            return
        qs = urllib.parse.parse_qs(parsed.query)
        if "error" in qs:
            CODE["error"] = qs["error"][0]
        else:
            CODE["code"] = qs.get("code", [None])[0]
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"<html><body><h2>Auth complete.</h2>You can close this window.</body></html>")

def serve_once():
    # Listen on both IPv4 and IPv6 if possible; whichever arrives first wins.
    done = threading.Event()
    errs = []
    target_path = urllib.parse.urlparse(REDIRECT_URI).path or "/callback"

    class Srv(socketserver.TCPServer):
        allow_reuse_address = True

    def run(host, fam):
        try:
            class SrvFam(Srv): pass
            SrvFam.address_family = fam
            with SrvFam((host, PORT), Handler) as httpd:
                httpd.timeout = TIMEOUT_SECS
                print(f"[INFO] Listening on http://{host}:{PORT}{target_path} (timeout {TIMEOUT_SECS}s)", file=sys.stderr, flush=True)
                end = time.time() + TIMEOUT_SECS
                while not done.is_set() and time.time() < end:
                    httpd.handle_request()
                    if CODE.get("code") or CODE.get("error"):
                        done.set()
                        break
        except Exception as e:
            errs.append((host, str(e)))

    threads = []
    t4 = threading.Thread(target=run, args=("127.0.0.1", socket.AF_INET), daemon=True); t4.start(); threads.append(t4)
    try:
        socket.socket(socket.AF_INET6).close()
        t6 = threading.Thread(target=run, args=("::1", socket.AF_INET6), daemon=True); t6.start(); threads.append(t6)
    except Exception:
        pass
    done.wait(TIMEOUT_SECS)
    if not done.is_set():
        print("OAUTH_TIMEOUT", file=sys.stderr); sys.exit(3)


def serve_dual_until_hit():
    """
    Listen on both 127.0.0.1 (IPv4) and ::1 (IPv6) so browsers that prefer IPv6 work.
    Return when we receive one request with a ?code=... or ?error=...
    """
    done = threading.Event()
    errs = []

    def serve(host, fam_label, family):
        try:
            class Srv(socketserver.TCPServer):
                allow_reuse_address = True
                address_family = family
            with Srv((host, PORT), Handler) as httpd:
                httpd.timeout = TIMEOUT_SECS
                print(f"[INFO] Listening on http://{host}:{PORT}/callback ({fam_label}, timeout {TIMEOUT_SECS}s)", file=sys.stderr, flush=True)
                end = time.time() + TIMEOUT_SECS
                while not done.is_set() and time.time() < end:
                    httpd.handle_request()
                    # Handler sets CODE[...] when hit; poll it:
                    if CODE.get("code") or CODE.get("error"):
                        done.set()
                        break
        except Exception as e:
            errs.append((host, str(e)))

    threads = []
    # IPv4
    t4 = threading.Thread(target=serve, args=("127.0.0.1", "IPv4", socket.AF_INET), daemon=True)
    t4.start(); threads.append(t4)
    # IPv6 (if supported)
    try:
        socket.socket(socket.AF_INET6).close()
        t6 = threading.Thread(target=serve, args=("::1", "IPv6", socket.AF_INET6), daemon=True)
        t6.start(); threads.append(t6)
    except Exception:
        pass

    done.wait(TIMEOUT_SECS)
    if not done.is_set():
        print("OAUTH_TIMEOUT", file=sys.stderr)
        sys.exit(3)

def exchange(code: str):
    r = requests.post("https://www.strava.com/oauth/token", data={
        "client_id": CLIENT_ID,
        "client_secret": CLIENT_SECRET,
        "code": code,
        "grant_type": "authorization_code",
    }, timeout=30)
    if r.status_code != 200:
        print("TOKEN_EXCHANGE_FAILED:" + str(r.status_code), file=sys.stderr)
        print(r.text, file=sys.stderr)
        sys.exit(4)
    print(r.json().get("refresh_token", ""))

def main():
    # Fast path: allow manual AUTH_CODE use (no local server)
    if AUTH_CODE:
        exchange(AUTH_CODE)
        return

    params = {
        "client_id": CLIENT_ID,
        "redirect_uri": REDIRECT_URI,
        "response_type": "code",
        "approval_prompt": "force",                   # ← show consent again
        "scope": "read,activity:read,activity:read_all",  # ← include both
    }
    url = "https://www.strava.com/oauth/authorize?" + urllib.parse.urlencode(params)
    # Start the server first, then show the URL so you're safe to click it anytime.
    t = threading.Thread(target=serve_once, daemon=True); t.start()
    print(f"[INFO] Auth URL:\n{url}\n", file=sys.stderr, flush=True)
    if not NO_OPEN:
        # If you really want auto-open, use macOS 'open' explicitly; ignore failures.
        try: subprocess.run(["open", url], check=False)
        except Exception: print("[WARN] Auto-open failed. Copy/paste the URL above.", file=sys.stderr, flush=True)
    t.join(timeout=TIMEOUT_SECS + 10)

    if CODE["error"]:
        print(f"OAuth error: {CODE['error']}", file=sys.stderr)
        sys.exit(2)
    if not CODE["code"]:
        print("OAUTH_TIMEOUT", file=sys.stderr)
        sys.exit(3)

    exchange(CODE["code"])

if __name__ == "__main__":
    main()

