#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

from utils.gpx_utils import extract_data_from_gpx, load_gpx, load_json, write_json
from gpx_enrich import attach_to_tracepoints
# import your helpers from gpx_enrich.py


def find_gpx(gpx_dir: Path, stem: str) -> Path | None:
    """Find <stem>.gpx (case-insensitive) in gpx_dir."""
    print(f"[INFO] Looking for .gpx files in {gpx_dir}")
    cand1 = gpx_dir / f"{stem}.gpx"
    cand2 = gpx_dir / f"{stem}.GPX"
    if cand1.exists():
        return cand1
    if cand2.exists():
        return cand2
    # try any file with same stem and .gpx-like suffix
    for p in gpx_dir.glob(f"{stem}.*"):
        if p.suffix.lower() == ".gpx":
            return p
    return None


def process_one(json_path: Path, gpx_dir: Path, out_dir: Path,
                tol_m: float, precision: int,
                add_leg_times: bool, node_times: bool) -> tuple[Path, str]:
    stem = json_path.stem
    gpx_path = find_gpx(gpx_dir, stem)
    if not gpx_path:
        return json_path, f"[SKIP] no GPX found for {stem}"

    try:
        osrm = load_json(json_path)
        gpx = load_gpx(gpx_path)
        gpx_points = extract_data_from_gpx(gpx)
        attach_to_tracepoints(osrm, gpx_points, tol_m, precision=4)

        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"{stem}_enriched.json"
        write_json(osrm, out_path)
        return json_path, f"[OK] {json_path.name} -> {out_path.name}"
    except Exception as e:
        return json_path, f"[ERR] {json_path.name}: {e}"


def main():
    ap = argparse.ArgumentParser(
        description="Batch-enrich OSRM JSONs with matching GPX metadata.")
    ap.add_argument("--json-dir", required=True,
                    help="Directory with .json files")
    ap.add_argument("--gpx-dir",  required=True,
                    help="Directory with .gpx files (same basenames)")
    ap.add_argument("--out-dir",  required=True,
                    help="Output directory for *_enriched.json")
    ap.add_argument("--tol", type=float, default=15.0,
                    help="Match tolerance (meters)")
    ap.add_argument("--precision", type=int, default=5,
                    help="Grid precision (decimals)")
    ap.add_argument("--no-leg-times", action="store_true",
                    help="Don’t add per-leg time windows")
    ap.add_argument("--node-times", action="store_true",
                    help="Also interpolate per-node timestamps")
    ap.add_argument("--workers", type=int, default=4, help="Parallel workers")
    args = ap.parse_args()

    json_dir = Path(args.json_dir)
    gpx_dir = Path(args.gpx_dir)
    out_dir = Path(args.out_dir)

    json_files = sorted(p for p in json_dir.glob("*.json") if p.is_file())
    if not json_files:
        print(f"No .json files found in {json_dir}")
        return

    tasks = []
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        for jp in json_files:
            tasks.append(ex.submit(
                process_one, jp, gpx_dir, out_dir,
                args.tol, args.precision,
                not args.no_leg_times, args.node_times
            ))
        for fut in as_completed(tasks):
            _jp, msg = fut.result()
            print(msg)


if __name__ == "__main__":
    main()
