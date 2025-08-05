import argparse
from utils.gpx_utils import bounding_box_from_data, download_osm_file
from utils.gpx_utils import load_gpx, extract_data_from_gpx
from utils.osrm_utils import points_to_osrm_json


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in a GPX file"
    )
    parser.add_argument(
        "gpx_file", help="The GPX file that the"
        " GPS Observations are being read from"
    )
    parser.add_argument(
        "osm_output", help="filename (with extension .osm)"
        " that the downloaded map is saved to"
    )
    parser.add_argument(
        "json_dir", help="Directory that the json chunks"
        " will be temporarily stored in")
    parser.add_argument(
        "radius", help="The radius (in m) that each point"
        " searches for a nearby route")
    parser.add_argument(
        "chunk_size", help="Size of json chunks sent to OSRM, "
        "default 0 (no chunking)")
    parser.add_argument(
        "--debug", action="store_true", help="Run in debug mode"
    )
    return parser.parse_args()


def main(debug=False):
    if debug:
        print("[DEBUG] Debug Mode Active")
    args = parse_args()

    GPX_FILE = args.gpx_file
    OSM_FILE = args.osm_output
    JSON_DIR = args.json_dir
    CHUNK_SIZE = int(args.chunk_size)
    RADIUS = args.radius

    # ========Load data into points=======
    gpx = load_gpx(GPX_FILE)
    points = extract_data_from_gpx(gpx)
    bbox = bounding_box_from_data(points)
    # ====================================

    # ======= Get data from Overpass =====
    # if it doesn't already exist
    print("[PYTHON] Downloading OSM Graph")
    download_osm_file(bbox, OSM_FILE)
    # if not os.path.exists(OSM_FILE):
    #     G = download_osm_file(bbox, OSM_FILE)

    print("[PYTHON] Saving points json for OSRM")
    # =========Package data into json=====
    points_to_osrm_json(points, JSON_DIR, RADIUS, CHUNK_SIZE)


if __name__ == "__main__":
    args = parse_args()
    main(debug=args.debug)
