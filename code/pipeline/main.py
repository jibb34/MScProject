import argparse
import os
import venv
import subprocess


from utils.gpx_utils import bounding_box_from_data
from utils.gpx_utils import load_gpx, write_json, extract_data_from_gpx, download_osm_pbf


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in a GPX file"
    )
    parser.add_argument(
        "gpx_file", help="The GPX file that the GPS Observations are being read from"
    )
    parser.add_argument(
        "pbf_output", help="filename (with extension .pbf) that the downloaded map is saved to"
    )
    parser.add_argument(
        "--debug", action="store_true", help="Run in debug mode"
    )

    return parser.parse_args()


def main(debug=False):
    if debug:
        print("[DEBUG] Debug Mode Active")
    args = parse_args()

    GPX_FILE = args.gpx_file
    PBF_FILE = args.pbf_output

    # ========Load data into points=======
    gpx = load_gpx(GPX_FILE)
    points = extract_data_from_gpx(gpx)
    bbox = bounding_box_from_data(points)
    # ====================================

    # ======= Get data from OSRM =========
    # if it doesn't already exist
    print("Downloading OSM Graph")
    if not os.path.exists(PBF_FILE):
        G = download_osm_pbf(bbox, PBF_FILE)

    print("Saving PBF for OSRM")
    # ====================================


if __name__ == "__main__":
    args = parse_args()
    main(debug=args.debug)
