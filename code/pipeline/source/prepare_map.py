import argparse
import os
from concurrent.futures import ThreadPoolExecutor, as_completed

from utils.gpx_utils import bounding_box_from_data, download_osm_file
from utils.gpx_utils import load_gpx, extract_data_from_gpx
from utils.osrm_utils import points_to_osrm_json


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pass in a GPX file"
    )
    parser.add_argument(
        "gpx_file_path", help="The directory of GPX files that the"
        " GPS Observations are being read from"
    )
    parser.add_argument(
        "osm_output_path", help="filename (with extension .osm)"
        " that the downloaded map is saved to"
    )
    parser.add_argument(
        "radius", help="The radius (in m) that each point"
        " searches for a nearby route")
    parser.add_argument(
        "chunk_size", help="Size of json chunks sent to OSRM, "
        "default 0 (no chunking)")
    return parser.parse_args()


def thread_processes(gpx_file, output_path, chunk_dir, radius, chunk_size):
    """ Thread processes that are executed at the start of the pipeline.
    Includes reading and parsing GPX files, splitting the files into chunks,
    and downloading the OSM file from the Overpass API
    Could maybe further optimise this by having separate threads for splitting
    chunks and downloading files from Overpass, but loading the files
    must be done first regardless, so not much time would be saved from
    creating more threads"""
    # Gather data
    gpx = load_gpx(gpx_file)
    points = extract_data_from_gpx(gpx)
    # Get all names that are required
    gpx_name = os.path.splitext(os.path.basename(gpx_file))[0]
    output_file = gpx_name + ".osm"
    os.makedirs(output_path, exist_ok=True)
    full_output_path = os.path.join(output_path, output_file)

    points_to_osrm_json(points, chunk_dir, gpx_name, radius,
                        chunk_size)
    if not os.path.exists(full_output_path):
        print(len(points))
        get_single_osm_from_gpx(gpx_file, points, gpx_name, output_path)
    else:
        print(f"[WARNING] {output_file} exists in {output_path}, skipping...")
        return


def get_single_osm_from_gpx(gpx_file, points, gpx_name, output_path):
    bbox = bounding_box_from_data(points)
    gpx_name = os.path.splitext(os.path.basename(gpx_file))[0]
    output_file = gpx_name + ".osm"
    full_output_path = os.path.join(output_path, output_file)

    # ========Load data into points=======
    gpx = load_gpx(gpx_file)
    points = extract_data_from_gpx(gpx)
    bbox = bounding_box_from_data(points)

    # ======= Get data from Overpass =====
    if not os.path.exists(full_output_path):
        download_osm_file(bbox, full_output_path)
    else:
        print(f"[WARNING] {output_file} exists in {output_path}, skipping...")
        return


def main():
    args = parse_args()
    GPX_FILE_PATH = args.gpx_file_path
    OSM_FILE_PATH = args.osm_output_path
    CHUNK_SIZE = int(args.chunk_size)
    RADIUS = args.radius
    MAX_THREADS = min(32, sum(1 for file in os.listdir(GPX_FILE_PATH)))

    print("[PYTHON] Downloading OSM Graph")

    gpx_files = [
        os.path.join(GPX_FILE_PATH, f)
        for f in os.listdir(GPX_FILE_PATH)
        if os.path.isfile(os.path.join(GPX_FILE_PATH, f))
        and os.path.splitext(f)[1].lower() == ".gpx"]

    print(f"Found {len(gpx_files)} GPX files: {gpx_files}")
    with ThreadPoolExecutor(max_workers=MAX_THREADS) as executor:
        futures = [executor.submit(thread_processes,
                                   gpx_file,
                                   OSM_FILE_PATH,
                                   f"data/temp/{os.path.splitext(
                                       os.path.basename(gpx_file))[0]}",
                                   RADIUS,
                                   CHUNK_SIZE
                                   )
                   for gpx_file in gpx_files]
    for future in as_completed(futures):
        future.result()


if __name__ == "__main__":
    print("test")
    main()
