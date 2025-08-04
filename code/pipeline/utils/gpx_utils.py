import gpxpy
import osmnx as ox
import json
import os


def load_gpx(path):
    with open(path, "r") as f:
        return gpxpy.parse(f)


def bounding_box_from_data(data):
    # TODO: Possibly may need to expand the bbox slightly if error matching causes issues, route could go outside the box even if gpx points don't
    # Get all lat/long values
    lats = [p["lat"] for p in data if "lat" in p and p["lat"] is not None]
    longs = [p["lon"] for p in data if "lon" in p and p["lon"] is not None]
    # get the extrema
    north = max(lats)
    south = min(lats)
    east = max(longs)
    west = min(longs)

    # return a quaduple of the extrema in bbox format (left, bottom, right, top)
    return (south, west, north, east)


def load_json(file_path):
    with open(file_path, "r") as f:
        return json.load(f)


def write_json(points, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        # separators remove spaces, to get a smaller file size
        json.dump(points, f, separators=(",", ":"), ensure_ascii=False)


def download_osm_file(bbox, output_path):
    import requests

    # Convert bbox to Overpass API bbox string:
    s, w, n, e = bbox
    print(f"Min Lat: {s}, max lat: {n}, min long: {w}, max long: {e}")
    bbox_str = f"{w},{s},{e},{n}"
    url = f"https://overpass-api.de/api/map?bbox={bbox_str}"

    response = requests.get(url)
    with open(output_path, 'wb') as f:
        f.write(response.content)


def extract_data_from_gpx(gpx):
    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for pt in segment.points:
                # Each point must have at least latitude and longitude
                point = {
                    "lat": pt.latitude,
                    "lon": pt.longitude,
                    "elv": pt.elevation if pt.elevation is not None else None,
                    "time": pt.time.isoformat() if pt.time else None,
                }

                # include extensions
                if pt.extensions:
                    for ext in pt.extensions:
                        # If an extension does have children, parse them individually, otherwise add the key:value pair directly
                        if any(True for _ in ext):
                            for child in ext:
                                point[child.tag.split("}")[-1]] = child.text
                        else:
                            tag = ext.tag.split("}")[-1]
                            text = (
                                ext.text.strip()
                                if ext.text and ext.text.strip()
                                else None
                            )
                            if text:
                                point[tag] = text

                # Add parsed point to list
                points.append(point)

    return points
