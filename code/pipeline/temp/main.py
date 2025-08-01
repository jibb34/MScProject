import os
import gpxpy
import json
import argparse

# Using gpxpy, parse a GPX file and return a GPX object


def parse_args():
    parser = argparse.ArgumentParser(
        description="Normalize a GPX file to a compact JSON list of track-point objects"
    )
    parser.add_argument("gpx_file", help="Path to the GPX file to read")
    parser.add_argument("output_json", help="Path to write the normalized JSON output")
    return parser.parse_args()


def load_gpx(path):
    with open(path, "r") as f:
        return gpxpy.parse(f)


# Convert GPX object into JSON standardized (simpler)
def gpx_to_points(gpx):
    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for pt in segment.points:
                # Each point must have at least latitude and longitude
                point = {
                    "lat": pt.latitude,
                    "long": pt.longitude,
                    "elv": pt.elevation if pt.elevation is not None else None,
                    "time": pt.time.isoformat() if pt.time else None,
                }

                # include extensions
                # TODO: Possibly implement recursive algorithm for dealing with children
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


def write_json(points, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        # separators remove spaces, to get a smaller file size
        json.dump(points, f, separators=(",", ":"), ensure_ascii=False)


def main():
    args = parse_args()
    gpx = load_gpx(args.gpx_file)
    points = gpx_to_points(gpx)
    write_json(points, args.output_json)


if __name__ == "__main__":
    main()
