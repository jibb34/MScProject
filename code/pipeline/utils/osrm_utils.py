import os
import json
from datetime import datetime
from pathlib import Path


"""Generates JSON object(s) from a list of GPX trackpoints, in the format:
{
    "lat": 51.501,
    "lon": -0.141,
    "time": "2025-08-02T12:34:56Z",
    ...
}
Args: gpx_points (list): List of gpx trackpoint objects
      output_dir (str): Directory to save the JSON files
      chunk_size (int): Number of GPX points per chunk. 0 = no chunking.
Returns:
      List of output JSON file paths
Notes:
      This is a required step outside of saving gpx data into json format
      /match OSRM API requires a specific json object format, meaning we must 
      remove extensions and other metadata
    """


def points_to_osrm_json(gpx_points, output_dir, chunk_size=0):

    Path(output_dir).mkdir(parents=True, exist_ok=True)

    # Create our payload from a list of points
    def create_payload(points):
        coords = [[pt["lon"], pt["lat"]]
                  for pt in points]  # Create coordinate pair from points
        timestamps = []
        for pt in points:
            if "time" in pt and pt["time"]:
                ts = pt["time"]
                if isinstance(ts, str):
                    try:
                        dt = datetime.fromisoformat(ts.replace("Z", "+00:00"))
                        timestamps.append(int(dt.timestamp()))
                    except ValueError:
                        timestamps.append(None)
                else:
                    timestamps.append(int(ts.timestamp()))

            payload = {
                "coordinates": coords,
                "geometries": "geojson",
                "overview": "full",
                "steps": False
            }
            if any(t is None for t in timestamps):
                raise ValueError(
                    "Missing or invalid timestamps in GPX data. Attempting to interpolate...")
                for i, t in enumerate(timestamps):
                    if t is None:
                        try:
                            t = timestamps[i-1] + 1
                        except Exception:
                            t = timestamps[i+1] - 1

            payload["timestamps"] = timestamps

        return payload

    # Create payload, and save to .json
    output_files = []
    if chunk_size <= 0:
        # no chunking
        payload = create_payload(gpx_points)
        path = os.path.join(output_dir, "match.json")
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)
        output_files.append(path)
    else:
        # chunk_size elements per chunk
        for i in range(0, len(gpx_points), chunk_size):
            chunk = gpx_points[i:i+chunk_size]
            payload = create_payload(chunk)
            path = os.path.join(output_dir, f"match_chunk_{
                                i // chunk_size}.json")
            with open(path, "w") as f:
                json.dump(payload, f, indent=2)
            output_files.append(path)
    return output_files
