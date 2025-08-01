# GOAL: Detect Multi-scale changes in GPX files using Wavelets.
#
# Should parse GPX files, gather the metadata (Power, Elevation, Heartrate, Cadence, etc.) and format it into numpy arrays.
# Then, using PyWavelet, the program should analyise at three levels of granularity (<1 minute effort, 1-10 min, and 10+ min effort).
#
# Candidates for significant changes should be peaks in the absolute detail coefficient that are larger than a threshold value time
# the median absolute deviation (MAD).
# Peaks that are closer than a minimum merge tolerance time frame should be merged together.
#


from __future__ import annotations

from pathlib import Path
from typing import List, Dict, Sequence
from datetime import timezone

import numpy as np
import pandas as pd
import pywt
import gpxpy


# ================== GPX LOADING AND PRE-PROCESSING =======================
#
def load_gpx_as_dataframe(
    path: str | Path, resample_interval: str | None = "1S"
) -> pd.DataFrame:
    path = Path(path)  # get file path
    if not path.exists():
        raise FileNotFoundError(path)
    # Open GPX File
    with path.open("r", encoding="utf-8") as fh:
        gpx = gpxpy.parse(fh)  # parse the gpx file using the gpxpy library
    # Collect Points into array
    points = []
    # GPX File have the hierarchy: Track -> Segment -> Point
    for track in gpx.tracks:
        for segment in track.segments:
            for p in segment.points:
                if p.time is None:
                    continue  # Skip points that don't have a time stamp, they are useless
                raw_time = p.time  # Get the raw time
                if raw_time.tzinfo is None:
                    raw_time = raw_time.replace(tzinfo=timezone.utc)
                # Base Data to add to structure:
                point_data: dict = {
                    "time": raw_time,
                    "lat": p.latitude,
                    "long": p.longitude,
                    "elevation": p.elevation,
                }
                # If extensions exist, append them here
                if p.extensions:
                    for extension in p.extensions:
                        # Get the name of the extension
                        tag = extension.tag.split("}")[-1]
                        # Get the value of the extension and store into data structure
                        # if the value is non-numerical, convert to NaN
                        point_data[tag] = float(extension.text or np.nan)

    # Define DataFrame as the set of points we have collected, and use the "Time" value as the indexing, then sort by time to keep chronology
    df = pd.DataFrame(points).set_index("time").sort_index()

    # TODO: Possibly Implement resampling to make sure every interval is consistent (if GPX data does not have values at specific intervals)
    # Also, can derive metrics if missing (aka. speed from position)
    if resample_interval is not None:
        df = df.resample(resample_interval).ffill().dropna(how="all")
    return df


# ========================= CHANGEPOINT DETECTION ====================================
# TODO: Define functions relating to Wavelets, define the three levels, and the main business logic of detecting changepoints.


# INTENSITY BINS - Different Timeframes (in seconds) for each Wavelet Focus, based on Training Zones
INTENSITY_BINS = {
    "neuromuscular": (0.0, 20.0),  # seconds
    "anaerobic": (20.0, 180.0),
    "vo2_max": (180.0, 600.0),
    "threshold": (600.0, 3600.0),
    "tempo_endurance": (3600.0, float("inf")),
}


# Function to return the category based on the given sample interval
def _level_category(level: int, dt_sec: float) -> str | None:
    # Return intensity bucket name for SWT *level* given sample interval.
    window = (2**level) * dt_sec
    for name, (lo, hi) in INTENSITY_BINS.items():
        if lo < window <= hi:
            return name
    return None
