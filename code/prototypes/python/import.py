import requests
import os
from tqdm import tqdm
import time
import json
import gpxpy
import gpxpy.gpx
from datetime import datetime, timedelta
from xml.etree.ElementTree import Element, SubElement, tostring

# TODO: make sure to exchange secret info with a secrets manager
REFRESH_TOKEN = "e7422345710f87d52d545ef54ff860f94a226c26"
CLIENT_ID = "154594"
CLIENT_SECRET = "25fe40bf90e9f91886d5636e06280c392f951262"

BASE_URL = "https://www.strava.com/api/v3"
OUTPUT_DIR = "output_files"
# NOTE: If the application gets approved, this can be increased
RATE_LIMIT_SHORT = 200  # requests per 15 minutes
RATE_LIMIT_LONG = 2000  # requests per day
PROGRESS_FILE = "downloaded_activities.json"

# saves after each download, so if a download is interrupted, it can be resumed.


def save_downloaded_activities(downloaded_activities):
    with open(PROGRESS_FILE, "w") as f:
        json.dump(list(downloaded_activities), f, indent=4)


# load all the downloaded files, so we can skip them in our check.


def load_downloaded_activities():
    if os.path.exists(PROGRESS_FILE):
        with open(PROGRESS_FILE, "r") as f:
            return set(json.load(f))
    return set()


# gets a new access token when calling the api


def refresh_access_token():
    print("[INFO] Refreshing access token...")
    url = "https://www.strava.com/oauth/token"
    response = requests.post(
        url,
        data={
            "client_id": CLIENT_ID,
            "client_secret": CLIENT_SECRET,
            "refresh_token": REFRESH_TOKEN,
            "grant_type": "refresh_token",
        },
    )
    if response.status_code == 200:
        print("[SUCCESS] Access token refreshed successfully.")
    else:
        print(
            f"[ERROR] Failed to refresh access token: {response.status_code} - {
                response.text
            }"
        )
    response.raise_for_status()
    return response.json()["access_token"]


def fetch_activity_data(activity_id, access_token):
    url = f"{BASE_URL}/activities/{activity_id}/streams"
    params = {
        "keys": "latlng,time,altitude,velocity_smooth,heartrate,cadence,watts,grade_smooth,moving",
        "resolution": "high",
        "access_token": access_token,
    }

    response = requests.get(url, params=params)
    # pause for 15 min if rate limited
    if response.status_code == 429:
        handle_rate_limit(response)
        response = requests.get(url, params=params)

    response.raise_for_status()
    stream_data = response.json()

    return stream_data


def fetch_activity_id_list(access_token):
    print("[INFO] Fetching all activity IDs...")

    # resolve url
    url = f"{BASE_URL}/athlete/activities"
    # initialize page counter
    page = 1
    # Init activities list
    activities = []
    activity_name_id_map = {}  # New dictionary to store activity name to ID mapping
    metadata = {}  # new metadata dictionary to save metadata

    # loop until there is no data in the response json
    while True:
        # set get params
        params = {"access_token": access_token, "per_page": 200, "page": page}
        # perform get request
        response = requests.get(url, params=params)
        if response.status_code == 429:
            print("[WARNING] Rate limit hit. Waiting before retrying...")
            handle_rate_limit(response)
            continue
        if response.status_code != 200:
            print(
                f"[ERROR] Failed to fetch activities: {response.status_code} - {
                    response.text
                }"
            )
            response.raise_for_status()
        response.raise_for_status()
        # get the json of the response
        data = response.json()
        if not data:
            print("[INFO] No more activities found.")
            break

        # for every entry in the json, get the 'id' value and store it into activities list
        for activity in data:
            is_cycling = activity.get("type") == "Ride"
            has_heart_rate = activity.get("has_heartrate", False)
            average_watts = activity.get("average_watts", None)
            is_indoors = activity.get("trainer", False)

            # add to list if it is an outdoor cycling ride, and has either HR or power metric
            if (is_cycling and not is_indoors) and (
                has_heart_rate or (
                    average_watts is not None and average_watts > 0)
            ):
                activity_id = str(activity["id"])
                metadata[activity_id] = {
                    "name": activity.get("name"),
                    "start_date": activity.get("start_date"),
                    "type": activity.get("type"),
                    "trainer": activity.get("trainer", False),
                    "distance": activity.get("distance"),
                    "has_heartrate": activity.get("has_heartrate", False),
                    "average_watts": activity.get("average_watts"),
                    "moving_time": activity.get("moving_time"),
                }
                activities.append(activity["id"])

        print(f"[INFO] Retrieved {len(data)} activities from page {page}.")
        page += 1
    save_name_id_mapping(activity_name_id_map, OUTPUT_DIR)
    save_metadata(metadata, OUTPUT_DIR)
    print(f"[SUCCESS] Saved metadata for {len(activities)} activities.")
    return activities


# ====================================METADATA======================================
# Saves metadata to json
def save_metadata(metadata, output_dir):
    print("[INFO] Saving metadata to file...")
    metadata_file = os.path.join(output_dir, "activity_metadata.json")
    with open(metadata_file, "w") as f:
        json.dump(metadata, f, indent=4)
    print(f"Saved metadata to {metadata_file}")


# Load metadata from json
def load_metadata(output_dir):
    metadata_file = os.path.join(output_dir, "activity_metadata.json")
    if not os.path.exists(metadata_file):
        raise FileNotFoundError(
            f"[ERROR] Metadata file not found at {metadata_file}")
    print("[INFO] Loading metadata from file...")
    with open(metadata_file, "r") as f:
        metadata = json.load(f)
    print(f"[SUCCESS] Loaded metadata from {metadata_file}.")

    return metadata


# ==================================================================================


def save_name_id_mapping(activity_map, output_dir):
    if not os.path.exists(output_dir):
        print(f"[SUCCESS] Created directory: {output_dir}")
        os.makedirs(output_dir)

    mapping_file = os.path.join(output_dir, "activityList.json")

    with open(mapping_file, "w") as f:
        json.dump(activity_map, f, indent=4)


def handle_rate_limit(response):
    # Fetch rate limit info from headers
    rate_limit = response.headers.get("X-RateLimit-Limit", "200,2000")
    rate_usage = response.headers.get("X-RateLimit-Usage", "0,0")

    short_limit, long_limit = map(int, rate_limit.split(","))
    short_usage, long_usage = map(int, rate_usage.split(","))

    print(
        f"Rate Limit Hit! Short-term usage: {short_usage}/{
            short_limit
        }, Long-term usage: {long_usage}/{long_limit}"
    )

    # Wait until the 15-minute window resets
    wait_time = 15 * 60  # 15 minutes in seconds
    print(f"Waiting for {wait_time / 60} minutes...")
    time.sleep(wait_time)


def save_to_gpx(activity_id, data, output_dir, metadata):
    print(f"[INFO] Processing activity {activity_id}...")
    activity_metadata = metadata.get(activity_id)
    if not activity_metadata:
        print(f"[WARNING] No metadata found for activity {
              activity_id}. Skipping...")
        return

    # Check if data is valid
    # get start date from metadata and convert to datetime object
    start_time_str = activity_metadata.get("start_date")
    try:
        start_time = datetime.strptime(start_time_str, "%Y-%m-%dT%H:%M:%SZ")
    except Exception as e:
        print(f"[ERROR] Failed to parse start date for activity {
              activity_id}: {e}")
        return

    # new gpx generator object
    gpx = gpxpy.gpx.GPX()
    # create a new track with the name of the activity
    gpx_track = gpxpy.gpx.GPXTrack(name=activity_metadata.get("name"))
    gpx.tracks.append(gpx_track)
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)

    # Create a dictionary for the stream data
    streams = {d["type"]: d["data"] for d in data if "data" in d}
    print(
        f"[INFO] Available streams for activity {activity_id}: {
            ', '.join(streams.keys())
        }"
    )

    # if esssential gpx data is missing, skips the creation of the file
    if "latlng" not in streams or "time" not in streams:
        print(
            f"[WARNING] Missing essential streams (latlng or time) for activity {
                activity_id
            }. Skipping..."
        )
        return

    # Basic GPX data
    latlng_data = streams.get("latlng", [])
    altitude_data = streams.get("altitude", [None] * len(latlng_data))
    time_data = streams.get("time", [None] * len(latlng_data))

    # Extension GPX data (aggregated in a list)
    extension_variables = [
        "velocity_smooth",
        "heartrate",
        "cadence",
        "watts",
        "grade_smooth",
        "moving",
    ]

    # get data streams that are present in the input data
    data_streams = {
        var: streams.get(
            # NOTE: might need to look here
            var,
            [None] * len(latlng_data),
        )
        for var in extension_variables
    }

    # for each coordinate position
    for i, latlng in enumerate(latlng_data):
        if latlng:
            # generate a new track point if the latlng exists
            point = gpxpy.gpx.GPXTrackPoint(
                latitude=latlng[0], longitude=latlng[1])

        # add and calculate time metadata
        if i < len(time_data) and time_data[i] is not None:
            # set the current trackpoint's time equal to start time plus the time offset
            point.time = start_time + timedelta(seconds=time_data[i])

        # add the elevation data
        if i < len(altitude_data) and altitude_data[i] is not None:
            point.elevation = altitude_data[i]

        # dynamically add extensions that exist in the data stream if the values exist and i is within the range of values
        extensions = []
        for var, values in data_streams.items():
            if i < len(values) and values[i] is not None:
                extension_element = Element(var)
                extension_element.text = str(values[i])
                point.extensions.append(extension_element)
        if extensions:
            for extensio_element in extensions:
                point.extensions.append(extension_element)
        gpx_segment.points.append(point)

    # write file to .gpx
    try:
        output_file = os.path.join(output_dir, f"{activity_id}.gpx")
        with open(output_file, "w") as f:
            f.write(gpx.to_xml())
        print(f"[SUCCESS] Saved GPX file for activity {
              activity_id} to {output_file}")
    except Exception as e:
        print(f"[ERROR] Failed to save GPX file for activity {
              activity_id}: {e}")


if __name__ == "__main__":
    # Refresh Access Token
    access_token = refresh_access_token()

    # Fetch all Ids
    activity_ids = fetch_activity_id_list(access_token)

    # Create Files
    # create output directory
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    gpx_dir = os.path.join(OUTPUT_DIR, "gpx")
    os.makedirs(gpx_dir, exist_ok=True)
    metadata = load_metadata(OUTPUT_DIR)

    downloaded_activities = load_downloaded_activities()
    activity_ids = list(metadata.keys())

    total_activities = len(activity_ids)
    # for each activity ID, create a file (named accordingly) of the .json
    for activity_id in tqdm(activity_ids, desc="Downloading activities as json"):
        # if str(activity_id) in downloaded_activities:
        #     continue
        try:
            json_file = os.path.join(OUTPUT_DIR, f"{activity_id}.json")
            if os.path.exists(json_file):
                print(
                    f"[INFO] JSON file found for activity {
                        activity_id
                    }. Skipping download."
                )
                try:
                    with open(json_file, "r") as f:
                        data = json.load(f)
                    save_to_gpx(activity_id, data, gpx_dir, metadata)
                    continue
                except Exception as e:
                    print(
                        f"[ERROR] Failed to load existing JSON file for activity {
                            activity_id
                        }: {e}"
                    )
                    continue
            else:
                data = fetch_activity_data(activity_id, access_token)
                save_to_gpx(activity_id, data, gpx_dir, metadata)
                output_file = os.path.join(OUTPUT_DIR, f"{activity_id}.json")
                with open(output_file, "w") as f:
                    json.dump(data, f, indent=4)
                downloaded_activities.add(str(activity_id))
                save_downloaded_activities(downloaded_activities)
        except Exception as e:
            print(f"Failed to download activity: {activity_id}: {e}")
