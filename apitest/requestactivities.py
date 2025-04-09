import requests
import json
import os
import time

# Your Strava API credentials
CLIENT_ID = "154594"
CLIENT_SECRET = "25fe40bf90e9f91886d5636e06280c392f951262"
REFRESH_TOKEN = "b31a2176d721d0310e63c60563d22cf702ac5a91"
TOKENS_FILE = "tokens.json"

# Function to refresh the access token


def refresh_access_token():
    url = "https://www.strava.com/oauth/token"
    payload = {
        'client_id': CLIENT_ID,
        'client_secret': CLIENT_SECRET,
        'grant_type': 'refresh_token',
        'refresh_token': REFRESH_TOKEN
    }

    response = requests.post(url, data=payload)
    new_tokens = response.json()

    if "access_token" in new_tokens:
        with open(TOKENS_FILE, "w") as token_file:
            json.dump(new_tokens, token_file)

        print("Access token refreshed successfully.")
        return new_tokens["access_token"], new_tokens["refresh_token"]
    else:
        print("Failed to refresh token:", new_tokens)
        return None, None

# Load tokens from file or refresh if not available


def load_access_token():
    try:
        with open(TOKENS_FILE, "r") as token_file:
            tokens = json.load(token_file)
            return tokens["access_token"], tokens["refresh_token"]
    except (FileNotFoundError, KeyError):
        return refresh_access_token()


ACCESS_TOKEN, REFRESH_TOKEN = load_access_token()

# Fetch all activities


def get_activities(page=1):
    url = f"https://www.strava.com/api/v3/athlete/activities"
    headers = {"Authorization": f"Bearer {ACCESS_TOKEN}"}
    params = {"per_page": 200, "page": page}
    response = requests.get(url, headers=headers, params=params)

    try:
        activities = response.json()
    except ValueError:
        print("Error: Failed to parse response as JSON")
        print(response.text)
        return []

    if not isinstance(activities, list):
        print("Error: Expected a list of activities, got:", type(activities))
        print(activities)
        return []

    return activities

# Fetch activity streams (for latlng, power, HR, cadence data)


def get_activity_streams(activity_id):
    url = f"https://www.strava.com/api/v3/activities/{activity_id}/streams"
    headers = {"Authorization": f"Bearer {ACCESS_TOKEN}"}
    params = {"keys": "latlng,time,power,heartrate,cadence",
              "key_by_type": "true"}
    response = requests.get(url, headers=headers, params=params)

    try:
        streams = response.json()
    except ValueError:
        print(f"Error: Failed to parse streams for activity {activity_id}")
        print(response.text)
        return {}

    return streams


# Save activity as .gpx
def save_gpx(activity_id, streams, activity_name):
    filename = f"{activity_name}_{activity_id}.gpx".replace(" ", "_")
    with open(filename, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<gpx version="1.1" creator="Strava API">\n')
        f.write('  <trk>\n')
        f.write(f'    <name>{activity_name}</name>\n')
        f.write('    <trkseg>\n')

        latlng_data = streams.get("latlng", {}).get("data", [])
        time_data = streams.get("time", {}).get("data", [])
        power_data = streams.get("power", {}).get("data", [])
        hr_data = streams.get("heartrate", {}).get("data", [])
        cadence_data = streams.get("cadence", {}).get("data", [])

        for i, point in enumerate(latlng_data):
            lat, lon = point
            f.write(f'      <trkpt lat="{lat}" lon="{lon}">\n')

            if i < len(time_data):
                f.write(f'        <time>{time_data[i]}</time>\n')

            if i < len(power_data):
                f.write(f'        <extensions><power>{
                        power_data[i]}</power></extensions>\n')

            if i < len(hr_data):
                f.write(f'        <extensions><heartrate>{
                        hr_data[i]}</heartrate></extensions>\n')

            if i < len(cadence_data):
                f.write(f'        <extensions><cadence>{
                        cadence_data[i]}</cadence></extensions>\n')

            f.write('      </trkpt>\n')

        f.write('    </trkseg>\n')
        f.write('  </trk>\n')
        f.write('</gpx>\n')

    print(f"Saved: {filename}")


    # Fetch and save all cycling activities
page = 1
while True:
    activities = get_activities(page)
    if not activities:
        break

    for activity in activities:
        if isinstance(activity, dict) and activity.get('type') in ["Ride", "VirtualRide"]:
            activity_id = activity['id']
            activity_name = activity.get('name', f"Activity_{activity_id}")
            print(f"Fetching activity: {activity_name} (ID: {activity_id})")

            streams = get_activity_streams(activity_id)

            if 'latlng' in streams:
                save_gpx(activity_id, streams, activity_name)
            else:
                print(f"No GPS data found for {
                      activity_name} (ID: {activity_id})")

    page += 1
    time.sleep(1)  # Avoid hitting the rate limit
