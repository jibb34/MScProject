import requests
import json

CLIENT_ID = "154594"
CLIENT_SECRET = "25fe40bf90e9f91886d5636e06280c392f951262"
REFRESH_TOKEN = "b31a2176d721d0310e63c60563d22cf702ac5a91"


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
        with open("tokens.json", "w") as token_file:
            json.dump(new_tokens, token_file)

        print("Access token refreshed successfully.")
        return new_tokens["access_token"]
    else:
        print("Failed to refresh token:", new_tokens)
        return None


# Load the latest token from file
try:
    with open("tokens.json", "r") as token_file:
        tokens = json.load(token_file)
        ACCESS_TOKEN = tokens["access_token"]
except FileNotFoundError:
    ACCESS_TOKEN = refresh_access_token()
