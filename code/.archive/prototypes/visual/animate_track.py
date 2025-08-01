import json
import folium
from folium.plugins import TimestampedGeoJson

INPUT_JSON = "enriched.json"
OUTPUT_HTML = "animation.html"


def load_points(path):
    with open(path, "r") as f:
        return json.load(f)


def build_timestamped_geojson(points):
    features = []
    for p in points:
        t = p.get("time")
        if not t:
            continue

        # pull raw values
        speed = p.get("speed")
        heading = p.get("heading")

        # format them safely
        speed_str = f"{speed:.1f} km/h" if speed is not None else "N/A"
        heading_str = f"{heading:.1f}°" if heading is not None else "N/A"

        popup = f"Speed: {speed_str}<br>Heading: {heading_str}"

        features.append(
            {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [p["long"], p["lat"]],
                },
                "properties": {
                    "time": t,
                    "popup": popup,
                    "icon": "circle",
                    "iconstyle": {"fillColor": "red", "radius": 5},
                },
            }
        )

    return {"type": "FeatureCollection", "features": features}


def main():
    points = load_points(INPUT_JSON)
    if not points:
        print("No points to animate.")
        return

    # center map on first point
    start = points[0]
    m = folium.Map(location=[start["lat"], start["long"]], zoom_start=14)

    geojson = build_timestamped_geojson(points)

    TimestampedGeoJson(
        data=geojson,
        # time-step resolution (adjust if timestamps are sparse)
        period="PT1S",
        transition_time=200,  # ms between frames
        add_last_point=True,
        auto_play=False,
        loop=False,
        max_speed=1,  # play speed multiplier
    ).add_to(m)

    m.save(OUTPUT_HTML)
    print(f"Animation written to {OUTPUT_HTML}")


if __name__ == "__main__":
    main()
