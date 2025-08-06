import os
import argparse
import json
import folium
from itertools import cycle


def main():
    # Directory containing match result JSON files
    directory = "./data/results"

    # Collect all match_result_*.json files
    # files = sorted([
    #     f for f in os.listdir(directory)
    #     if f.startswith("match_result_") and f.endswith(".json")
    # ])
    files = [f for f in os.listdir(directory)]
    # chunk_files = sorted([
    #     f for f in os.listdir(chunk_dir)
    #     if f.startswith("match_chunk_") and f.endswith(".json")
    # ])

    if not files:
        raise FileNotFoundError(
            f"No .json files found in {directory}")

    # Compute an average center for the initial map view
    all_lats = []
    all_lons = []
    for filename in files:
        data = json.load(open(os.path.join(directory, filename)))
        for seg in data.get("matchings", []):
            for lon, lat in seg["geometry"]["coordinates"]:
                all_lats.append(lat)
                all_lons.append(lon)
    center = [sum(all_lats) / len(all_lats), sum(all_lons) / len(all_lons)]

    # Create Folium map

    # Define two alternating colors
    colors = ['blue', 'orange']
    color_cycle = cycle(colors)

    # Add each chunk as a PolyLine with alternating color
    for filename in files:
        m = folium.Map(location=center, zoom_start=13)
        color = next(color_cycle)
        data = json.load(open(os.path.join(directory, filename)))
        for seg in data.get("matchings", []):
            coords = [(lat, lon)
                      for lon, lat in seg["geometry"]["coordinates"]]
            folium.PolyLine(coords, color=color, weight=5,
                            opacity=0.8).add_to(m)
        # Plot coordinates of raw points to compare
        # for lon, lat in raw_pts:
        #     folium.CircleMarker(
        #         location=[lat, lon],
        #         radius=1,
        #         color=color,
        #         fill=True,
        #         fill_opacity=0.5,
        #     ).add_to(m)

        # Save to HTML
        output_file = f"interactive_map_{
            os.path.splitext(os.path.basename(filename))[0]}.html"
        m.save(output_file)
        print(f"Saved interactive map to {output_file}")


if __name__ == "__main__":
    main()
