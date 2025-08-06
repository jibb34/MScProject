import os
import json
import folium
from itertools import cycle
import geopandas as gpd


def load_roads(osm_file):
    """
    Load road features from the 'lines' layer of an OSM-XML file.
    """
    try:
        gdf = gpd.read_file(osm_file, layer='lines')
    except Exception as e:
        raise RuntimeError(
            f"Failed to load 'lines' layer from {osm_file}: {e}")
    # Filter to highways if available
    if 'highway' in gdf.columns:
        roads = gdf[gdf['highway'].notna()]
    else:
        roads = gdf
    if roads.empty:
        raise RuntimeError("No road features found in the input file.")
    return roads


def main():
    # Directory containing match result JSON files
    directory = "./data/results"
    chunk_dir = "./data/temp"
    osm_file = "data/osm_files/map.osm"
    roads = load_roads(osm_file)
    # Collect all match_result_*.json files
    files = sorted([
        f for f in os.listdir(directory)
        if f.startswith("match_result_") and f.endswith(".json")
    ])
    chunk_files = sorted([
        f for f in os.listdir(chunk_dir)
        if f.startswith("match_chunk_") and f.endswith(".json")
    ])

    if not files:
        raise FileNotFoundError(
            f"No match_result_*.json files found in {directory}")

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
    minx, miny, maxx, maxy = roads.total_bounds
    center = [(miny + maxy) / 2, (minx + maxx) / 2]
    m = folium.Map(location=center, tiles='cartodbpositron')

    folium.GeoJson(roads.to_json(), name='roads', style_function=lambda x: {
                   'color': "#006400", 'weight': 1}).add_to(m)
    # Define two alternating colors
    colors = ['#6A5ACD', '#FFD700']
    color_cycle = cycle(colors)

    # Add each chunk as a PolyLine with alternating color
    for filename, chunk_file in zip(files, chunk_files):
        color = next(color_cycle)
        with open(os.path.join(chunk_dir, chunk_file)) as f:
            chunk_data = json.load(f)
        raw_pts = chunk_data.get("coordinates", [])
        # Plot coordinates of raw points to compare
        for lon, lat in raw_pts:
            folium.CircleMarker(
                location=[lat, lon],
                radius=1,
                color=color,
                fill=True,
                fill_opacity=0.5,
            ).add_to(m)

    # Save to HTML
    output_file = "interactive_map.html"
    m.save(output_file)
    print(f"Saved interactive map to {output_file}")


if __name__ == "__main__":
    main()
