"""
visualize_osm.py

Load an OSM-XML (.osm) file, extract the road network, and render it as an interactive Folium map.

Dependencies:
    pip install geopandas folium

Usage:
    python visualize_osm.py /path/to/map.osm [-o map.html] [-z 12]
"""
import os
import sys
import argparse

import geopandas as gpd
import folium
from utils.gpx_utils import extract_data_from_gpx, load_gpx


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


def visualize(osm_file, output_html, zoom):
    roads = load_roads(osm_file)
    # Center map on roads bounding box
    minx, miny, maxx, maxy = roads.total_bounds
    center = [(miny + maxy) / 2, (minx + maxx) / 2]
    gpx = load_gpx("data/input/test.gpx")
    points = extract_data_from_gpx(gpx)

    m = folium.Map(location=center, zoom_start=zoom, tiles='cartodbpositron')
    folium.GeoJson(roads.to_json(), name='roads').add_to(m)
    fg = folium.FeatureGroup(name="GPX Points")
    for _, row in points.iterrows():
        lon, lat = row.geometry.x, row.geometry.y
        folium.CircleMarker(
            location=(lat, lon),
            radius=1,
            color="red",
            fill=True,
            fill_color="red"
        ).add_to(fg)
    fg.add_to(m)

    folium.LayerControl().add_to(m)
    m.save(output_html)
    print(f"Saved interactive map to {output_html}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Visualize roads from an OSM-XML file on a Folium map"
    )
    parser.add_argument('osm_file', help='Path to .osm file')
    parser.add_argument('-o', '--output', default='map.html',
                        help='Output HTML file')
    parser.add_argument('-z', '--zoom', type=int,
                        default=13, help='Initial zoom level')
    args = parser.parse_args()

    if not os.path.exists(args.osm_file):
        print(f"Error: file not found: {args.osm_file}")
        sys.exit(1)

    try:
        visualize(args.osm_file, args.output, args.zoom)
    except Exception as e:
        print(f"Failed to visualize roads: {e}")
        sys.exit(1)
