import os
import json
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors


def main():
    # TEST: QUICK TEST TO CHECK WE HAVE THE RIGHT MAP

    directory_path = "./data/results"
    temp_path = "./data/temp"
 # Get all match result files
    files = sorted([
        f for f in os.listdir(directory_path)
        if f.startswith("match_result_") and f.endswith(".json")
    ])
    temp_files = sorted([
        f for f in os.listdir(temp_path)
        if f.startswith("match_chunk") and f.endswith(".json")

    ])

    if not files:
        raise FileNotFoundError(
            f"No match_result_*.json files found in {directory_path}")

    # Color map
    num_files = len(files)
    # colormap = cm.coolwarm
    colors = ["tab:blue", "tab:orange"]

    # Plotting
    plt.figure(figsize=(10, 10))
    for idx, filename in enumerate(files):
        file_path = os.path.join(directory_path, filename)
        with open(file_path, "r") as f:
            data = json.load(f)

        # color = colormap(idx / max(1, num_files - 1))
        color = colors[idx % len(colors)]

        for segment in data.get("matchings", []):
            coords = segment["geometry"]["coordinates"]
            lats, lons = zip(*[(lat, lon) for lon, lat in coords])
            plt.plot(lons, lats, '-', color=color,
                     label=f"{filename}")

    plt.title("OSRM Matched Segments by Chunk")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
