package gpxheatmap

import (
	"encoding/json"
	"fmt"
	"os"
	"strconv"
)

func extractHeatmapData(data *GPXData) ([][3]float64, error) {
	var heatmapData [][3]float64
	var prevMetricValue float64
	prevMetricValue = 0
	//for each point in the file
	for _, point := range data.Points {
		var metricValue float64 // get the metric we use for heatmapping, either power or HR (Power preferred)
		var err error

		//if power found, use that for metric
		if power, ok := point.Extensions["power"]; ok {
			metricValue, err = strconv.ParseFloat(power, 64)
			if err != nil {
				return nil, fmt.Errorf("invalid power value: %v", err)
			}
			prevMetricValue = metricValue
			//else if heartrate is found, use that
		} else if heartrate, ok := point.Extensions["heartrate"]; ok {
			metricValue, err = strconv.ParseFloat(heartrate, 64)
			if err != nil {
				return nil, fmt.Errorf("invalid heartrate value: %v", err)
			}
			prevMetricValue = metricValue

			//shouldn't not have one or the other, if so, just set value to previous metric. Initialized to 0, so if no
			//data for either, both will just be 0
		} else {
			metricValue = prevMetricValue
		}
		heatmapData = append(heatmapData, [3]float64{point.Latitude, point.Longitude, metricValue})
	}
	return heatmapData, nil
}

// Function generates a heat map and stores it in the outputPath file.
func GenerateHeatmap(data *GPXData, outputPath string) error {
	// get data from heatmapData
	heatmapData, err := extractHeatmapData(data)
	if err != nil {
		fmt.Println("Error generating heatmap data... GPXData struct not parsed")
		return err
	}

	jsonData, err := json.MarshalIndent(heatmapData, "", "    ")
	// var result string
	//parse into nice looking json
	// for _, point := range heatmapData {
	// 	result += fmt.Sprintf("    [%f, %f, %f],\n", point[0], point[1], point[2])
	// }
	// result = "[\n" + result + "\n]"
	//done parsing

	// Create output json file
	file, err := os.Create(outputPath)
	if err != nil {
		return fmt.Errorf("could not create data file: %v", err)
	}
	defer file.Close()

	// write json data to file
	_, err = file.Write(jsonData)
	if err != nil {
		return fmt.Errorf("could not write data to file: %v", err)
	}

	fmt.Printf("Heatmap data saved successfully to: %s\n", outputPath)
	return nil
}

// TODO: implement function
func GenerateHeatmapHTML(dataFilePath, outputPath string) error {
	htmlContent := fmt.Sprintf(
		`<!DOCTYPE html>
<html>
<head>
    <title>GPX Heatmap</title>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
    <script src="https://unpkg.com/leaflet.heat/dist/leaflet-heat.js"></script>
    <style>
        html, body, #map { height: 100%%; margin: 0; padding: 0; }
    </style>
</head>
<body>
    <div id="map"></div>
    <script>
		fetch('%s')
    .then(response => response.json())
    .then(data => {
        const map = L.map('map').setView([data[0][0], data[0][1]], 13);

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 18,
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);

        // Get min and max metric
        const metrics = data.map(p => p[2]);
        const minMetric = Math.min(...metrics);
        const maxMetric = Math.max(...metrics);

        // Convert a value to a color (blue to red)
        function getColor(value) {
            const t = (value - minMetric) / (maxMetric - minMetric); // 0 to 1
            const r = Math.round(255 * t);
            const g = Math.round(0);
            const b = Math.round(255 * (1 - t));
            return "rgb(${r},${g},${b})";
        }

        // Draw colored segments
        for (let i = 1; i < data.length; i++) {
            const p1 = data[i - 1];
            const p2 = data[i];
            const color = getColor(p1[2]);

            L.polyline([[p1[0], p1[1]], [p2[0], p2[1]]], {
                color: color,
                weight: 5,
                opacity: 0.8
            }).addTo(map);
        }

        // Center map on route
        const bounds = L.latLngBounds(data.map(p => [p[0], p[1]]));
        map.fitBounds(bounds);
    })
    .catch(error => console.error('Error loading data:', error));

    </script>
</body>
</html>
`, dataFilePath)
	file, err := os.Create(outputPath)
	if err != nil {
		return fmt.Errorf("Could not create HTML file: %v", err)
	}
	defer file.Close()

	_, err = file.WriteString(htmlContent)

	if err != nil {
		return fmt.Errorf("Could not write HTML content to file: %v", err)
	}

	fmt.Printf("Heatmap successfully generated at: %s \n", outputPath)
	return nil
}
