package main

import (
	"fmt"
	"github.com/jibb34/gpxheatmap"
	"log"
)

func main() {
	// gpxDir := "../python/output_files/gpx"
	gpxDir := "/home/jackjibb/Downloads"
	dataFile := gpxDir + "/gpxData.json"
	files, err := gpxheatmap.LoadGPXFiles(gpxDir)

	if err != nil {
		log.Fatal(err)
	}

	fmt.Printf("Found %d GPX files.\n", len(files))

	for _, file := range files {
		gpxData, err := gpxheatmap.ParseGPXFile(file)
		if err != nil {
			fmt.Printf("Error parsing %s: %v\n", file, err)
			continue
		}
		err = gpxheatmap.GenerateHeatmap(gpxData, dataFile)
		if err != nil {
			fmt.Printf("Error Generating Heatmap: %v\n", err)
			continue
		}
		err = gpxheatmap.GenerateHeatmapHTML("/gpxData.json", gpxDir+"/gpxData.html")
		if err != nil {
			fmt.Printf("Error Generating HTML File from heatmap: %v", err)
		}

	}

}
