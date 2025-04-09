package gpxheatmap

import (
	"fmt"
	"github.com/tkrajina/gpxgo/gpx"
	"os"
	"path/filepath"
	"strings"
	"time"
)

// GPXData will hold parsed data
type GPXData struct {
	Points       []GPXPoint //gpx track point
	AveragePower float64    // average power metric
	AverageHR    float64    // average hr metric
}

type GPXPoint struct {
	Time       time.Time
	Latitude   float64
	Longitude  float64
	Elevation  float64
	Extensions map[string]string
}

func LoadGPXFiles(dir string) ([]string, error) {
	//TODO: implement load gpx files
	var files []string
	// walk through each file in the path
	err := filepath.Walk(dir, func(path string, info os.FileInfo, err error) error {
		if err != nil {
			return err
		}
		// for each file that ends in .gpx, add it to the files list
		if !info.IsDir() && filepath.Ext(path) == ".gpx" {
			files = append(files, path)
		}
		return nil
	})
	if err != nil {
		return nil, err
	}
	//return files list
	return files, nil
}

func ParseGPXFile(filename string) (*GPXData, error) {

	//initialize parser object and parse our file
	gpxFile, err := gpx.ParseFile(filename)

	if err != nil {
		return nil, err
	}
	// init variables for parsing
	var points []GPXPoint
	var totalPower float64
	var countPower int
	var totalHR float64
	var countHR int

	//perform the parsing
	for _, track := range gpxFile.Tracks {
		for _, segment := range track.Segments {
			for _, point := range segment.Points {
				var gpxPoint GPXPoint //define current point

				gpxPoint.Latitude = point.Latitude
				gpxPoint.Longitude = point.Longitude
				gpxPoint.Elevation = point.Elevation.Value()
				gpxPoint.Time = point.Timestamp

				// get extensions
				if len(point.Extensions.Nodes) > 0 {
					gpxPoint.Extensions = make(map[string]string)
					/*NOTE: recursively parse extensions so nested extensions are handled as:
					* <ext1><ext2>value</ext2></ext1> => extlvl1/extlvl2 = value
					 */
					parseRecursiveExtensions(point.Extensions.Nodes, &gpxPoint, "")
				}
				points = append(points, gpxPoint)
			}
		}
	}
	// if countHR == 0 && countPower == 0 {
	// 	return nil, fmt.Errorf("No power or heart rate data found in %s", filename)
	// }
	avgPower := totalPower / float64(countPower)
	avgHR := totalHR / float64(countHR)

	return &GPXData{Points: points, AveragePower: avgPower, AverageHR: avgHR}, nil
}

func parseRecursiveExtensions(nodes []gpx.ExtensionNode, gpxPoint *GPXPoint, prefix string) {
	for _, node := range nodes {
		key := prefix + node.XMLName.Local
		value := strings.TrimSpace(node.Data)
		if value != "" {
			gpxPoint.Extensions[key] = value
			fmt.Printf("Found extension: %s -> %s\n", key, value)
		} else {
			if len(node.Nodes) > 0 {
				newPrefix := key + "/"
				parseRecursiveExtensions(node.Nodes, gpxPoint, newPrefix)
			}
		}

	}
}
