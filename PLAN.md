1.) Route Plotting Interface
  - React for front end
  - Leaflet.js or Mapbox GL JS for map graphics.
  - Waypoint generation and drawing
  - .gpx integration to upload and parse routes.
2.) Data Collection and Storage
  - aggregate anonymous data for training algorithm
  - keep track of all routes globally? a lot of storage might be required. Could find a way to optimise? (Only analyse major areas? Allow local storage on devices for individual users?)

3.) Training Suitability Algorithm (Focus of Project)
  - algorithm trained on training files (like .fit) to fine tune the algorithm
  - Input data: GPX file
  - output scoring index file (.dex)
  - Scoring logic: 
    - analyse segments of time between gpx points, and integrate power stability, elevation change, cadence consistency, and other metrics that could be used to determine a specific training suitability.
  - Scoring categories: Create various scoring categories to score a route based on specific needs
    - Endurance
    - Exploration
    - Interval
    - Race
    - Safety
    - others can be added as necessary
4.) Visualisation/Feedback
  - scoring overlays: Highlight segments by color or allow common segments to be clicked on to discover their ratings.
  - visualisation must be a normal distribution of average power of a ride
  - this section will need work
5.) Web Application Framework
  - Front end in React with Leaflet.js or Mapbox Gl JS
  - Back end in Go to provide endpoints for the crud operations, and handling of gpx files
  - interface with a dedicated server running the algorithm to generate a training suitability score for each segment.
  - Database in SQLite, allowing storage of user-generated routes, training metrics, and analysis results.
  - Traditional software methodologies can be used to generate the database
6.) Testing/Evaluation 
  - implement strategies for improving algorithm against user feedback
  - Physical excursions to routes to determine accuracy
7.) CI/CD Strategies
  - local development in docker containers for back end and front end, with a test database
  - production deployment using a cloud service (aws or azure)
  - Nginx reverse proxy for serving the application.
  - Github version control and deployment through standard DevOps practices
  - write documentation along with the code

Problems to solve:
  - How are we going to gather data for accuracy?
  - How do we determine what a "segment" is defined as. *** Key question... As once we can meaningfully define segments, it is trivial to assign values to it.
  - GPX path may not perfectly align with OSM roads due to GPS drift or other inaccuracies (could implement snapping   algorithm) R tree spacial indexing?
  - if segments stored as units, their granularity needs to be well defined
  - too many small segments = performance issues, too few = loss of detail
  - Connection of 2 segments may not occur edge-to-edge since routes should be fully flexible.
  - how to ensure continuation of metrics when splitting a segment?
  - how to build a route from a segment to find valid paths between 2 disconnected segments?
  - how to define interesctions?
  - how to handle intersecting segments
  - as collection of data increases, segment queries will become slower.
  


Ignore this, my stream of thought:


So the general idea will be to be able to generate "artificial" gpx files from a collection of data points. We need to
figure out a way to determine what a "segment" is in terms of cycling, should it be any intersection? how do we define
intersections? The "segment" will be a key entity I believe, as a segment will be a building block of routes

gpx trkpt -> gpx trkseg -> segment -> route

we can maybe cross reference gpx data with OpenStreetMap data through Overpass API? this would allow us to determine when we approach an intersection (one example of the end of a route). We also need to not split if an intersection is not chosen (go straight on).

Potenti


Python Problems:
Extract all data from various sources: communicating with apis, open files, webscraping?
save data to files
section merging
algorithm development
section detection and cleaning
clustering
creating visualisations
other data manipulation:

Go Problems:
File generation
spatial hashing, R-tree processes
performance tasks
Parse and standardize GPX files
back end server/api
database management with SQLite
route-section mapping
routing section/route metadata to the front end
concurrent processing on multiple or heavy route files
