
// Main Function within the segmenter engine
//
//
// Segmenter class
//------------------------------------------------------------------------------
class SegmentEngine {
public:
  SegmentEngine() = default;
  ~SegmentEngine() = default;

  // Function to initialize the segmenter
  void initialize();

  // Function to process the segmentation
  void processSegmentation();

  // Function to finalize the segmenter
  void finalize();

  // Function to get the segmentation results
  void getResults();

private:
};
