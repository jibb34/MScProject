#pragma once
#include "models/CoreTypes.hpp"
#include "models/SegmentModel.hpp"
// Main Function within the segmenter engine
//
//
// Segmenter class
//------------------------------------------------------------------------------
struct RouteSignal;
struct DataPoint;
class SegmentDB;
class ExistingSegmentMatcher;

class SegmentEngine {
  struct Params {};
  using index_t = std::size_t;
  explicit SegmentEngine(Params p = Params{}) : P(p) {};
  ~SegmentEngine() = default;

  void setParams(const Params &params);
  const Params &params() const noexcept;

  // Define other parameter configurations here:
  // ================================================

  // ================================================

  // Function to initialize the segmenter: loads params, and gets wavelet
  // functions running.
  void initialize();

  // Function to process the segmentation
  std::vector<SegmentInstance> processSegmentation(RouteSignal &signal,
                                                   SegmentDB &db) const;

  void reset() noexcept; // for clearing cache later

  // Function to get the segmentation results
  std::vector<DataPoint> getResults();

private:
  Params P;
  std::vector<Coordinate> buildPolyline(const RouteSignal &s) const;
};
