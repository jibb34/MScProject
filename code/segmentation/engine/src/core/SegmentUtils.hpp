#include "models/OsrmResponse.hpp"

struct ENU {
  double x;
  double y;
};
struct Curvature {
  std::vector<double> kappa; // instantaneous curvature values
  double pct_twisty;         // % distance above threshold
  double cbi;                // curvature burden index
};

class SegmentUtils {
public:
  bool checkForJunction(const Leg &l);
  // Displacement is the as-the-crow-flies distance between gpx points, not the
  // road length
  double calculateGradient(const std::vector<GpxPoint> &points);
  double calculateGradientVariation(const std::vector<GpxPoint> &points);
  Curvature calculateCurvature(const std::vector<GpxPoint> &points,
                               double critical_radius);
  double calculateHeadingDegToRad(const GpxPoint &point_from,
                                  const GpxPoint &point_to);
  // haversine formulas
  static double haversine(double x1, double x2, double y1, double y2);
  static double haversine(const GpxPoint &p1, const GpxPoint &p2);
  // private:
};
