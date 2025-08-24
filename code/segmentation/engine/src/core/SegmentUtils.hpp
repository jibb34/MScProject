#pragma once
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
  static double calculateGradient(const Coordinate &coord1,
                                  const Coordinate &coord2);
  static double
  calculateGradientVariation(const std::vector<DataPoint> &points);
  static Curvature calculateCurvature(const std::vector<DataPoint> &points,
                                      double critical_radius);
  static double calculateHeadingDegToRad(const Coordinate &point_from,
                                         const Coordinate &point_to);
  // haversine formulas
  static double haversine(double x1, double x2, double y1, double y2);
  static double haversine(const GpxPoint &p1, const GpxPoint &p2);
  static double haversine(const Coordinate &p1, const Coordinate &p2);
  static void compute_windowed_gradients(std::vector<DataPoint> &pts,
                                         double back_window_m = 20.0,
                                         double fwd_window_m = 20.0,
                                         double eps_m = 1e-6);
  static double ang_diff(double a, double b);

  // Compute + smooth speed in-place for rs.points
  static void compute_and_smooth_speed(
      std::vector<DataPoint> &pts,
      double tau_s = 2.0,  // EMA time-constant (s)
      int med_win = 5,     // odd window length for median (samples)
      double a_max = 2.5); // m/s^2 accel cap (bike-ish)

  // If you already computed raw speed elsewhere, you can call:
  static void smooth_speed(std::vector<DataPoint> &pts, double tau_s = 2.0,
                           int med_win = 5, double a_max = 2.5);

  static void
  normalize_heading_to_speed(std::vector<DataPoint> &pts,
                             double v0 = 2.5,   // knee (~9 km/h)
                             double vhi = 5.56, // falloff (~20 km/h)
                             double p = 2.0, double q = 2.0);
  // private:
};
