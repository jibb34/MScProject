#pragma once
#include "models/CoreTypes.hpp"
#include <cmath>
#include <functional>
#include <string>
#include <vector>

struct RouteSignal;
struct DataPoint;

struct UniformSignal {
  std::vector<double> s; // distances [m], uniform
  std::vector<double> y; // interpolated values
  double ds = 0.0;
};

struct WaveletPoint {
  double s_m; // cumulative distance [m], non-decreasing
  double y;   // signal value at that distance
};

using WaveletSignal = std::vector<WaveletPoint>;

enum class ValueKind {
  Scalar,              // plain double (e.g., elevation, speed, gradient)
  AngleRadians_WrapPi, // angles are wrapped to (-pi, pi]; we will UNWRAP to be
                       // continuous
  Vector               // duple pair of value and direction
};
enum class SeriesKind { Scalar, Angle, Categorical };
enum class MergeSide { Left, Right };
struct NamedUniform {
  std::string name;
  SeriesKind kind;
  UniformSignal sig;
};

struct NamedGetter {
  const char *name;
  SeriesKind kind;
  std::function<double(const DataPoint &)> get;
};

class WaveletFootprintEngine {
public:
  // for input radial values wrapped to [-π,π], we must unwrap the angle to
  // [0,2π]
  inline double unwrap_angle_step(double prev_unwrapped,
                                  double current_wrapped) const {
    const double PI = M_PI, TWO_PI = 2 * M_PI;
    double w = std::fmod(current_wrapped + PI, TWO_PI);
    if (w < 0)
      w += TWO_PI;
    w -= PI;
    double k = std::round((prev_unwrapped - w) / TWO_PI);
    return w + k * TWO_PI;
  }

  using GetterFn = std::function<double(const DataPoint &)>;

  // NEW: non-template declaration
  WaveletSignal make_wavelet_signal(const RouteSignal &rs,
                                    const GetterFn &getter, SeriesKind kind,
                                    double min_step_m = 0.0) const;

  // Resample signal to be uniform
  UniformSignal resample_uniform_scalar(const WaveletSignal &sig,
                                        double ds) const;
  UniformSignal resample_uniform_angle(const WaveletSignal &sig,
                                       double ds) const;

  UniformSignal resample_uniform_categorical(const WaveletSignal &sig,
                                             double ds) const;

  std::vector<NamedUniform>
  make_uniform_bundle(const RouteSignal &rs, double ds_m,
                      std::initializer_list<NamedGetter> defs,
                      double min_step_m = 0.0) const;

  std::vector<NamedUniform>
  make_uniform_bundle_default(const RouteSignal &rs, double ds_m,
                              double min_step_m = 0.0) const;

  // Wavelet Objects
  //============= Elevation Wavelet Analysis ====================
  enum class TerrainState : uint8_t { // Categories for Elevation behaviour
    Flat = 0,
    Uphill = 1,
    Downhill = 2,
    Rolling = 3,
    Unknown = 4
  };
  enum class EnergyAlgorithm { FFT = 0, RMS = 1 };

  // Parameters that can tune the Wavelet Footprint function for elevation
  struct TerrainParams {
    // Scale factors:
    double dx = 5.0; // sampling distance

    // Pair of L_T and L_H trend scales.
    double L_T = 400;

    EnergyAlgorithm energy_mode = EnergyAlgorithm::FFT;
    // FFT Params for Rolling-energy calculation
    // double lambda_min = 200.0;  // band pass lower bound
    // double lambda_max = 1000.0; // band pass upper bound
    double L_E = 2400.0; // window length for FFT

    // HAAR Params for rolling energy calcs
    std::vector<double> L_H = {10, 20, 40, 80};

    // Threshold multipliers:
    double k_g = 46;    // for tau_g
    double k_E = 78;    // for tau_E
    double tau_p = 1.5; // for p[i]

    // Post processing
    bool E_use_hysteresis = true;
    double min_run_length = 20; // minimum run length before allowed to change
    double maj_window_filter = 25; // majority filter window
    double E_env_m = 500;
    double E_hyst_hi = 2.0;      // high threshold in MADs
    double E_hyst_lo = 1.0;      // low threshold in MADs
    double E_gap_close_m = 5000; // close holes shorter than this
    double E_min_run_m = 80;     // discard short bursts

    double min_segment_length_m = 500.0; // UI: “Min segment (m)”
    bool merge_side_right = false;       // false=left, true=right
  };

  UniformSignal terrain_states_from_elevation(const RouteSignal &rs,
                                              const TerrainParams &p,
                                              std::vector<double> &E_out) const;
  std::vector<WaveletFootprintEngine::TerrainState> get_states() const;

  std::vector<WaveletFootprintEngine::TerrainState> smoothSegments(
      const std::vector<WaveletFootprintEngine::TerrainState> &states,
      const std::vector<double> &lengths, // same size as states
      double min_length, MergeSide side) const;

private:
  mutable std::vector<TerrainState> states_;
  /*
   * NOTE: Uses Haar wavelets in a dual-channel (known as wavelet footprints)
   * in order to calculate logical change-points for distance series of
   * elevation. Each Terrain state is logically derived as follows:
   * =============================================================
   * X[t] = Signal at time t
   * T = trend percentage - trend of the signal to continuously go up/downhill
   * A = Activity meters - the amount of meters that defines the Mean absolute
   *     deviation of a small window scale. Characterises "rolling" terrain
   * If |T| <= flat_eps_pct && A < activity_thr_m -> X[t] = flat
   * else if T >= slope_thr_pct && A < activity_thr_m -> X[t] = Uphill
   * else if t <= -slope_thr_pct && A<activity_thr_m -> X[t] = Downhill
   * else -> Rolling
   */
};
