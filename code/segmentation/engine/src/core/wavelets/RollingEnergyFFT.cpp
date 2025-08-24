#include "RollingEnergyFFT.hpp"
#include "core/wavelets/WaveletFootprint.hpp"
#include <algorithm>
#include <iostream>
#include <vector>

namespace wf {

// Computes k_min, k_max from [λ_min, λ_max]
static inline int bin_from_lambda(double N, double ds, double lambda_m) {
  /* TODO: Map wavelength to FFT bin index k.
  NOTE:
  Window Length L_win = N⋅Δs
  Spatial Frequency f=1/λ
  Discrete bin k ≈ f⋅N⋅Δs = L_win/λ
  */
  return (int)std::lround((N * ds) / std::max(lambda_m, 1e-9));
}

// NOTE: Constructor: define one fft plan to be used for each index in U[n]
RollingEnergyCalculator::RollingEnergyCalculator(int N, double ds,
                                                 double lambda_min_m,
                                                 double lambda_max_m)
    : ds_(ds), in_(nullptr), out_(nullptr), plan_(nullptr), h2sum_(0.0) {
  N_ = std::max(5, (N | 1)); // force odd, >=5
  N_half_ = N_ / 2;

  // Clamp band into [1, N_/2]. This removes DC offset and clamps to Nyquist
  // (Nyquist = Sampling rate / 2)
  // k = N * ds / λ_k
  int k_min = std::max(1, (int)std::ceil((N_ * ds_) / lambda_max_m));
  int k_max = std::min(N_ / 2, (int)std::floor((N_ * ds_) / lambda_min_m));
  // safeguard against swapping inputs, min is always min, max is always the max
  kmin_ = std::clamp(k_min, 1, N_ / 2);
  kmax_ = std::clamp(k_max, 1, N_ / 2);

  // allocate space for fftw data with aligned arrays (good for SIMD)
  in_ = (double *)fftw_malloc(sizeof(double) * N_);
  out_ = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * (N_ / 2 + 1));
  /* Create FFTW Plan:
   * r2c_1d = real -> complex, in 1 dimension.
   * uses transform size N_, and pointers in_ and out_ to precompute
   * an FFT strategy. FFTW_MEASURE tests a few algorithms to pick the fastest.
   * Takes longer to spin up, but is much faster to execute on the data series.
   */
  plan_ = fftw_plan_dft_r2c_1d(N_, in_, out_, FFTW_MEASURE);

  // build hann window h[N_] for tapering edges, and precompute sum of squares
  // of h for normalization.
  makeHann();
  (void)N;
  (void)ds;
  (void)lambda_max_m;
  (void)lambda_max_m;
}

// NOTE:: Destructor
RollingEnergyCalculator::~RollingEnergyCalculator() {

  // clear up memory for all c objects from fftw.
  if (plan_)
    fftw_destroy_plan(plan_);
  if (out_)
    fftw_free(out_);
  if (in_)
    fftw_free(in_);
}

void RollingEnergyCalculator::makeHann() {
  hann_.resize(N_);
  h2sum_ = 0.0;
  // in case where size is single or 0 samples, assign the hann the value 1:
  if (N_ <= 1) {
    hann_.assign(N_, 1.0);
    h2sum_ = (double)N_;
    return;
  }
  // compute hann function over N_, as well as the sum of hann squares.
  // h[n] = 1/2 (1- cos(2πn/N)) for 0<=n<=N
  for (int n = 0; n < N_; ++n) {
    const double v = 0.5 * (1.0 - std::cos(2.0 * M_PI * n / (N_ - 1)));
    hann_[n] = v;
    h2sum_ += v * v;
  }
}

void RollingEnergyCalculator::detrendWindowReflected(const UniformSignal &U,
                                                     int i) {
  // Prepare N_-length window around sample i by edge-reflection
  const int N_signal = static_cast<int>(U.y.size());
  const int half = N_ / 2;

  auto reflect = [&](int idx) {
    if (idx < 0)
      return -idx - 1;
    if (idx >= N_signal)
      return 2 * N_signal - 1 - idx;
    return idx;
  };

  // Fill in_[] with reflected data
  const int start = i - half;
  for (int n = 0; n < N_; ++n) {
    int j = reflect(start + n);
    in_[n] = U.y[j];
  }

  //=== Polynomial detrending via Eigen least-squares ===
  // Build Vandermonde matrix V (size N_ x 4) and vector y
  Eigen::MatrixXd V(N_, 3);
  Eigen::VectorXd y(N_);
  for (int n = 0; n < N_; ++n) {
    double x = (n - half) * ds_;
    V(n, 0) = 1.0;
    V(n, 1) = x;
    V(n, 2) = x * x;
    y(n) = in_[n];
  }
  // Solve V * coeffs = y in least-squares sense
  Eigen::VectorXd coeffs = V.colPivHouseholderQr().solve(y);
  double a = coeffs(0);
  double b = coeffs(1);
  double c = coeffs(2);

  //=== Apply polynomial detrend and window taper ===
  for (int n = 0; n < N_; ++n) {
    double x = (n - half) * ds_;
    double trend = a + b * x + c * x * x;
    in_[n] = (in_[n] - trend) * hann_[n];
  }
}

void RollingEnergyCalculator::compute_FFT(const UniformSignal &U,
                                          std::vector<double> &E) {

  // perform the FFT on the particular window
  if (U.y.empty() || ds_ <= 0 || U.y.size() != E.size()) {
    std::fill(E.begin(), E.end(), 0);
    std::cerr << "[ERROR] Shape error on computing Rolling Energy array: "
              << U.y.size() << " vs " << E.size() << "\n"
              << "ds_" << ds_ << "\n";
  }
  // our hann taper normalisation
  const double norm = 1.0 / std::max(h2sum_ * (double)N_, 1e-12);

  for (int i = 0; i < (int)U.y.size(); ++i) {
    // prepare window and fill in_ pointer
    detrendWindowReflected(U, i);
    // compute FFT on each window centred at i
    fftw_execute(plan_);
    double P_band = 0;
    // one-sided bins
    for (int k = kmin_; k <= kmax_; ++k) {
      // X[k] = re + j im: Magnitude^2 = real^2 + imaginary^2
      const double re = out_[k][0], im = out_[k][1];
      double mag2 = re * re + im * im;
      if (k != 0 && k != N_ / 2)
        // double all values except for DC and nyquist.
        mag2 *= 2.0; // one sided doubling
      // accumulate band power: (unnormalized)
      P_band += mag2;
    }

    E[i] = (P_band / double(kmax_ - kmin_ + 1)) * norm;
    // for that index
  }
}
int RollingEnergyCalculator::reflect_index(int idx, int N_signal) const {
  if (idx < 0)
    return -idx - 1;
  if (idx >= N_signal)
    return 2 * N_signal - 1 - idx;
  return idx;
}

void RollingEnergyCalculator::fillWindow(const UniformSignal &U, int center,
                                         std::vector<double> &w) const {
  int N_signal = (int)U.y.size();
  w.resize(N_);
  for (int n = 0; n < N_; ++n) {
    int j = reflect_index(center - N_half_ + n, N_signal);
    w[n] = U.y[j];
  }
}
void RollingEnergyCalculator::compute_RMS(const UniformSignal &U,
                                          std::vector<double> &E) {
  int L = U.y.size();
  E.resize(L);
  std::vector<double> window;
  for (int i = 0; i < L; ++i) {
    fillWindow(U, i, window);
    double Ei;
    compute_RMS_Energy(window, Ei);
    E[i] = Ei;
  }
}

inline void
RollingEnergyCalculator::compute_RMS_Energy(const std::vector<double> &w,
                                            double &outE) const {
  // Build Vandermonde for [1,x]
  Eigen::MatrixXd V(N_, 2);
  Eigen::VectorXd y(N_);
  for (int n = 0; n < N_; ++n) {
    double x = (n - N_half_) * ds_;
    V(n, 0) = 1.0;
    V(n, 1) = x;
    y(n) = w[n];
  }
  // solve for [b,m]
  Eigen::Vector2d coeffs = V.colPivHouseholderQr().solve(y);
  double b = coeffs(0);
  double m = coeffs(1);

  // RMS residuals
  double sum2 = 0;
  for (int n = 0; n < N_; ++n) {
    double x = (n - N_half_) * ds_;
    double r = w[n] - (b + m * x);
    sum2 += r * r;
  }
  outE = std::sqrt(sum2 / static_cast<double>(N_));
}

} // namespace wf
  //
