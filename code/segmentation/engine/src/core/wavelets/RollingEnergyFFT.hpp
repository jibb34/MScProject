#pragma once
#include "WaveletFootprint.hpp"
#include <Eigen/Dense>
extern "C" {
#include <fftw3.h>
}
#include <vector>

namespace wf {
class RollingEnergyCalculator {

public:
  RollingEnergyCalculator(int N, double ds, double lambda_min_m,
                          double lambda_max_m);
  ~RollingEnergyCalculator();
  void compute(const UniformSignal &U, std::vector<double> &E,
               bool isFFT) const;

  void compute_FFT(const UniformSignal &U, std::vector<double> &E);
  void compute_RMS(const UniformSignal &U, std::vector<double> &E);
  // Compute E[i] for whole signal U (with reflect-padded edges, Hann and
  // detrending algos)

private:
  void makeHann();
  int reflect_index(int idx, int N_signal) const;
  void fillWindow(const UniformSignal &U, int center,
                  std::vector<double> &w) const;
  void detrendWindowReflected(const UniformSignal &U, int i);
  void compute_RMS_Energy(const std::vector<double> &w, double &outE) const;

  // inst vars

  int N_;
  int N_half_;
  double ds_;
  WaveletFootprintEngine::EnergyAlgorithm energy_mode_;
  int kmin_, kmax_;
  double h2sum_;

  // buffers that align for FFTW:
  double *in_;        // N_
  fftw_complex *out_; // N_/2 + 1
  std::vector<double> hann_;
  fftw_plan plan_;
};
} // namespace wf
