#include <cctype>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "core/wavelets/WaveletFootprint.hpp"
#include "json.hpp"

using json = nlohmann::json;

// ultra-light arg parser
struct Options {
  std::string input;           // --input uploads/your.json
  std::string signals = "all"; // --signals speed,elev,grad,head,way | all
  double ds = -1.0;            // --ds 5    (meters). If <0, auto from route
  bool dump_csv = false;       // --csv out.csv
  std::string csv_path;
  bool verbose = false;    // --verbose
  double min_step_m = 0.0; // pass to make_wavelet_signal (usually 0)
};

static Options parse(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; i++) {
    std::string a(argv[i]);
    auto next = [&](double &tgt) {
      if (i + 1 < argc)
        tgt = std::stod(argv[++i]);
      else
        std::cerr << "missing value after " << a << "\n";
    };
    auto nexts = [&](std::string &tgt) {
      if (i + 1 < argc)
        tgt = argv[++i];
      else
        std::cerr << "missing value after " << a << "\n";
    };
    if (a == "--input")
      nexts(o.input);
    else if (a == "--signals")
      nexts(o.signals);
    else if (a == "--ds")
      next(o.ds);
    else if (a == "--csv") {
      o.dump_csv = true;
      nexts(o.csv_path);
    } else if (a == "--verbose")
      o.verbose = true;
    else if (a == "--min-step")
      next(o.min_step_m);
    else {
      std::cerr << "Unknown arg: " << a << "\n";
    }
  }
  return o;
}

static void write_csv(const std::string &path,
                      const std::vector<NamedUniform> &nus) {
  if (nus.empty())
    return;
  // Assume all have same s grid (they should if same ds)
  const auto &base_s = nus.front().sig.s;
  std::ofstream out(path);
  out << "s_km";
  for (auto &nu : nus)
    out << "," << nu.name;
  out << "\n";

  const std::size_t m = base_s.size();
  for (std::size_t k = 0; k < m; k++) {
    out << (base_s[k] / 1000.0);
    for (auto &nu : nus) {
      double y =
          (k < nu.sig.y.size() ? nu.sig.y[k]
                               : std::numeric_limits<double>::quiet_NaN());
      // For convenience: convert speed to km/h in CSV too
      if (nu.name == std::string("speed"))
        y *= 3.6;
      out << "," << y;
    }
    out << "\n";
  }
  std::cerr << "[csv] wrote " << path << " rows=" << m
            << " cols=" << (nus.size() + 1) << "\n";
}

// int main(int argc, char **argv) {
//   Options opt = parse(argc, argv);
//   if (opt.input.empty()) {
//     std::cerr << "Usage: signal_lab --input <file.json> [--signals "
//                  "all|speed,elev,grad,head,way] [--ds 5] [--csv out.csv] "
//                  "[--verbose]\n";
//     return 2;
//   }
//
//   // 1) Load JSON and build RouteSignal
//   json j;
//   try {
//     std::ifstream in(opt.input);
//     if (!in) {
//       std::cerr << "Cannot open: " << opt.input << "\n";
//       return 2;
//     }
//     in >> j;
//   } catch (const std::exception &e) {
//     std::cerr << "JSON read/parse error: " << e.what() << "\n";
//     return 2;
//   }
//
//   OsrmResponse osrm;
//   try {
//     osrm = j.get<OsrmResponse>();
//   } catch (const std::exception &e) {
//     std::cerr << "OsrmResponse parse error: " << e.what() << "\n";
//     return 2;
//   }
//
//   RouteSignalBuilder builder;
//   RouteSignal rs = builder.build(osrm);
//   if (rs.points.size() < 2) {
//     std::cerr << "RouteSignal has too few points.\n";
//     return 3;
//   }
//
//   // quick cum_dist sanity
//   double total_m = rs.points.back().cum_dist;
//   std::cerr << "[route] points=" << rs.points.size() << " total=" << total_m
//             << " m (" << total_m / 1000.0 << " km)\n";
//
//   // 2) Pick ds (auto if not provided)
//   double ds = opt.ds > 0 ? opt.ds : choose_ds_from_route(rs);
//   std::cerr << "[resample] ds=" << ds << " m\n";
//
//   // 3) Build getter set and compute bundle
//   WaveletFootprintEngine eng;
//   auto uni = default_getters();
//   auto defs = select_getters(opt.signals, uni);
//
//   if (defs.empty()) {
//     std::cerr
//         << "No known signals selected (use --signals all or a CSV of
//         names)\n";
//     return 4;
//   }
//
//   auto bundle = compute_uniform_bundle(rs, eng, ds, defs, opt.min_step_m);
//
//   // 4) Print debug stats
//   std::cout << std::fixed << std::setprecision(6);
//   for (const auto &nu : bundle) {
//     print_uniform_stats(nu);
//   }
//
//   // 5) Optional CSV
//   if (opt.dump_csv)
//     write_csv(opt.csv_path, bundle);
//
//   return 0;
// }
