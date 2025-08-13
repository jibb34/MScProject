
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
//------------------------------------------------------------------------------
// MergeCostManager: loads per-attribute merge costs from configuration
//------------------------------------------------------------------------------
class MergeCostManager {
public:
  // Load costs from a JSON or simple key=value file
  static void loadFromFile(const std::string &filepath);

  // Retrieve cost for a given attribute key from costs_
  // Throws exception if key not found in costs_
  static double getCost(const std::string &attribute) {
    auto it = costs_.find(attribute);
    if (it == costs_.end()) {
      throw std::runtime_error("Cost for attribute '" + attribute +
                               "' not found in costs map.");
    }
    return it->second;
  };

private:
  static std::unordered_map<std::string, double> costs_;
};
