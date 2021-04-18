#include <cassert>
#include <vector>
namespace VectorOperations {
  std::vector<double> vecSum(std::vector<double> a, std::vector<double> b) {
    assert(a.size() == b.size());
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); i++) {
      result[i] = a[i] + b[i];
    }
    return result;
  }
}
