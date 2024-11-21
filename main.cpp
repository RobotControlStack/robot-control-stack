#include <iostream>
#include "mylib.h"

int main() {
  RelaxedIK* ik = relaxed_ik_new("./fr3.yaml");
  double target_pos[3] = {0.3, 0.2, 0.2};
  double target_quat[4] = {1, 0, 0, 0};
  double tolerances[6] = {0.1, 0.1, 0.1, 0.01, 0.01, 0.01 };
  auto ret = solve(ik, target_pos, 3, target_quat, 4, tolerances, 6);
  std::cout << ret.length << std::endl;
  for (size_t i = 0; i < ret.length; ++i) {
    std::cout << ret.data[i] << " ";
  }
  std::cout << std::endl;
  relaxed_ik_free(ik);
  return 0;
}
