#include <iostream>

#include "mylib.h"

int main() {
  RelaxedIK* ik = relaxed_ik_new("./fr3.yaml");
  double target_pos[3] = {0.2, 0.2, 0.2};
  double target_quat[4] = {0, 0, 0, 1};
  double tolerances[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  auto ret = solve_position(ik, target_pos, target_quat, 3, 4, tolerances, 6);
  std::cout << ret.length << std::endl;
  relaxed_ik_free(ik);
  return 0;
}
