#include <math.h>
#include <mujoco/mujoco.h>
#include <stdbool.h>

mjtNum target_joint_angles[7] = {
  0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4
};

bool target_joint_angles_updated = false;

// Set target_joint_angles then set 
void joint_angle_controller(const mjModel *m, mjData *d) {
  double min, max, diff;
  if (target_joint_angles_updated) {
    for (int i = 0; i < 7; ++i)
      d->ctrl[i] = target_joint_angles[i];
  }
}
