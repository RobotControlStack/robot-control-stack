#include "ik.h"
#include "rl/math/Matrix.h"
#include "rl/math/Rotation.h"
#include "rl/math/Transform.h"
#include "rl/math/Vector.h"
#include "rl/mdl/Dynamic.h"
#include "rl/mdl/Model.h"
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/UrdfFactory.h>
#include <stdexcept>
#include <stdio.h>

using namespace std;
using namespace rl::mdl;
using namespace rl::math;

struct ik_model {
  Model *m;
  JacobianInverseKinematics *ik;
};

IK ik_load(char *filename) {
  UrdfFactory factory;
  Dynamic *model = new Dynamic();
  factory.load(filename, model);
  JacobianInverseKinematics *ik =
      new JacobianInverseKinematics((Kinematic *)model);
  IK m = new ik_model();
  m->m = model;
  m->ik = ik;
  return m;
}

void ik_free(IK model) { delete (model); }

void ik_solve(IK model, rl::math::Transform transform, double *result) {
  model->ik->addGoal(transform, 0); // What is the second parameter?
  bool success = model->ik->solve();
  if (success) {
    rl::math::Vector q(6);
    q = model->m->getPosition();
    for (int i = 0; i < q.size(); ++i)
      result[i] = q[i];
  }
}
