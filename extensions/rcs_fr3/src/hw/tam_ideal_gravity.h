#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace adaptor {

// Gravity torque of the TAM ideal model — the MJCF the checkpoint was trained
// against — at a measured arm configuration.
//
// TAM's training convention is a controller that compensates gravity with the
// *ideal* model; a real robot compensates with its own model instead. Feeding
// the plant ``PD + g_ideal + residual`` (i.e. commanding ``g_ideal - g_robot``
// on top of the robot's own compensation) restores that convention, so the
// adaptor only has to absorb genuine plant differences and its target is the
// ideal model under exact compensation regardless of the robot's configured
// end-effector load.
//
// The shipped ideal MJCF carries ``gravcomp="1"`` bodies (a neutral asset);
// the checkpoints' "real-gravity" ideal model zeroes ``body_gravcomp``
// (simadaptor core/structs.py), which this helper mirrors.
//
// Owned and used by the control thread only. ``gravity()`` costs ~10-20 us
// (kinematics + CoM + RNE at rest on a 9-DoF model).
class IdealModelGravity {
 public:
  explicit IdealModelGravity(const std::string& xml_path) {
    char err[1024] = {0};
    m_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
    if (m_ == nullptr) {
      throw std::runtime_error(std::string("IdealModelGravity: ") + err);
    }
    for (int b = 0; b < m_->nbody; ++b) {
      m_->body_gravcomp[b] = 0.0;
    }
    int n = 0;
    for (int j = 0; j < m_->njnt && n < 7; ++j) {
      if (m_->jnt_type[j] == mjJNT_HINGE) {
        qpos_adr_[n] = m_->jnt_qposadr[j];
        dof_adr_[n] = m_->jnt_dofadr[j];
        ++n;
      }
    }
    if (n != 7) {
      mj_deleteModel(m_);
      m_ = nullptr;
      throw std::runtime_error(
          "IdealModelGravity: the ideal model needs 7 hinge joints (arm)");
    }
    d_ = mj_makeData(m_);
    mj_resetData(m_, d_);  // non-arm joints (fingers) stay at qpos0
    rne_.assign(static_cast<size_t>(m_->nv), 0.0);
  }
  ~IdealModelGravity() {
    if (d_ != nullptr) mj_deleteData(d_);
    if (m_ != nullptr) mj_deleteModel(m_);
  }
  IdealModelGravity(const IdealModelGravity&) = delete;
  IdealModelGravity& operator=(const IdealModelGravity&) = delete;

  // Gravity torque on the 7 arm joints at configuration ``q`` (qvel = 0).
  std::array<double, 7> gravity(const std::array<double, 7>& q) {
    for (int j = 0; j < 7; ++j) {
      d_->qpos[qpos_adr_[j]] = q[j];
    }
    for (int i = 0; i < m_->nv; ++i) {
      d_->qvel[i] = 0.0;
    }
    mj_kinematics(m_, d_);
    mj_comPos(m_, d_);
    mj_rne(m_, d_, /*flg_acc=*/0, rne_.data());  // C(q,0) + g(q) = g(q)
    std::array<double, 7> g{};
    for (int j = 0; j < 7; ++j) {
      g[j] = rne_[static_cast<size_t>(dof_adr_[j])];
    }
    return g;
  }

 private:
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int qpos_adr_[7] = {0};
  int dof_adr_[7] = {0};
  std::vector<mjtNum> rne_;
};

}  // namespace adaptor
