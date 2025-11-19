#ifndef RCS_SO101_H
#define RCS_SO101_H

#include <rcs/Kinematics.h>
#include <rcs/Pose.h>
#include <rcs/utils.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace rcs {
namespace so101 {

class SO101IK : public rcs::common::Kinematics {
private:
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;
  const double eps_pos = 0.01;
  const double eps_ori = 10 * (M_PI / 180.0);
  int FRAME_ID;

  pinocchio::Model model;
  pinocchio::Data data;

 public:
  SO101IK(const std::string& path, const std::string& frame_id, bool urdf = true)
      : model() {
    if (urdf) {
      pinocchio::urdf::buildModel(path, this->model);
    } else {
      pinocchio::mjcf::buildModel(path, this->model);
    }
    this->data = pinocchio::Data(this->model);
    this->FRAME_ID = model.getFrameId(frame_id);
    if (FRAME_ID == -1) {
      throw std::runtime_error(
          frame_id + " frame id could not be found in the provided URDF");
    }
  }
  std::optional<rcs::common::VectorXd> inverse(
      const rcs::common::Pose& pose, const rcs::common::VectorXd& q0,
      const rcs::common::Pose& tcp_offset =
          rcs::common::Pose::Identity()) override {
      // --- Tunables (could be class members) ------------------------------------
    const double WP = 1.0;        // position weight
    const double WO = 0.12;       // orientation (pitch+yaw) weight
    const double WO_ROLL = 0.03;  // orientation roll weight (let it "float" more)
    const double ORI_TOL = 5.0 * M_PI / 180.0;  // 5 deg tolerance (dead-zone)
    const double ORI_CAP =
        0.6;  // cap on |eo| scaling (rad/s-equivalent per iter)
    const double STEP_CAP =
        0.15;  // cap on ||J * v|| (meters/rad) per iter to avoid jumps
    // If you already adapt damping elsewhere, keep using this->damp. Otherwise:
    const double DAMP_BASE = this->damp;  // your existing scalar
    // --------------------------------------------------------------------------

    rcs::common::Pose new_pose = pose * tcp_offset.inverse();
    rcs::common::VectorXd q(model.nq);
    q.setZero();
    q.head(q0.size()) = q0;

    const pinocchio::SE3 oMdes(new_pose.rotation_m(), new_pose.translation());

    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    bool success = false;

    // Pre-allocations
    rcs::common::Vector6d err, err_w;                // 6x1
    Eigen::Vector3d ep, eo;             // position/orientation parts
    Eigen::VectorXd v(model.nv);        // nv x 1
    pinocchio::Data::Matrix6 JJt;       // 6x6
    Eigen::Matrix<double, 6, 6> Wsqrt;  // sqrt of weights

    // Build constant sqrt-weights (diag): [WP, WP, WP, WO, WO, WO_ROLL]
    Wsqrt.setZero();
    Wsqrt(0, 0) = std::sqrt(WP);
    Wsqrt(1, 1) = std::sqrt(WP);
    Wsqrt(2, 2) = std::sqrt(WP);
    Wsqrt(3, 3) = std::sqrt(WO);
    Wsqrt(4, 4) = std::sqrt(WO);
    Wsqrt(5, 5) = std::sqrt(WO_ROLL);

    for (int i = 0;; ++i) {
      // FK + current frame pose
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      // Error in frame tangent space (twist): [trans; rot]
      const pinocchio::SE3 iMd = data.oMf[this->FRAME_ID].actInv(oMdes);
      err = pinocchio::log6(iMd).toVector();

      // Split and shape errors
      ep = err.head<3>();
      eo = err.tail<3>();

      // ---- Orientation tolerance (dead-zone + soft cap) ----------------------
      // If |eo| is small, ignore it completely (prevents twitch).
      const double eo_norm = eo.norm();
      if (eo_norm < ORI_TOL) {
        eo.setZero();
      } else {
        // shrink just the "excess" beyond tolerance (preserve direction)
        const double scaled = std::min(eo_norm - ORI_TOL, ORI_CAP);
        eo *= (scaled / eo_norm);
      }
      // ------------------------------------------------------------------------

      // Re-pack the shaped error
      err.head<3>() = ep;
      err.tail<3>() = eo;

      // Convergence test emphasizes position because eo may be zeroed
      if (ep.norm() < this->eps_pos && eo.norm() < this->eps_ori) {
        success = true;
        break;
      }
      if (i >= this->IT_MAX) {
        success = false;
        break;
      }

      // 6xnv body Jacobian for the frame
      pinocchio::computeFrameJacobian(model, data, q, this->FRAME_ID, J);

      // Map to log space (same as your original)
      pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(iMd.inverse(), Jlog);
      J = -Jlog * J;  // tangent-space Jacobian for the error you’re minimizing

      // ---- Weighted damped least-squares -------------------------------------
      // Apply sqrt-weights on rows: this prioritizes translation > orientation,
      // and de-weights tool roll further.
      const auto Jw = Wsqrt * J;  // 6xnv
      err_w = Wsqrt * err;        // 6x1

      // Optional: adapt damping using position manipulability (safer near
      // singularities)
      double damp = DAMP_BASE;
      {
        // Use position block for manipulability (3xnv)
        Eigen::Matrix<double, 3, Eigen::Dynamic> Jp = J.topRows<3>();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            Jp, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const double sigma_min =
            std::max(1e-9, svd.singularValues().tail<1>()(0));
        // Increase damping when near singular (simple schedule)
        damp = DAMP_BASE + 0.01 / sigma_min;
      }

      // Normal equations on weighted system
      JJt.noalias() = Jw * Jw.transpose();
      JJt.diagonal().array() += damp;

      // Solve for v (joint velocity increment)
      v.noalias() = -Jw.transpose() * JJt.ldlt().solve(err_w);
      // ------------------------------------------------------------------------

      // ---- Step-size limiter in task space to avoid sudden jumps -------------
      // Predict task step and scale if too large
      Eigen::Matrix<double, 6, 1> task_step = J * v * this->DT;
      double step_norm = task_step.head<3>().norm() + task_step.tail<3>().norm();
      if (step_norm > STEP_CAP && step_norm > 1e-9) {
        const double scale = STEP_CAP / step_norm;
        v *= scale;
      }
      // Also respect per-joint velocity limits if you have them (not shown)
      // ------------------------------------------------------------------------

      // Integrate
      q = pinocchio::integrate(model, q, v * this->DT);
    }

    if (success) {
      return q.head(q0.size());
    }
    // std::cout << "IK failed after " << this->IT_MAX << " iterations." <<
    // std::endl;
    std::cout << "ep: " << ep.norm() << " eo: " << eo.norm() << std::endl;
    return std::nullopt;
  }
  rcs::common::Pose forward(const rcs::common::VectorXd& q0,
                            const rcs::common::Pose& tcp_offset) override {
    // pose is assumed to be in the robots coordinate frame
    rcs::common::VectorXd q(model.nq);
    q.setZero();
    q.head(q0.size()) = q0;
    pinocchio::framesForwardKinematics(model, data, q);
    rcs::common::Pose pose(data.oMf[this->FRAME_ID].rotation(),
                          data.oMf[this->FRAME_ID].translation());

    // apply the tcp offset
    return pose * tcp_offset.inverse();
  }
};

}  // namespace so101
}  // namespace rcs

#endif  // RCS_SO101_H
