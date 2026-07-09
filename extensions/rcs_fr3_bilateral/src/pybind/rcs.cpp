#include <hw/BilateralFranka.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
  py::module_::import("rcs_fr3._core");

  m.doc() = R"pbdoc(
        Bilateral Franka Python Bindings
        --------------------------------
    )pbdoc";
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  auto hw = m.def_submodule("hw", "rcs bilateral franka module");

  py::enum_<rcs::hw::BilateralControlMode>(hw, "BilateralControlMode")
      .value("bilateral", rcs::hw::BilateralControlMode::bilateral)
      .value("gravity_only", rcs::hw::BilateralControlMode::gravity_only)
      .export_values();

  py::class_<rcs::hw::BilateralFrankaConfig>(hw, "BilateralFrankaConfig")
      .def(py::init(
               [](const rcs::hw::FrankaConfig& leader_cfg,
                  const rcs::hw::FrankaConfig& follower_cfg,
                  rcs::hw::BilateralControlMode control_mode,
                  double update_rate_hz, double follower_joint_position_scale,
                  double haptic_feedback_gain, double max_follower_joint_step,
                  bool relative_joint_mapping, bool leader_haptic_feedback,
                  rcs::common::Vector7d feedback_avoidance_alpha) {
                 rcs::hw::BilateralFrankaConfig cfg;
                 cfg.leader_cfg = leader_cfg;
                 cfg.follower_cfg = follower_cfg;
                 cfg.control_mode = control_mode;
                 cfg.update_rate_hz = update_rate_hz;
                 cfg.follower_joint_position_scale =
                     follower_joint_position_scale;
                 cfg.haptic_feedback_gain = haptic_feedback_gain;
                 cfg.max_follower_joint_step = max_follower_joint_step;
                 cfg.relative_joint_mapping = relative_joint_mapping;
                 cfg.leader_haptic_feedback = leader_haptic_feedback;
                 cfg.feedback_avoidance_alpha = feedback_avoidance_alpha;
                 return cfg;
               }),
           py::arg("leader_cfg"), py::arg("follower_cfg"),
           py::arg("control_mode") = rcs::hw::BilateralControlMode::bilateral,
           py::arg("update_rate_hz") = 1000.0,
           py::arg("follower_joint_position_scale") = 1.0,
           py::arg("haptic_feedback_gain") = 1.0,
           py::arg("max_follower_joint_step") = 0.05,
           py::arg("relative_joint_mapping") = true,
           py::arg("leader_haptic_feedback") = true,
           py::arg("feedback_avoidance_alpha") =
               (rcs::common::Vector7d() << 18.75, 15.0, 13.5, 9.0, 5.25, 3.0,
                1.5)
                   .finished())
      .def_readwrite("leader_cfg", &rcs::hw::BilateralFrankaConfig::leader_cfg)
      .def_readwrite("follower_cfg",
                     &rcs::hw::BilateralFrankaConfig::follower_cfg)
      .def_readwrite("control_mode",
                     &rcs::hw::BilateralFrankaConfig::control_mode)
      .def_readwrite("update_rate_hz",
                     &rcs::hw::BilateralFrankaConfig::update_rate_hz)
      .def_readwrite(
          "follower_joint_position_scale",
          &rcs::hw::BilateralFrankaConfig::follower_joint_position_scale)
      .def_readwrite("haptic_feedback_gain",
                     &rcs::hw::BilateralFrankaConfig::haptic_feedback_gain)
      .def_readwrite("max_follower_joint_step",
                     &rcs::hw::BilateralFrankaConfig::max_follower_joint_step)
      .def_readwrite("relative_joint_mapping",
                     &rcs::hw::BilateralFrankaConfig::relative_joint_mapping)
      .def_readwrite("leader_haptic_feedback",
                     &rcs::hw::BilateralFrankaConfig::leader_haptic_feedback)
      .def_readwrite("feedback_avoidance_alpha",
                     &rcs::hw::BilateralFrankaConfig::feedback_avoidance_alpha);

  py::class_<rcs::hw::BilateralFrankaState>(hw, "BilateralFrankaState")
      .def(py::init<>())
      .def_readonly("leader_q", &rcs::hw::BilateralFrankaState::leader_q)
      .def_readonly("leader_dq", &rcs::hw::BilateralFrankaState::leader_dq)
      .def_readonly("follower_q", &rcs::hw::BilateralFrankaState::follower_q)
      .def_readonly("follower_dq", &rcs::hw::BilateralFrankaState::follower_dq)
      .def_readonly("follower_target_q",
                    &rcs::hw::BilateralFrankaState::follower_target_q)
      .def_readonly("follower_external_tau",
                    &rcs::hw::BilateralFrankaState::follower_external_tau)
      .def_readonly("leader_torque_command",
                    &rcs::hw::BilateralFrankaState::leader_torque_command)
      .def_readonly("running", &rcs::hw::BilateralFrankaState::running)
      .def_readonly("has_reference",
                    &rcs::hw::BilateralFrankaState::has_reference);

  py::class_<rcs::hw::BilateralFranka,
             std::shared_ptr<rcs::hw::BilateralFranka>>(hw, "BilateralFranka")
      .def(py::init<const rcs::hw::BilateralFrankaConfig&,
                    std::optional<std::shared_ptr<rcs::common::Kinematics>>,
                    std::optional<std::shared_ptr<rcs::common::Kinematics>>>(),
           py::arg("cfg"), py::arg("leader_ik") = std::nullopt,
           py::arg("follower_ik") = std::nullopt)
      .def("start", &rcs::hw::BilateralFranka::start)
      .def("stop", &rcs::hw::BilateralFranka::stop)
      .def("move_home", &rcs::hw::BilateralFranka::move_home)
      .def("update_once", &rcs::hw::BilateralFranka::update_once)
      .def("reset", &rcs::hw::BilateralFranka::reset)
      .def("is_running", &rcs::hw::BilateralFranka::is_running)
      .def("get_config", &rcs::hw::BilateralFranka::get_config)
      .def("get_state", &rcs::hw::BilateralFranka::get_state)
      .def("get_leader", &rcs::hw::BilateralFranka::get_leader)
      .def("get_follower", &rcs::hw::BilateralFranka::get_follower);
}
