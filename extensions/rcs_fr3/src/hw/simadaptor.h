#pragma once

#include <Eigen/Dense>

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace adaptor {

// Best-effort real-time scheduling for the calling thread (a 1 kHz robot
// control loop). Two mechanisms, tried in order:
//  1. SCHED_FIFO at the given priority — needs an rtprio rlimit
//     (one line in /etc/security/limits.conf + a fresh login).
//  2. RealtimeKit (the D-Bus service the desktop audio stack uses), which
//     can grant SCHED_RR without any host configuration. rtkit caps the
//     priority (typically 20) and requires a finite RLIMIT_RTTIME on the
//     process; both are fine here — SCHED_RR at any priority preempts all
//     normal threads, and a control thread blocks every millisecond so it
//     can never approach the runtime limit.
// If both fail the thread stays on the normal scheduler and a warning is
// printed; expect communication-constraint violations on a loaded machine
// in that state. priority <= 0 disables.
inline void TryElevateControlThreadPriority(int priority) {
  if (priority <= 0) {
    return;
  }
  sched_param param{};
  param.sched_priority = priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0) {
    std::cerr << "[rcs] control thread on SCHED_FIFO priority " << priority
              << std::endl;
    return;
  }

  rlimit rttime{};
  rttime.rlim_cur = 200000;  // 200 ms of uninterrupted RT CPU (rtkit requires a finite cap)
  rttime.rlim_max = 200000;
  setrlimit(RLIMIT_RTTIME, &rttime);
  const int rtkit_priority = std::min(priority, 20);
  const long pid = static_cast<long>(getpid());
  const long tid = static_cast<long>(syscall(SYS_gettid));
  const std::string cmd =
      "busctl --system --timeout=2 call org.freedesktop.RealtimeKit1 "
      "/org/freedesktop/RealtimeKit1 org.freedesktop.RealtimeKit1 "
      "MakeThreadRealtimeWithPID ttu " +
      std::to_string(pid) + " " + std::to_string(tid) + " " +
      std::to_string(rtkit_priority) + " >/dev/null 2>&1";
  const int sys_rc = std::system(cmd.c_str());
  (void)sys_rc;
  const int policy = sched_getscheduler(0);
  if (policy == SCHED_RR || policy == SCHED_FIFO) {
    std::cerr << "[rcs] control thread on SCHED_RR priority " << rtkit_priority
              << " via RealtimeKit" << std::endl;
    return;
  }

  std::cerr << "[rcs] real-time scheduling unavailable (SCHED_FIFO denied, "
               "RealtimeKit not reachable); control thread stays on the "
               "normal scheduler. For the strong SCHED_FIFO path add "
               "\"<user> - rtprio 99\" to /etc/security/limits.conf and open "
               "a fresh login session." << std::endl;
}


using M = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using V = Eigen::VectorXf;

// Binary format flags (optional int32 in weights header).
// Header layout variants:
//  - v0: int32 dof, emb_dim, hidden, depth
//  - v1: v0 + int32 history_steps
//  - v2: v1 + int32 flags (bitfield, see below)
// Flags:
//  bit0: monotone command map
//  bit1: jointwise conditioning
//  bit2: external torque auxiliary head
//  bit3: jointwise direct-residual no-command-map head
//  bit4: command-conditioned head that consumes the current desired torque
static constexpr uint32_t kFlagUseMonotoneMap = 1u << 0;
static constexpr uint32_t kFlagJointwiseConditioning = 1u << 1;
static constexpr uint32_t kFlagPredictExternalTorque = 1u << 2;
static constexpr uint32_t kFlagJointwiseDirectResidualHead = 1u << 3;
static constexpr uint32_t kFlagCommandConditionedHead = 1u << 4;
static constexpr int kJointDirectProjDim = 16;

inline float gelu_scalar(float x) {
  const float k = std::sqrt(2.0f / static_cast<float>(M_PI));
  return 0.5f * x * (1.0f + std::tanh(k * (x + 0.044715f * x * x * x)));
}

inline void gelu_inplace(M& X) {
  X = X.unaryExpr([](float v) { return gelu_scalar(v); });
}

inline M softplus(const M& Z) {
  const Eigen::ArrayXXf x = Z.array();
  const Eigen::ArrayXXf y = x.max(0.0f) + ((-x.abs()).exp() + 1.0f).log();
  return y.matrix();
}

inline M softplus_k(const M& Z, float k) {
  const Eigen::ArrayXXf x = (k * Z.array());
  const Eigen::ArrayXXf y = x.max(0.0f) + ((-x.abs()).exp() + 1.0f).log();
  return (y / k).matrix();
}

inline M three_regime_map(const M& tau,
                          const M& xl,
                          const M& xh,
                          const M& s_neg,
                          const M& s_mid,
                          const M& s_pos,
                          float k = 30.0f) {
  const M tau_anchor = 0.5f * (xl + xh);
  const M h1 = softplus_k(tau - xl, k) - softplus_k(tau_anchor - xl, k);
  const M h2 = softplus_k(tau - xh, k) - softplus_k(tau_anchor - xh, k);
  return (s_neg.array() * (tau - tau_anchor).array() +
          (s_mid - s_neg).array() * h1.array() +
          (s_pos - s_mid).array() * h2.array())
      .matrix();
}

struct Dense {
  M W;
  V b;
  bool use_bias{true};

  Dense() = default;
  Dense(int Fin, int Fout, bool use_bias_ = true)
      : W(Fin, Fout), b(Fout), use_bias(use_bias_) {}

  M forward(const M& X) const {
    M Y = X * W;
    if (use_bias) {
      Y.rowwise() += b.transpose();
    }
    return Y;
  }
};

struct LayerNorm {
  float eps = 1e-6f;
  V gamma;
  V beta;

  LayerNorm() = default;
  LayerNorm(int dim) : eps(1e-6f), gamma(dim), beta(dim) {}

  M forward(const M& X) const {
    const int B = static_cast<int>(X.rows());
    const int D = static_cast<int>(X.cols());
    M Y(B, D);
    for (int i = 0; i < B; ++i) {
      float mean = X.row(i).mean();
      float var = 0.0f;
      for (int j = 0; j < D; ++j) {
        const float d = X(i, j) - mean;
        var += d * d;
      }
      var /= static_cast<float>(D);
      const float denom = 1.0f / std::sqrt(var + eps);
      for (int j = 0; j < D; ++j) {
        const float normed = (X(i, j) - mean) * denom;
        Y(i, j) = normed * gamma(j) + beta(j);
      }
    }
    return Y;
  }
};

struct NormStats {
  V mean_q;
  V mean_qd;
  V mean_tau;
  V inv_std_q;
  V inv_std_qd;
  V inv_std_tau;
  bool enabled{false};

  void disable() {
    enabled = false;
    mean_q.resize(0);
    mean_qd.resize(0);
    mean_tau.resize(0);
    inv_std_q.resize(0);
    inv_std_qd.resize(0);
    inv_std_tau.resize(0);
  }

  void apply(M& q, M& qd, M& tau) const {
    if (!enabled) return;
    const auto apply_vec = [](M& X, const V& mean, const V& inv_std) {
      X.rowwise() -= mean.transpose();
      X = (X.array().rowwise() * inv_std.transpose().array()).matrix();
    };
    apply_vec(q, mean_q, inv_std_q);
    apply_vec(qd, mean_qd, inv_std_qd);
    apply_vec(tau, mean_tau, inv_std_tau);
  }
};

struct AdaLN {
  int emb_dim;
  int cond_dim;
  Dense proj;
  LayerNorm ln;

  AdaLN() = default;
  AdaLN(int emb_dim_, int cond_dim_)
      : emb_dim(emb_dim_),
        cond_dim(cond_dim_),
        proj(cond_dim_, 2 * emb_dim_),
        ln(emb_dim_) {}

  M forward(const M& x, const M& cond) const {
    M x_norm = ln.forward(x);
    M ss = proj.forward(cond);
    M gamma = ss.leftCols(emb_dim);
    M beta = ss.rightCols(emb_dim);
    return (x_norm.array() * (1.0f + gamma.array()) + beta.array()).matrix();
  }
};

struct SimAdaptorBlock {
  int hidden_dim;
  int out_dim;
  int cond_dim;
  AdaLN adaln1;
  Dense fc1;
  AdaLN adaln2;
  Dense fc2;

  SimAdaptorBlock() = default;
  SimAdaptorBlock(int hidden_dim_, int out_dim_, int cond_dim_)
      : hidden_dim(hidden_dim_),
        out_dim(out_dim_),
        cond_dim(cond_dim_),
        adaln1(hidden_dim_, cond_dim_),
        fc1(hidden_dim_, hidden_dim_),
        adaln2(hidden_dim_, cond_dim_),
        fc2(hidden_dim_, out_dim_) {}

  M forward(const M& x, const M& cond) const {
    M h = adaln1.forward(x, cond);
    h = fc1.forward(h);
    gelu_inplace(h);
    h = adaln2.forward(h, cond);
    h = fc2.forward(h);
    return h;
  }
};

struct ForwardResult {
  M delta_tau;
  bool has_external_torque{false};
  M external_torque;
};

struct SimAdaptor {
  int dof;
  int emb_dim;  // per-joint embedding width when jointwise conditioning is enabled
  int hidden;
  int depth;
  int history_steps;
  bool use_monotone_map{false};
  bool use_jointwise_conditioning{false};
  bool predict_external_torque{false};
  bool use_jointwise_direct_head{false};
  bool use_command_conditioned_head{false};
  uint32_t flags{0};

  Dense q_stem;
  Dense qd_stem;
  Dense tau_stem;
  LayerNorm ln_q;
  LayerNorm ln_qd;
  LayerNorm ln_tau;
  std::vector<SimAdaptorBlock> blocks;
  SimAdaptorBlock out_block;
  SimAdaptorBlock external_torque_block;
  std::vector<Dense> joint_global_projs;
  Dense direct_tau_des_proj;
  Dense joint_direct_tau_hyper;
  Dense joint_direct_projected;
  Dense joint_direct_projected_out;
  NormStats norm_stats;

  SimAdaptor(int dof_,
             int emb_dim_,
             int hidden_,
             int depth_,
             int history_steps_ = 1,
             bool use_monotone_map_ = false,
             bool use_jointwise_conditioning_ = false,
             bool predict_external_torque_ = false,
             bool use_jointwise_direct_head_ = false,
             bool use_command_conditioned_head_ = false)
      : dof(dof_),
        emb_dim(emb_dim_),
        hidden(hidden_),
        depth(depth_),
        history_steps(history_steps_),
        use_monotone_map(use_monotone_map_),
        use_jointwise_conditioning(use_jointwise_conditioning_),
        predict_external_torque(predict_external_torque_),
        use_jointwise_direct_head(use_jointwise_direct_head_),
        use_command_conditioned_head(use_command_conditioned_head_),
        flags((use_monotone_map_ ? kFlagUseMonotoneMap : 0u) |
              (use_jointwise_conditioning_ ? kFlagJointwiseConditioning : 0u) |
              (predict_external_torque_ ? kFlagPredictExternalTorque : 0u) |
              (use_jointwise_direct_head_ ? kFlagJointwiseDirectResidualHead : 0u) |
              (use_command_conditioned_head_ ? kFlagCommandConditionedHead : 0u)),
        q_stem(use_jointwise_conditioning_ ? history_steps_ : (dof_ * history_steps_),
               hidden_,
               false),
        qd_stem(use_jointwise_conditioning_ ? history_steps_ : (dof_ * history_steps_),
                hidden_,
                false),
        tau_stem(use_jointwise_conditioning_ ? history_steps_ : (dof_ * history_steps_),
                 hidden_,
                 false),
        ln_q(hidden_),
        ln_qd(hidden_),
        ln_tau(hidden_),
        out_block(hidden_,
                  use_jointwise_conditioning_
                      ? (use_monotone_map_ ? 6 : 1)
                      : (use_monotone_map_ ? (6 * dof_) : dof_),
                  emb_dim_),
        external_torque_block(hidden_,
                              use_jointwise_conditioning_ ? 1 : dof_,
                              emb_dim_),
        direct_tau_des_proj(hidden_ + dof_, hidden_, true),
        joint_direct_tau_hyper(hidden_, 2 * kJointDirectProjDim, true),
        joint_direct_projected(use_command_conditioned_head_ ? (2 * kJointDirectProjDim)
                                                             : kJointDirectProjDim,
                               kJointDirectProjDim,
                               true),
        joint_direct_projected_out(
            kJointDirectProjDim,
            (use_jointwise_conditioning_ && use_monotone_map_ &&
             use_command_conditioned_head_)
                ? 6
                : 1,
            true) {
    const int middle = std::max(0, depth_ - 1);
    blocks.reserve(middle);
    if (use_jointwise_conditioning_) {
      joint_global_projs.reserve(middle);
    }
    for (int i = 0; i < middle; ++i) {
      blocks.emplace_back(hidden_, hidden_, emb_dim_);
      if (use_jointwise_conditioning_) {
        joint_global_projs.emplace_back(hidden_, hidden_, true);
      }
    }
  }

  int input_cols() const { return dof * history_steps; }

  int expected_history_embedding_cols() const {
    return use_jointwise_conditioning ? (dof * emb_dim) : emb_dim;
  }


  // A single sample of a streaming controller history (torque in the
  // ideal-model, gravity-included space).
  struct StreamRow {
    std::array<double, 7> q{};
    std::array<double, 7> dq{};
    std::array<double, 7> tau_model{};
  };

  // Residual for the newest sample of a 1 kHz stream.
  // ``rows`` must hold exactly ``history_steps`` samples, oldest first;
  // ``latent`` is the history embedding. Returns false (with ``delta`` zero)
  // when the inputs do not match the model or the forward fails. The result
  // is clipped per joint to ``|clip|``; ramping is the caller's policy.
  bool forward_stream(const std::vector<StreamRow>& rows,
                      const Eigen::VectorXd& latent,
                      const Eigen::Matrix<double, 7, 1>& clip,
                      Eigen::Matrix<double, 7, 1>& delta) const {
    delta.setZero();
    const int T = history_steps;
    const int D = dof;
    if (D > 7 || static_cast<int>(rows.size()) != T ||
        latent.size() != expected_history_embedding_cols()) {
      return false;
    }
    M emb(1, latent.size());
    for (Eigen::Index i = 0; i < latent.size(); ++i) {
      emb(0, i) = static_cast<float>(latent(i));
    }
    M q_hist(1, T * D);
    M dq_hist(1, T * D);
    M tau_hist(1, T * D);
    for (int t = 0; t < T; ++t) {
      const StreamRow& r = rows[static_cast<size_t>(t)];
      const int offset = t * D;
      for (int j = 0; j < D; ++j) {
        q_hist(0, offset + j) = static_cast<float>(r.q[j]);
        dq_hist(0, offset + j) = static_cast<float>(r.dq[j]);
        tau_hist(0, offset + j) = static_cast<float>(r.tau_model[j]);
      }
    }
    M out;
    try {
      out = forward(q_hist, dq_hist, tau_hist, emb);
    } catch (...) {
      return false;
    }
    if (out.size() != D) {
      return false;
    }
    for (int j = 0; j < D; ++j) {
      const double v = static_cast<double>(out(0, j));
      const double lim = std::abs(clip(j));
      delta(j) = std::isfinite(v) ? std::clamp(v, -lim, lim) : 0.0;
    }
    return true;
  }

  M forward(const M& q, const M& qd, const M& tau, const M& history_emb) const {
    return forward_with_aux(q, qd, tau, history_emb).delta_tau;
  }

  ForwardResult forward_with_aux(const M& q,
                                 const M& qd,
                                 const M& tau,
                                 const M& history_emb) const {
    ForwardResult out;
    out.delta_tau = M::Zero(q.rows(), dof);
    const int expected_cols = input_cols();
    if (q.cols() != expected_cols || qd.cols() != expected_cols || tau.cols() != expected_cols) {
      std::cerr << "Input dim mismatch: expected " << expected_cols << " cols but got "
                << q.cols() << "/" << qd.cols() << "/" << tau.cols() << ".\n";
      return out;
    }
    if (q.rows() != qd.rows() || q.rows() != tau.rows()) {
      std::cerr << "Batch dim mismatch across q/qd/tau: " << q.rows() << "/" << qd.rows() << "/"
                << tau.rows() << ".\n";
      return out;
    }
    if (history_emb.rows() != q.rows()) {
      std::cerr << "History embedding batch mismatch: expected " << q.rows() << " rows but got "
                << history_emb.rows() << ".\n";
      return out;
    }
    const int expected_emb_cols = expected_history_embedding_cols();
    if (history_emb.cols() != expected_emb_cols) {
      std::cerr << "History embedding dim mismatch: expected " << expected_emb_cols
                << " cols but got " << history_emb.cols() << ".\n";
      return out;
    }

    if (use_jointwise_conditioning) {
      return forward_jointwise(q, qd, tau, history_emb);
    }
    return forward_global(q, qd, tau, history_emb);
  }

  static inline bool read_array(std::istream& in, float* dst, size_t count) {
    in.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(count * sizeof(float)));
    return in.good();
  }

  static bool load_dense(std::istream& in, Dense& d) {
    if (!read_array(in, d.W.data(), static_cast<size_t>(d.W.size()))) return false;
    if (d.use_bias) {
      if (!read_array(in, d.b.data(), static_cast<size_t>(d.b.size()))) return false;
    } else {
      d.b.setZero();
    }
    return true;
  }

  static bool load_layernorm(std::istream& in, LayerNorm& ln) {
    if (!read_array(in, ln.gamma.data(), static_cast<size_t>(ln.gamma.size()))) return false;
    if (!read_array(in, ln.beta.data(), static_cast<size_t>(ln.beta.size()))) return false;
    return true;
  }

  static bool load_block(std::istream& in, SimAdaptorBlock& blk) {
    if (!load_layernorm(in, blk.adaln1.ln)) return false;
    if (!load_dense(in, blk.adaln1.proj)) return false;
    if (!load_dense(in, blk.fc1)) return false;
    if (!load_layernorm(in, blk.adaln2.ln)) return false;
    if (!load_dense(in, blk.adaln2.proj)) return false;
    if (!load_dense(in, blk.fc2)) return false;
    return true;
  }

  static std::unique_ptr<SimAdaptor> LoadFromFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return nullptr;

    in.seekg(0, std::ios::end);
    const std::streampos file_size_pos = in.tellg();
    if (file_size_pos <= 0) return nullptr;
    const size_t file_size = static_cast<size_t>(file_size_pos);
    in.seekg(0, std::ios::beg);
    return LoadFromStream(in, file_size);
  }

  // Same binary layout as LoadFromFile, from an in-memory buffer.
  static std::unique_ptr<SimAdaptor> LoadFromMemory(const char* data,
                                                    size_t size) {
    if (data == nullptr || size == 0) return nullptr;
    std::istringstream in(std::string(data, size), std::ios::binary);
    return LoadFromStream(in, size);
  }

  static std::unique_ptr<SimAdaptor> LoadFromStream(std::istream& in,
                                                    const size_t file_size) {

    int32_t dof_f = 0;
    int32_t emb_f = 0;
    int32_t hidden_f = 0;
    int32_t depth_f = 0;
    in.read(reinterpret_cast<char*>(&dof_f), sizeof(int32_t));
    in.read(reinterpret_cast<char*>(&emb_f), sizeof(int32_t));
    in.read(reinterpret_cast<char*>(&hidden_f), sizeof(int32_t));
    in.read(reinterpret_cast<char*>(&depth_f), sizeof(int32_t));
    if (!in.good()) return nullptr;

    auto try_match_size = [&](int header_ints,
                              int32_t history,
                              uint32_t flags_candidate,
                              size_t& stats_floats_out) -> bool {
      if (history <= 0) return false;
      const size_t header_bytes = static_cast<size_t>(header_ints) * sizeof(int32_t);
      if (file_size < header_bytes) return false;
      const size_t payload_bytes = file_size - header_bytes;
      if (payload_bytes % sizeof(float) != 0) return false;
      const size_t payload_floats = payload_bytes / sizeof(float);
      const bool mono = (flags_candidate & kFlagUseMonotoneMap) != 0u;
      const bool jointwise = (flags_candidate & kFlagJointwiseConditioning) != 0u;
      const bool predict_external =
          (flags_candidate & kFlagPredictExternalTorque) != 0u;
      const bool jointwise_direct =
          (flags_candidate & kFlagJointwiseDirectResidualHead) != 0u;
      const bool command_conditioned =
          (flags_candidate & kFlagCommandConditionedHead) != 0u;
      const uint32_t known_flags =
          kFlagUseMonotoneMap | kFlagJointwiseConditioning |
          kFlagPredictExternalTorque | kFlagJointwiseDirectResidualHead |
          kFlagCommandConditionedHead;
      if ((flags_candidate & ~known_flags) != 0u) return false;
      if (jointwise_direct && (!jointwise || mono)) return false;
      if (command_conditioned && (!jointwise && mono)) return false;
      const size_t weight_floats =
          ModelParamCount(
              dof_f,
              history,
              emb_f,
              hidden_f,
              depth_f,
              mono,
              jointwise,
              jointwise_direct,
              command_conditioned,
              predict_external);
      if (payload_floats < weight_floats) return false;
      const size_t stats_floats = payload_floats - weight_floats;
      const size_t per_dof = 6ull * static_cast<size_t>(dof_f);
      const size_t per_hist = 6ull * static_cast<size_t>(dof_f) * static_cast<size_t>(history);
      if (stats_floats == 0 || stats_floats == per_dof || stats_floats == per_hist) {
        stats_floats_out = stats_floats;
        return true;
      }
      return false;
    };

    int32_t history_f = 1;
    uint32_t flags_f = 0;
    bool use_mono = false;
    bool use_jointwise = false;
    bool predict_external = false;
    bool use_jointwise_direct = false;
    bool use_command_conditioned = false;
    int header_ints = 4;
    size_t stats_floats = 0;

    const std::streampos after_header4 = in.tellg();
    int32_t history_candidate = 1;
    in.read(reinterpret_cast<char*>(&history_candidate), sizeof(int32_t));
    if (in.good()) {
      const std::streampos after_header5 = in.tellg();

      int32_t flags_candidate_i32 = 0;
      in.read(reinterpret_cast<char*>(&flags_candidate_i32), sizeof(int32_t));
      if (in.good()) {
        const uint32_t flags_candidate = static_cast<uint32_t>(flags_candidate_i32);
        size_t stats_candidate = 0;
        if (try_match_size(6, history_candidate, flags_candidate, stats_candidate)) {
          history_f = history_candidate;
          flags_f = flags_candidate;
          use_mono = (flags_f & kFlagUseMonotoneMap) != 0u;
          use_jointwise = (flags_f & kFlagJointwiseConditioning) != 0u;
          predict_external =
              (flags_f & kFlagPredictExternalTorque) != 0u;
          use_jointwise_direct =
              (flags_f & kFlagJointwiseDirectResidualHead) != 0u;
          use_command_conditioned =
              (flags_f & kFlagCommandConditionedHead) != 0u;
          header_ints = 6;
          stats_floats = stats_candidate;
        } else {
          in.clear();
          in.seekg(after_header5);
        }
      } else {
        in.clear();
        in.seekg(after_header5);
      }

      if (header_ints == 4) {
        size_t stats_std = 0;
        size_t stats_mono = 0;
        const bool ok_std = try_match_size(5, history_candidate, 0u, stats_std);
        const bool ok_mono = try_match_size(5, history_candidate, kFlagUseMonotoneMap, stats_mono);
        if (ok_std ^ ok_mono) {
          history_f = history_candidate;
          use_mono = ok_mono;
          use_jointwise = false;
          predict_external = false;
          use_jointwise_direct = false;
          use_command_conditioned = false;
          flags_f = use_mono ? kFlagUseMonotoneMap : 0u;
          header_ints = 5;
          stats_floats = ok_mono ? stats_mono : stats_std;
        } else {
          in.clear();
          in.seekg(after_header4);
        }
      }
    } else {
      in.clear();
      in.seekg(after_header4);
    }

    if (header_ints == 4) {
      const size_t header_bytes = 4ull * sizeof(int32_t);
      if (file_size < header_bytes) return nullptr;
      const size_t payload_bytes = file_size - header_bytes;
      if (payload_bytes % sizeof(float) != 0) return nullptr;
      const size_t payload_floats = payload_bytes / sizeof(float);

      int best_history = -1;
      bool best_mono = false;
      size_t best_stats = 0;

      const size_t per_step =
          3ull * static_cast<size_t>(dof_f) * static_cast<size_t>(hidden_f);
      int max_history =
          per_step > 0 ? static_cast<int>(payload_floats / per_step + 1) : 1;
      max_history = std::min(max_history, 1024);

      for (int h = 1; h <= max_history; ++h) {
        for (const bool mono_candidate : {false, true}) {
          size_t stats_candidate = 0;
          if (try_match_size(4,
                             h,
                             mono_candidate ? kFlagUseMonotoneMap : 0u,
                             stats_candidate)) {
            if (best_history != -1) {
              return nullptr;
            }
            best_history = h;
            best_mono = mono_candidate;
            best_stats = stats_candidate;
          }
        }
      }
      if (best_history <= 0) return nullptr;
      history_f = best_history;
      use_mono = best_mono;
      use_jointwise = false;
      predict_external = false;
      use_jointwise_direct = false;
      use_command_conditioned = false;
      flags_f = use_mono ? kFlagUseMonotoneMap : 0u;
      header_ints = 4;
      stats_floats = best_stats;
    }

    in.clear();
    in.seekg(static_cast<std::streamoff>(header_ints * sizeof(int32_t)), std::ios::beg);

    auto model = std::make_unique<SimAdaptor>(
        dof_f,
        emb_f,
        hidden_f,
        depth_f,
        history_f,
        use_mono,
        use_jointwise,
        predict_external,
        use_jointwise_direct,
        use_command_conditioned);
    model->flags = flags_f;

    if (!load_dense(in, model->q_stem)) return nullptr;
    if (!load_dense(in, model->qd_stem)) return nullptr;
    if (!load_dense(in, model->tau_stem)) return nullptr;
    if (!load_layernorm(in, model->ln_q)) return nullptr;
    if (!load_layernorm(in, model->ln_qd)) return nullptr;
    if (!load_layernorm(in, model->ln_tau)) return nullptr;
    for (size_t i = 0; i < model->blocks.size(); ++i) {
      if (!load_block(in, model->blocks[i])) return nullptr;
      if (model->use_jointwise_conditioning) {
        if (!load_dense(in, model->joint_global_projs[i])) return nullptr;
      }
    }
    if (model->use_jointwise_conditioning && model->use_command_conditioned_head) {
      if (!load_dense(in, model->joint_direct_tau_hyper)) return nullptr;
      if (!load_dense(in, model->joint_direct_projected)) return nullptr;
      if (!load_dense(in, model->joint_direct_projected_out)) return nullptr;
    } else if (model->use_jointwise_direct_head) {
      if (!load_dense(in, model->joint_direct_tau_hyper)) return nullptr;
      if (!load_dense(in, model->joint_direct_projected)) return nullptr;
      if (!load_dense(in, model->joint_direct_projected_out)) return nullptr;
    } else {
      if (!model->use_jointwise_conditioning && model->use_command_conditioned_head) {
        if (!load_dense(in, model->direct_tau_des_proj)) return nullptr;
      }
      if (!load_block(in, model->out_block)) return nullptr;
    }
    if (model->predict_external_torque) {
      if (!load_block(in, model->external_torque_block)) return nullptr;
    }
    model->norm_stats.disable();

    if (stats_floats > 0) {
      const size_t per_dof = 6ull * static_cast<size_t>(model->dof);
      const size_t per_hist =
          6ull * static_cast<size_t>(model->dof) * static_cast<size_t>(model->history_steps);
      if (stats_floats == per_dof || stats_floats == per_hist) {
        if (!load_norm_stats_stream(
                in, stats_floats, model->dof, model->history_steps, model->norm_stats)) {
          std::cerr << "[error] Failed to load norm_stats; continuing without normalization.\n";
          model->norm_stats.disable();
        } else {
          std::cout << "Loaded norm_stats (" << (stats_floats == per_dof ? "per-DoF" : "per-step")
                    << ").\n";
        }
      } else {
        std::cerr << "[error] Unrecognized trailing data (" << stats_floats
                  << " floats). Expected " << per_dof << " (per-DoF) or " << per_hist
                  << " (per-step). Ignoring.\n";
      }
    } else {
      std::cout << "No norm_stats found; continuing without normalization.\n";
    }

    return model;
  }

 private:
  ForwardResult forward_global(const M& q,
                               const M& qd,
                               const M& tau,
                               const M& history_emb) const {
    ForwardResult result;
    M q_in = q;
    M qd_in = qd;
    M tau_in = tau;
    norm_stats.apply(q_in, qd_in, tau_in);

    M tau_des;
    if (use_monotone_map || use_command_conditioned_head) {
      tau_des = tau_in.rightCols(dof);
      tau_in.rightCols(dof).setZero();
    }

    M h_q = ln_q.forward(q_stem.forward(q_in));
    M h_qd = ln_qd.forward(qd_stem.forward(qd_in));
    M h_tau = ln_tau.forward(tau_stem.forward(tau_in));
    M h = h_q + h_qd + h_tau;
    for (const auto& blk : blocks) {
      h = blk.forward(h, history_emb);
      gelu_inplace(h);
    }

    if (!use_monotone_map) {
      if (use_command_conditioned_head) {
        M h_tau(h.rows(), hidden + dof);
        h_tau.leftCols(hidden) = h;
        h_tau.rightCols(dof) = tau_des;
        h_tau = direct_tau_des_proj.forward(h_tau);
        gelu_inplace(h_tau);
        result.delta_tau = out_block.forward(h_tau, history_emb);
      } else {
        result.delta_tau = out_block.forward(h, history_emb);
      }
    } else {
      M params = out_block.forward(h, history_emb);
      if (params.cols() != 6 * dof) {
        std::cerr << "Monotone head dim mismatch: expected " << (6 * dof) << " cols but got "
                  << params.cols() << ".\n";
        result.delta_tau = M::Zero(q.rows(), dof);
      } else {
        const auto slice = [&](int idx) { return params.middleCols(idx * dof, dof); };
        const M s_neg = slice(0);
        const M s_mid_raw = slice(1);
        const M s_pos = slice(2);
        const M xl_raw = slice(3);
        const M dx_raw = slice(4);
        const M bias = slice(5);

        const M s_mid = (softplus(s_mid_raw).array() + 1e-4f).matrix();
        const M xl = xl_raw;
        const M xh = (xl_raw.array() + softplus(dx_raw).array() + 1e-3f).matrix();

        result.delta_tau =
            three_regime_map(tau_des, xl, xh, s_neg, s_mid, s_pos) + bias;
      }
    }

    if (predict_external_torque) {
      M external = external_torque_block.forward(h, history_emb);
      if (external.cols() != dof) {
        std::cerr << "External torque head dim mismatch: expected " << dof
                  << " cols but got " << external.cols() << ".\n";
      } else {
        result.has_external_torque = true;
        result.external_torque = std::move(external);
      }
    }
    return result;
  }

  ForwardResult forward_jointwise(const M& q,
                                  const M& qd,
                                  const M& tau,
                                  const M& history_emb) const {
    ForwardResult result;
    const int batch = static_cast<int>(q.rows());
    M q_in = q;
    M qd_in = qd;
    M tau_in = tau;
    norm_stats.apply(q_in, qd_in, tau_in);

    M tau_des;
    if (use_monotone_map || use_jointwise_direct_head || use_command_conditioned_head) {
      const int last_offset = (history_steps - 1) * dof;
      tau_des = tau_in.middleCols(last_offset, dof);
      tau_in.middleCols(last_offset, dof).setZero();
    }

    const M q_joint = reshape_flat_history_by_joint(q_in, batch);
    const M qd_joint = reshape_flat_history_by_joint(qd_in, batch);
    const M tau_joint = reshape_flat_history_by_joint(tau_in, batch);
    const M cond_joint = reshape_flat_embedding_by_joint(history_emb, batch);

    M h_q = ln_q.forward(q_stem.forward(q_joint));
    M h_qd = ln_qd.forward(qd_stem.forward(qd_joint));
    M h_tau = ln_tau.forward(tau_stem.forward(tau_joint));
    M h = h_q + h_qd + h_tau;

    for (size_t i = 0; i < blocks.size(); ++i) {
      h = blocks[i].forward(h, cond_joint);
      gelu_inplace(h);
      const M h_global = mean_over_joint_rows(h, batch);
      const M projected = joint_global_projs[i].forward(h_global);
      add_joint_residual_inplace(h, projected, batch);
    }

    if (use_command_conditioned_head) {
      M linear_bias = joint_direct_tau_hyper.forward(h);
      if (linear_bias.cols() != 2 * kJointDirectProjDim) {
        std::cerr << "Jointwise command tau hyper dim mismatch: expected "
                  << (2 * kJointDirectProjDim) << " cols but got "
                  << linear_bias.cols() << ".\n";
        result.delta_tau = M::Zero(batch, dof);
      } else {
        const M linear = linear_bias.leftCols(kJointDirectProjDim);
        const M bias = linear_bias.rightCols(kJointDirectProjDim);
        M tau_projected = project_tau_basis(linear, bias, tau_des, batch);
        gelu_inplace(tau_projected);
        const M other_joint_tau_context = other_joint_context(tau_projected, batch);
        const M head_input =
            concat_columns(use_monotone_map ? bias : tau_projected,
                           other_joint_tau_context);
        M features = joint_direct_projected.forward(head_input);
        gelu_inplace(features);
        const M out = joint_direct_projected_out.forward(features);
        if (use_monotone_map) {
          if (out.cols() != 6) {
            std::cerr << "Jointwise command monotone head dim mismatch: expected 6 cols but got "
                      << out.cols() << ".\n";
            result.delta_tau = M::Zero(batch, dof);
          } else {
            const M s_neg = collect_joint_column(out, batch, 0);
            const M s_mid_raw = collect_joint_column(out, batch, 1);
            const M s_pos = collect_joint_column(out, batch, 2);
            const M xl_raw = collect_joint_column(out, batch, 3);
            const M dx_raw = collect_joint_column(out, batch, 4);
            const M map_bias = collect_joint_column(out, batch, 5);

            const M s_mid = (softplus(s_mid_raw).array() + 1e-4f).matrix();
            const M xl = xl_raw;
            const M xh = (xl_raw.array() + softplus(dx_raw).array() + 1e-3f).matrix();

            result.delta_tau =
                three_regime_map(tau_des, xl, xh, s_neg, s_mid, s_pos) + map_bias;
          }
        } else {
          if (out.cols() != 1) {
            std::cerr << "Jointwise command direct output dim mismatch: expected 1 col but got "
                      << out.cols() << ".\n";
            result.delta_tau = M::Zero(batch, dof);
          } else {
            result.delta_tau = collapse_joint_scalar_output(out, batch);
          }
        }
      }
    } else if (use_jointwise_direct_head) {
      M linear_bias = joint_direct_tau_hyper.forward(h);
      if (linear_bias.cols() != 2 * kJointDirectProjDim) {
        std::cerr << "Jointwise direct tau hyper dim mismatch: expected "
                  << (2 * kJointDirectProjDim) << " cols but got "
                  << linear_bias.cols() << ".\n";
        result.delta_tau = M::Zero(batch, dof);
      } else {
        const M linear = linear_bias.leftCols(kJointDirectProjDim);
        const M bias = linear_bias.rightCols(kJointDirectProjDim);
        M projected = project_tau_basis(linear, bias, tau_des, batch);
        gelu_inplace(projected);
        projected = joint_direct_projected.forward(projected);
        gelu_inplace(projected);
        const M out = joint_direct_projected_out.forward(projected);
        if (out.cols() != 1) {
          std::cerr << "Jointwise direct output dim mismatch: expected 1 col but got "
                    << out.cols() << ".\n";
          result.delta_tau = M::Zero(batch, dof);
        } else {
          result.delta_tau = collapse_joint_scalar_output(out, batch);
        }
      }
    } else if (!use_monotone_map) {
      const M out = out_block.forward(h, cond_joint);
      if (out.cols() != 1) {
        std::cerr << "Jointwise head dim mismatch: expected 1 col but got " << out.cols()
                  << ".\n";
        result.delta_tau = M::Zero(batch, dof);
      } else {
        result.delta_tau = collapse_joint_scalar_output(out, batch);
      }
    } else {
      const M out = out_block.forward(h, cond_joint);
      if (out.cols() != 6) {
        std::cerr << "Jointwise monotone head dim mismatch: expected 6 cols but got "
                  << out.cols() << ".\n";
        result.delta_tau = M::Zero(batch, dof);
      } else {
        const M s_neg = collect_joint_column(out, batch, 0);
        const M s_mid_raw = collect_joint_column(out, batch, 1);
        const M s_pos = collect_joint_column(out, batch, 2);
        const M xl_raw = collect_joint_column(out, batch, 3);
        const M dx_raw = collect_joint_column(out, batch, 4);
        const M bias = collect_joint_column(out, batch, 5);

        const M s_mid = (softplus(s_mid_raw).array() + 1e-4f).matrix();
        const M xl = xl_raw;
        const M xh = (xl_raw.array() + softplus(dx_raw).array() + 1e-3f).matrix();

        result.delta_tau =
            three_regime_map(tau_des, xl, xh, s_neg, s_mid, s_pos) + bias;
      }
    }

    if (predict_external_torque) {
      const M external = external_torque_block.forward(h, cond_joint);
      if (external.cols() != 1) {
        std::cerr << "Jointwise external torque head dim mismatch: expected 1 col but got "
                  << external.cols() << ".\n";
      } else {
        result.has_external_torque = true;
        result.external_torque = collapse_joint_scalar_output(external, batch);
      }
    }
    return result;
  }

  M reshape_flat_history_by_joint(const M& flat, int batch) const {
    M out(batch * dof, history_steps);
    for (int b = 0; b < batch; ++b) {
      for (int t = 0; t < history_steps; ++t) {
        const int src_offset = t * dof;
        for (int j = 0; j < dof; ++j) {
          out(b * dof + j, t) = flat(b, src_offset + j);
        }
      }
    }
    return out;
  }

  M reshape_flat_embedding_by_joint(const M& flat, int batch) const {
    M out(batch * dof, emb_dim);
    for (int b = 0; b < batch; ++b) {
      for (int j = 0; j < dof; ++j) {
        const int src_offset = j * emb_dim;
        for (int e = 0; e < emb_dim; ++e) {
          out(b * dof + j, e) = flat(b, src_offset + e);
        }
      }
    }
    return out;
  }

  M mean_over_joint_rows(const M& joint_rows, int batch) const {
    M out(batch, joint_rows.cols());
    out.setZero();
    for (int b = 0; b < batch; ++b) {
      for (int j = 0; j < dof; ++j) {
        out.row(b) += joint_rows.row(b * dof + j);
      }
      out.row(b) /= static_cast<float>(dof);
    }
    return out;
  }

  void add_joint_residual_inplace(M& joint_rows, const M& residual, int batch) const {
    for (int b = 0; b < batch; ++b) {
      for (int j = 0; j < dof; ++j) {
        joint_rows.row(b * dof + j) += residual.row(b);
      }
    }
  }

  M project_tau_basis(const M& scale, const M& bias, const M& tau_des, int batch) const {
    M out(scale.rows(), kJointDirectProjDim);
    for (int b = 0; b < batch; ++b) {
      for (int j = 0; j < dof; ++j) {
        const int row = b * dof + j;
        const float tau_value = tau_des(b, j);
        for (int p = 0; p < kJointDirectProjDim; ++p) {
          out(row, p) = scale(row, p) * tau_value + bias(row, p);
        }
      }
    }
    return out;
  }

  M other_joint_context(const M& joint_rows, int batch) const {
    M out(joint_rows.rows(), joint_rows.cols());
    for (int b = 0; b < batch; ++b) {
      Eigen::RowVectorXf sum = Eigen::RowVectorXf::Zero(joint_rows.cols());
      for (int j = 0; j < dof; ++j) {
        sum += joint_rows.row(b * dof + j);
      }
      for (int j = 0; j < dof; ++j) {
        const int row = b * dof + j;
        out.row(row) = sum - joint_rows.row(row);
      }
    }
    return out;
  }

  static M concat_columns(const M& left, const M& right) {
    M out(left.rows(), left.cols() + right.cols());
    out.leftCols(left.cols()) = left;
    out.rightCols(right.cols()) = right;
    return out;
  }

  M collapse_joint_scalar_output(const M& joint_rows, int batch) const {
    M out(batch, dof);
    for (int b = 0; b < batch; ++b) {
      for (int j = 0; j < dof; ++j) {
        out(b, j) = joint_rows(b * dof + j, 0);
      }
    }
    return out;
  }

  M collect_joint_column(const M& joint_rows, int batch, int col) const {
    M out(batch, dof);
    for (int b = 0; b < batch; ++b) {
      for (int j = 0; j < dof; ++j) {
        out(b, j) = joint_rows(b * dof + j, col);
      }
    }
    return out;
  }

  static size_t DenseParamCount(int in_dim, int out_dim, bool use_bias) {
    return static_cast<size_t>(in_dim) * static_cast<size_t>(out_dim) +
           (use_bias ? static_cast<size_t>(out_dim) : 0ull);
  }

  static size_t BlockParamCount(int hidden_dim, int out_dim, int cond_dim) {
    const size_t h = static_cast<size_t>(hidden_dim);
    const size_t od = static_cast<size_t>(out_dim);
    const size_t cond = static_cast<size_t>(cond_dim);
    const size_t adaln_proj = (2 * h * cond) + (2 * h);
    const size_t ln = 2 * h;
    const size_t fc1 = (h * h) + h;
    const size_t fc2 = (h * od) + od;
    return ln + adaln_proj + fc1 + ln + adaln_proj + fc2;
  }

  static size_t ModelParamCount(int dof,
                                int history_steps,
                                int emb_dim,
                                int hidden,
                                int depth,
                                bool use_monotone_map,
                                bool use_jointwise_conditioning,
                                bool use_jointwise_direct_head,
                                bool use_command_conditioned_head,
                                bool predict_external_torque) {
    const size_t h = static_cast<size_t>(hidden);
    const size_t d = static_cast<size_t>(dof);
    const size_t emb = static_cast<size_t>(emb_dim);
    const size_t hist = static_cast<size_t>(history_steps);
    const size_t stems =
        use_jointwise_conditioning ? (3ull * hist * h) : (3ull * d * hist * h);
    const size_t ln_stems = 6ull * h;
    const size_t middle_blocks =
        static_cast<size_t>(std::max(0, depth - 1)) * BlockParamCount(hidden, hidden, emb);
    const size_t joint_global =
        use_jointwise_conditioning
            ? static_cast<size_t>(std::max(0, depth - 1)) * DenseParamCount(hidden, hidden, true)
            : 0ull;
    const size_t out_dim =
        use_jointwise_conditioning ? (use_monotone_map ? 6ull : 1ull)
                                   : (use_monotone_map ? (6ull * d) : d);
    const bool jointwise_command_head =
        use_jointwise_conditioning && use_command_conditioned_head;
    const bool global_command_head =
        (!use_jointwise_conditioning) && (!use_monotone_map) &&
        use_command_conditioned_head;
    const size_t direct_head_params =
        jointwise_command_head
            ? DenseParamCount(hidden, 2 * kJointDirectProjDim, true) +
                  DenseParamCount(2 * kJointDirectProjDim, kJointDirectProjDim, true) +
                  DenseParamCount(kJointDirectProjDim,
                                  use_monotone_map ? 6 : 1,
                                  true)
            : (use_jointwise_direct_head
                   ? DenseParamCount(hidden, 2 * kJointDirectProjDim, true) +
                         DenseParamCount(kJointDirectProjDim,
                                         kJointDirectProjDim,
                                         true) +
                         DenseParamCount(kJointDirectProjDim, 1, true)
                   : 0ull);
    const size_t out_block_params =
        (use_jointwise_direct_head || jointwise_command_head)
            ? 0ull
            : BlockParamCount(hidden, static_cast<int>(out_dim), emb);
    const size_t global_command_params =
        global_command_head ? DenseParamCount(hidden + dof, hidden, true) : 0ull;
    const size_t external_out_dim = use_jointwise_conditioning ? 1ull : d;
    const size_t external_block_params =
        predict_external_torque
            ? BlockParamCount(hidden, static_cast<int>(external_out_dim), emb)
            : 0ull;
    return stems + ln_stems + middle_blocks + joint_global + global_command_params +
           out_block_params +
           direct_head_params +
           external_block_params;
  }

  static bool load_norm_stats_from_buffer(const std::vector<float>& buf,
                                          int dof,
                                          int history_steps,
                                          NormStats& stats) {
    const int input_dim = dof * history_steps;
    const size_t per_dof = 6ull * static_cast<size_t>(dof);
    const size_t per_step = 6ull * static_cast<size_t>(input_dim);
    if (buf.size() != per_dof && buf.size() != per_step) return false;

    const float eps = 1e-6f;
    V var_q_full;
    V var_qd_full;
    V var_tau_full;

    const auto fill_full = [&](size_t idx, V& dst) {
      dst.resize(input_dim);
      if (buf.size() == per_step) {
        const float* p = buf.data() + static_cast<std::ptrdiff_t>(idx * input_dim);
        for (int i = 0; i < input_dim; ++i) dst(i) = p[i];
        return;
      }
      const float* p = buf.data() + static_cast<std::ptrdiff_t>(idx * dof);
      for (int h = 0; h < history_steps; ++h) {
        for (int j = 0; j < dof; ++j) {
          dst(h * dof + j) = p[j];
        }
      }
    };

    fill_full(0, stats.mean_q);
    fill_full(1, stats.mean_qd);
    fill_full(2, stats.mean_tau);
    fill_full(3, var_q_full);
    fill_full(4, var_qd_full);
    fill_full(5, var_tau_full);

    stats.inv_std_q.resize(input_dim);
    stats.inv_std_qd.resize(input_dim);
    stats.inv_std_tau.resize(input_dim);
    for (int i = 0; i < input_dim; ++i) {
      stats.inv_std_q(i) = 1.0f / std::sqrt(var_q_full(i) + eps);
      stats.inv_std_qd(i) = 1.0f / std::sqrt(var_qd_full(i) + eps);
      stats.inv_std_tau(i) = 1.0f / std::sqrt(var_tau_full(i) + eps);
    }
    stats.enabled = true;
    return true;
  }

  static bool load_norm_stats_stream(std::istream& in,
                                     size_t float_count,
                                     int dof,
                                     int history_steps,
                                     NormStats& stats) {
    std::vector<float> buf(float_count);
    in.read(reinterpret_cast<char*>(buf.data()),
            static_cast<std::streamsize>(float_count * sizeof(float)));
    if (!in.good()) return false;
    return load_norm_stats_from_buffer(buf, dof, history_steps, stats);
  }
};

}  // namespace adaptor
