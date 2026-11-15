"use strict";

const COLORS = ["#2f6fde", "#e0723a", "#1f8f4e", "#c23b3b", "#8a5cd6", "#b7791f", "#2aa3b5"];
let viewerSession = null;
let viewerEpisode = null;
let cursorTime = 0;
let playTimer = null;

function fmtDuration(s) {
  return `${s.toFixed(1)} s`;
}

function labelPill(ep) {
  const main = ep.success ? '<span class="pill ok">successful</span>' : '<span class="pill fail">unsuccessful</span>';
  return ep.truncated ? `${main} <span class="pill warn">robot error</span>` : main;
}

async function loadSessions() {
  const { sessions } = await api("/api/sessions");
  const select = $("#session-select");
  const current = select.value;
  select.innerHTML = sessions.map((s) => `<option value="${esc(s.name)}">${esc(s.name)}</option>`).join("");
  if (current && sessions.some((s) => s.name === current)) select.value = current;
  window._sessions = Object.fromEntries(sessions.map((s) => [s.name, s]));
  if (select.value) await loadEpisodes(select.value);
}

async function loadEpisodes(name) {
  viewerSession = name;
  const meta = window._sessions[name] || {};
  $("#session-meta").textContent = `${meta.mode || ""} · ${meta.creator || ""} · ${meta.created || ""}`;
  const { episodes } = await api(`/api/sessions/${encodeURIComponent(name)}/episodes`);
  $("#episodes tbody").innerHTML = episodes
    .map(
      (ep, i) => `<tr class="clickable" data-uuid="${ep.uuid}">
        <td>${i + 1}</td><td><code>${ep.uuid.slice(0, 8)}</code></td><td>${ep.steps}</td>
        <td>${fmtDuration(ep.duration)}</td><td>${labelPill(ep)}</td><td>${esc(ep.instruction)}</td>
        <td class="row">
          <button class="btn small ok" data-act="label" data-success="true" title="label successful">&#10003;</button>
          <button class="btn small fail" data-act="label" data-success="false" title="label unsuccessful">&#10007;</button>
          <button class="btn small" data-act="delete" title="delete episode">delete</button>
        </td></tr>`
    )
    .join("") || '<tr><td colspan="7" class="muted">no episodes recorded</td></tr>';
}

$("#episodes").addEventListener("click", async (e) => {
  const row = e.target.closest("tr[data-uuid]");
  if (!row) return;
  const uuid = row.dataset.uuid;
  const base = `/api/sessions/${encodeURIComponent(viewerSession)}/episodes/${uuid}`;
  const act = e.target.dataset.act;
  try {
    if (act === "label") {
      await api(`${base}/label`, { success: e.target.dataset.success === "true" });
      await loadEpisodes(viewerSession);
      return;
    }
    if (act === "delete") {
      if (!confirm(`Delete episode ${uuid.slice(0, 8)}? This removes its rows from the parquet files.`)) return;
      await api(`${base}/delete`, {});
      if (viewerEpisode && viewerEpisode.uuid === uuid) $("#episode-card").hidden = true;
      await loadEpisodes(viewerSession);
      return;
    }
    document.querySelectorAll("#episodes tr").forEach((r) => r.classList.toggle("selected", r === row));
    await loadEpisode(uuid);
  } catch (err) {
    alert(err.message);
  }
});

$("#session-select").addEventListener("change", (e) => loadEpisodes(e.target.value));
$("#reload-sessions").addEventListener("click", loadSessions);

async function loadEpisode(uuid) {
  stopPlay();
  const ep = await api(`/api/sessions/${encodeURIComponent(viewerSession)}/episodes/${uuid}`);
  viewerEpisode = ep;
  $("#episode-card").hidden = false;
  $("#episode-title").innerHTML = `Episode <code>${uuid.slice(0, 8)}</code> · ${ep.steps} steps · ${labelPill(ep)}`;
  const errors = Object.entries(ep.robots).filter(([, r]) => r.robot_error).map(([n, r]) => `${n}: ${r.robot_error}`);
  $("#episode-error").textContent = errors.join("\n");
  $("#episode-cameras").innerHTML = ep.cameras
    .map((c) => `<figure><img data-cam="${esc(c)}" alt="${esc(c)}"><figcaption>${esc(c)}</figcaption></figure>`)
    .join("");
  $("#scrub").max = ep.steps - 1;
  $("#scrub").value = 0;
  buildPlots(ep);
  showStep(0);
}

function rows2series(rows) {
  // rows: [n][d] -> series: [d][n]
  if (!rows.length || !Array.isArray(rows[0])) return [rows];
  return rows[0].map((_, j) => rows.map((r) => r[j]));
}

function plotSvg(title, time, rows, labels) {
  const series = rows2series(rows);
  const W = 600, H = 200, L = 46, R = 8, T = 8, B = 22;
  const tmin = time[0] ?? 0, tmax = time[time.length - 1] ?? 1;
  let vmin = Infinity, vmax = -Infinity;
  series.forEach((s) => s.forEach((v) => { if (v < vmin) vmin = v; if (v > vmax) vmax = v; }));
  if (!isFinite(vmin)) { vmin = 0; vmax = 1; }
  if (vmax - vmin < 1e-9) { vmin -= 1; vmax += 1; }
  const x = (t) => L + ((t - tmin) / (tmax - tmin || 1)) * (W - L - R);
  const y = (v) => T + (1 - (v - vmin) / (vmax - vmin)) * (H - T - B);
  const paths = series
    .map((s, i) => {
      const d = s.map((v, k) => `${k ? "L" : "M"}${x(time[k]).toFixed(1)},${y(v).toFixed(1)}`).join("");
      return `<path d="${d}" fill="none" stroke="${COLORS[i % COLORS.length]}" stroke-width="1.2"/>`;
    })
    .join("");
  const ticks = [vmin, (vmin + vmax) / 2, vmax]
    .map((v) => `<text x="${L - 4}" y="${y(v) + 4}" text-anchor="end" font-size="10" fill="currentColor">${v.toPrecision(3)}</text>
      <line x1="${L}" x2="${W - R}" y1="${y(v)}" y2="${y(v)}" stroke="currentColor" stroke-opacity="0.12"/>`)
    .join("");
  const tticks = [tmin, (tmin + tmax) / 2, tmax]
    .map((t) => `<text x="${x(t)}" y="${H - 6}" text-anchor="middle" font-size="10" fill="currentColor">${t.toFixed(1)} s</text>`)
    .join("");
  const legend = (labels || series.map((_, i) => String(i)))
    .map((l, i) => `<span><span style="color:${COLORS[i % COLORS.length]}">&#9632;</span> ${esc(l)}</span>`)
    .join("");
  return `<div class="plot"><h3>${esc(title)}</h3>
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" data-tmin="${tmin}" data-tmax="${tmax}" data-l="${L}" data-r="${R}" data-w="${W}">
      ${ticks}${tticks}${paths}<line class="cursor" y1="${T}" y2="${H - B}" stroke="currentColor" stroke-width="1"/>
    </svg><div class="legend">${legend}</div></div>`;
}

const JOINT_LABELS = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"];
const CONTROL_PLOTS = [
  ["q", "joint positions (rad, controller rate)", JOINT_LABELS],
  ["dq", "joint velocities (rad/s)", JOINT_LABELS],
  ["tau_J", "joint torques (Nm)", JOINT_LABELS],
  ["dtau_J", "joint torque derivatives (Nm/s)", JOINT_LABELS],
  ["tau_ext_hat_filtered", "external torque estimate (Nm)", JOINT_LABELS],
  ["O_F_ext_hat_K", "external wrench estimate, base frame (N, Nm)", ["fx", "fy", "fz", "tx", "ty", "tz"]],
];

function buildPlots(ep) {
  const html = [];
  for (const [robot, r] of Object.entries(ep.robots)) {
    if (r.step.joints) html.push(plotSvg(`${robot}: joints (rad, per step)`, ep.time, r.step.joints, JOINT_LABELS));
    if (r.step.gripper) html.push(plotSvg(`${robot}: gripper (1 = open)`, ep.time, r.step.gripper, ["obs"]));
    if (r.step.tquat) html.push(plotSvg(`${robot}: TCP position (m)`, ep.time, r.step.tquat.map((v) => v.slice(0, 3)), ["x", "y", "z"]));
    for (const [key, values] of Object.entries(r.step)) {
      if (key.startsWith("env_action.")) {
        const rows = values.map((v) => (Array.isArray(v) ? v : [v]));
        html.push(plotSvg(`${robot}: action ${key.split(".").pop()}`, ep.time, rows));
      }
    }
    if (r.control.time) {
      for (const [sig, title, labels] of CONTROL_PLOTS) {
        if (r.control[sig]) html.push(plotSvg(`${robot}: ${title}`, r.control.time, r.control[sig], labels));
      }
    }
  }
  $("#plots").innerHTML = html.join("") || '<p class="muted">nothing to plot</p>';
}

function showStep(step) {
  if (!viewerEpisode) return;
  const ep = viewerEpisode;
  cursorTime = ep.time[step] ?? 0;
  $("#scrub-label").textContent = `step ${step} · ${cursorTime.toFixed(2)} s`;
  const base = `/api/sessions/${encodeURIComponent(viewerSession)}/episodes/${ep.uuid}/frames`;
  document.querySelectorAll("#episode-cameras img").forEach((img) => {
    img.src = `${base}/${encodeURIComponent(img.dataset.cam)}/${step}.jpg`;
  });
  document.querySelectorAll("#plots svg").forEach((svg) => {
    const tmin = +svg.dataset.tmin, tmax = +svg.dataset.tmax, L = +svg.dataset.l, R = +svg.dataset.r, W = +svg.dataset.w;
    const xx = L + ((cursorTime - tmin) / (tmax - tmin || 1)) * (W - L - R);
    const line = svg.querySelector(".cursor");
    line.setAttribute("x1", xx);
    line.setAttribute("x2", xx);
  });
}

function stopPlay() {
  if (playTimer) clearTimeout(playTimer);
  playTimer = null;
  $("#play").textContent = "play";
}

function playNext() {
  const ep = viewerEpisode;
  let step = +$("#scrub").value;
  if (step >= ep.steps - 1) return stopPlay();
  step += 1;
  $("#scrub").value = step;
  showStep(step);
  const dt = Math.max(0.01, (ep.time[step + 1] ?? ep.time[step]) - ep.time[step]);
  playTimer = setTimeout(playNext, dt * 1000);
}

$("#play").addEventListener("click", () => {
  if (playTimer) return stopPlay();
  if (+$("#scrub").value >= viewerEpisode.steps - 1) $("#scrub").value = 0;
  $("#play").textContent = "pause";
  playNext();
});
$("#scrub").addEventListener("input", (e) => showStep(+e.target.value));

window.addEventListener("tab", (e) => {
  if (e.detail === "data") loadSessions().catch((err) => alert(err.message));
});
if ((location.hash || "").slice(1) === "data") loadSessions();
