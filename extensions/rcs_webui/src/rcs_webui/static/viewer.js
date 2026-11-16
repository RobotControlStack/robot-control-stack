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
  const { episodes: all } = await api(`/api/sessions/${encodeURIComponent(name)}/episodes`);
  const filter = $("#episode-filter").value;
  const episodes = all.filter((ep) =>
    filter === "intervention" ? ep.intervention_steps > 0
    : filter === "no-intervention" ? !ep.intervention_steps
    : filter === "successful" ? ep.success
    : filter === "unsuccessful" ? !ep.success
    : true
  );
  $("#episode-count").textContent = `${episodes.length} of ${all.length} episodes`;
  $("#episodes tbody").innerHTML = episodes
    .map(
      (ep, i) => `<tr class="clickable" data-uuid="${ep.uuid}">
        <td>${i + 1}</td><td><code>${ep.uuid.slice(0, 8)}</code></td><td>${ep.steps}</td>
        <td>${fmtDuration(ep.duration)}</td><td>${labelPill(ep)}</td>
        <td>${controllerPills(ep)}${ep.intervention_steps ? ` <span class="pill warn" title="steps of human intervention">${ep.intervention_steps} intervention steps</span>` : ""}</td>
        <td>${esc(ep.instruction)}</td>
        <td class="row">
          <button class="btn small ok" data-act="label" data-success="true" title="label successful">&#10003;</button>
          <button class="btn small fail" data-act="label" data-success="false" title="label unsuccessful">&#10007;</button>
          <button class="btn small" data-act="delete" title="delete episode">delete</button>
        </td></tr>`
    )
    .join("") || `<tr><td colspan="8" class="muted">${all.length ? "no episode matches the filter" : "no episodes recorded"}</td></tr>`;
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
$("#episode-filter").addEventListener("change", () => viewerSession && loadEpisodes(viewerSession));
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

// who commanded the robot (annotations.controller); the color follows the controller, never its position
const CONTROLLER_ORDER = ["human", "learned_policy", "scripted_policy", "agent", "none"];
const controllerColor = (c) => `var(--ctrl-${CONTROLLER_ORDER.includes(c) ? c.replace("_", "-") : "unknown"})`;
const controllerName = (c) => c.replace("_", " ");

function controllerPills(ep) {
  const total = ep.steps || 1;
  return Object.entries(ep.controllers || {})
    .sort(([a], [b]) => (CONTROLLER_ORDER.indexOf(a) + 99) % 99 - (CONTROLLER_ORDER.indexOf(b) + 99) % 99)
    .map(([c, n]) => `<span class="ctrl-pill" title="${n} of ${ep.steps} steps"><span class="swatch" style="background:${controllerColor(c)}"></span>${esc(controllerName(c))} ${Math.round((100 * n) / total)}%</span>`)
    .join(" ");
}

function controllerTimeline(time, controller, intervention) {
  const W = 1300, H = 64, L = 46, R = 8, T = 6, B = 22; // about the rendered width of the full-width panel, so text is not stretched
  const n = controller.length;
  const tmin = time[0] ?? 0, tmax = time[n - 1] ?? 1;
  const x = (t) => L + ((t - tmin) / (tmax - tmin || 1)) * (W - L - R);
  // runs of the same controller; a step covers [t_k, t_k+1)
  const runs = [];
  for (let k = 0; k < n; k++) {
    const last = runs[runs.length - 1];
    if (last && last.c === controller[k] && last.iv === intervention[k]) last.end = k;
    else runs.push({ c: controller[k], iv: intervention[k], start: k, end: k });
  }
  const dt = n > 1 ? (tmax - tmin) / (n - 1) : 1;
  const rects = runs
    .map((r) => {
      const x0 = x(time[r.start]), x1 = Math.max(x0 + 1, x(time[r.end] + dt) - 1); // 1px gap between runs
      const w = Math.min(x1, W - R) - x0;
      const tip = `${controllerName(r.c)}${r.iv ? " (intervention)" : ""}: steps ${r.start}-${r.end}, ${(time[r.start] - tmin).toFixed(2)}-${(time[r.end] - tmin + dt).toFixed(2)} s`;
      const label = w > 70 ? `<text x="${x0 + 4}" y="${T + 17}" font-size="10" fill="#fff" pointer-events="none">${esc(controllerName(r.c))}</text>` : "";
      const hatch = r.iv ? `<rect x="${x0}" y="${T}" width="${w}" height="26" fill="url(#iv-hatch)" pointer-events="none"/>` : "";
      return `<rect x="${x0}" y="${T}" width="${w}" height="26" rx="3" fill="${controllerColor(r.c)}"><title>${esc(tip)}</title></rect>${hatch}${label}`;
    })
    .join("");
  const tticks = [tmin, (tmin + tmax) / 2, tmax]
    .map((t, i) => `<text x="${x(t)}" y="${H - 6}" text-anchor="${["start", "middle", "end"][i]}" font-size="10" fill="currentColor">${(t - tmin).toFixed(1)} s</text>`)
    .join("");
  const present = [...new Set(controller)];
  const legend = present
    .map((c) => `<span><span class="swatch" style="background:${controllerColor(c)}"></span> ${esc(controllerName(c))} (${controller.filter((v) => v === c).length} steps)</span>`)
    .join("");
  const ivSteps = intervention.filter(Boolean).length;
  return `<div class="plot plot-wide"><h3>controlled by</h3>
    <svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" class="timeline" data-tmin="${tmin}" data-tmax="${tmax}" data-l="${L}" data-r="${R}" data-w="${W}">
      <defs><pattern id="iv-hatch" width="6" height="6" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="6" stroke="#fff" stroke-opacity="0.45" stroke-width="2"/></pattern></defs>
      ${rects}${tticks}<line class="cursor" y1="${T - 3}" y2="${T + 29}" stroke="currentColor" stroke-width="1.5"/>
    </svg><div class="legend">${legend}${ivSteps ? `<span>hatched: intervention (${ivSteps} steps)</span>` : ""}</div></div>`;
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
  if (ep.controller && ep.controller.length) html.push(controllerTimeline(ep.time, ep.controller, ep.intervention || []));
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
