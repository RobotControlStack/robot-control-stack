"use strict";

const $ = (sel) => document.querySelector(sel);

async function api(path, body) {
  const opts = body === undefined ? {} : {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  };
  const res = await fetch(path, opts);
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || res.statusText);
  return data;
}

function esc(s) {
  return String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}

// ------------------------------------------------------------------ tabs
function showTab(name) {
  document.querySelectorAll(".tabpane").forEach((p) => (p.hidden = p.id !== `tab-${name}`));
  document.querySelectorAll(".tab").forEach((a) => a.classList.toggle("active", a.dataset.tab === name));
  window.dispatchEvent(new CustomEvent("tab", { detail: name }));
}
window.addEventListener("hashchange", () => showTab(location.hash.slice(1) || "session"));

// ------------------------------------------------------------------ session
let knownCameras = "";
let instructionTouched = false;

function renderStatus(s) {
  const state = $("#state");
  state.textContent = s.state === "recording" ? `recording (${s.episode_steps} steps)` : s.state;
  state.className = `badge ${s.state}`;
  if (s.state === "viewer") {
    document.querySelector('[data-tab="session"]').hidden = true;
    if (!location.hash) showTab("data");
    return;
  }
  const rows = [
    ["mode", s.mode],
    ["env", s.env],
    ["policy", s.policy ? `${s.policy.host}:${s.policy.port} (${s.policy.connected ? "connected" : "not connected"})` : null],
    ["data folder", s.session_dir],
    ["step rate", `${s.step_hz} Hz`],
    ["episodes", `${s.episodes.successful} successful, ${s.episodes.unsuccessful} unsuccessful, ${s.episodes.truncated} ended by the robot`],
    ["last event", s.last_event],
  ].filter(([, v]) => v !== null && v !== undefined);
  $("#status").innerHTML = rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join("");
  $("#error").textContent = s.error || "";
  if (!instructionTouched) $("#instruction").value = s.instruction || "";
  const cams = (s.cameras || []).join(",");
  if (cams !== knownCameras) {
    knownCameras = cams;
    $("#cameras").innerHTML = (s.cameras || [])
      .map((c) => `<figure><img data-cam="${esc(c)}" alt="${esc(c)}"><figcaption>${esc(c)}</figcaption></figure>`)
      .join("") || '<p class="muted">no cameras in the observation</p>';
  }
}

async function pollStatus() {
  try {
    renderStatus(await api("/api/status"));
  } catch (e) {
    $("#state").textContent = "offline";
    $("#state").className = "badge error";
  }
  setTimeout(pollStatus, 500);
}

function refreshCameras() {
  if (!$("#tab-session").hidden) {
    document.querySelectorAll("#cameras img").forEach((img) => {
      img.src = `/api/live/${encodeURIComponent(img.dataset.cam)}.jpg?t=${Date.now()}`;
    });
  }
  setTimeout(refreshCameras, 200);
}

document.querySelectorAll("[data-button]").forEach((b) =>
  b.addEventListener("click", () => api("/api/button", { button: b.dataset.button }).catch(alert))
);

$("#instruction").addEventListener("input", () => (instructionTouched = true));
$("#instruction-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  await api("/api/instruction", { instruction: $("#instruction").value }).catch(alert);
  instructionTouched = false;
});

async function fillRestartOptions() {
  const src = $("#restart-source").value;
  $("#env-label").hidden = src !== "env";
  $("#rec-label").hidden = src !== "recording";
  if (src === "env" && !$("#env-select").options.length) {
    const { envs } = await api("/api/envs");
    $("#env-select").innerHTML = envs.map((e) => `<option>${esc(e)}</option>`).join("");
  }
  if (src === "recording") {
    const { sessions } = await api("/api/sessions");
    $("#rec-select").innerHTML = sessions
      .map((s) => `<option value="${esc(s.name)}">${esc(s.name)} (${s.episodes ?? "?"} episodes)</option>`)
      .join("");
  }
}
$("#restart-source").addEventListener("change", fillRestartOptions);

$("#restart-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  let overrides = {};
  const text = $("#overrides").value.trim();
  if (text) {
    try {
      overrides = JSON.parse(text);
    } catch (err) {
      alert(`overrides are not valid JSON: ${err}`);
      return;
    }
  }
  const body = { overrides };
  const src = $("#restart-source").value;
  if (src === "env") body.env = $("#env-select").value;
  if (src === "recording") body.from_recording = $("#rec-select").value;
  if (!confirm("Restart the env? A running episode is stored as unsuccessful.")) return;
  await api("/api/restart", body).catch(alert);
});

showTab(location.hash.slice(1) || "session");
pollStatus();
refreshCameras();
