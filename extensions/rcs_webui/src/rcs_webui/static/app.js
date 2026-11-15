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
    ["policy", s.policy ? `${s.policy.client} ${s.policy.host}:${s.policy.port} (${s.policy.connected ? "connected" : "not connected"}${s.policy.rtc_delay ? `, RTC delay ${s.policy.rtc_delay}` : ""}${s.policy.intervening ? ", HUMAN INTERVENTION" : s.policy.running ? ", running" : ""})` : null],
    ["policy error", s.policy && s.policy.error ? s.policy.error : null],
    ["data folder", s.session_dir],
    ["step rate", `${s.step_hz} Hz`],
    ["episodes", `${s.episodes.successful} successful, ${s.episodes.unsuccessful} unsuccessful, ${s.episodes.truncated} ended by the robot`],
    ["last event", s.last_event],
  ].filter(([, v]) => v !== null && v !== undefined);
  $("#status").innerHTML = rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join("");
  $("#error").textContent = s.error || "";
  if (!instructionTouched) $("#instruction").value = s.instruction || "";
  $("#policy-card").hidden = s.mode !== "eval";
  if (s.mode === "eval" && s.policy && !policyFormFilled) {
    policyFormFilled = true;
    fillPolicyForm(s.policy);
  }
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

let policyFormFilled = false;

async function fillPolicyForm(p) {
  const { clients } = await api("/api/policy/clients");
  $("#policy-client").innerHTML = clients.map((c) => `<option>${esc(c)}</option>`).join("");
  if (!clients.includes(p.client)) $("#policy-client").insertAdjacentHTML("beforeend", `<option>${esc(p.client)}</option>`);
  $("#policy-client").value = p.client;
  $("#policy-host").value = p.host;
  $("#policy-port").value = p.port;
  $("#policy-exec").value = p.execute_steps ?? "";
  $("#policy-rtc").value = p.rtc_delay ?? 0;
}

$("#policy-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  let options = {};
  const text = $("#policy-options").value.trim();
  if (text) {
    try {
      options = JSON.parse(text);
    } catch (err) {
      return alert(`options are not valid JSON: ${err.message}`);
    }
  }
  const body = {
    client: $("#policy-client").value,
    host: $("#policy-host").value,
    port: +$("#policy-port").value,
    execute_steps: $("#policy-exec").value === "" ? null : +$("#policy-exec").value,
    rtc_delay: +$("#policy-rtc").value || 0,
  };
  if (text) body.options = options;
  await api("/api/policy", body).catch((err) => alert(err.message));
});

async function loadConfigCatalog() {
  const { configs } = await api("/api/configs");
  const groups = {};
  configs.forEach((c) => (groups[c.group] = groups[c.group] || []).push(c));
  $("#config-select").innerHTML =
    '<option value="">same as now</option>' +
    Object.entries(groups)
      .map(([g, cs]) => `<optgroup label="${esc(g)}">${cs.map((c) => `<option value="${esc(c.id)}">${esc(c.label)}</option>`).join("")}</optgroup>`)
      .join("");
}

$("#config-select").addEventListener("change", async (e) => {
  $("#config-check").textContent = "";
  if (!e.target.value) {
    $("#config-json").value = "";
    return;
  }
  try {
    const cfg = await api(`/api/configs/entry?id=${encodeURIComponent(e.target.value)}`);
    $("#config-json").value = JSON.stringify(cfg, null, 2);
  } catch (err) {
    $("#config-check").textContent = err.message;
  }
});

function parseConfigText() {
  const text = $("#config-json").value.trim();
  if (!text) return null;
  return JSON.parse(text);
}

$("#validate-config").addEventListener("click", async () => {
  try {
    const cfg = parseConfigText();
    if (!cfg) return ($("#config-check").textContent = "nothing to validate");
    const res = await api("/api/configs/validate", cfg);
    $("#config-check").textContent = res.ok ? `valid (${res.type})` : res.error;
  } catch (err) {
    $("#config-check").textContent = `not valid JSON: ${err.message}`;
  }
});

$("#restart-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  let cfg;
  try {
    cfg = parseConfigText();
  } catch (err) {
    $("#config-check").textContent = `not valid JSON: ${err.message}`;
    return;
  }
  if (!confirm("Start the env with this config? A running episode is stored as unsuccessful.")) return;
  try {
    await api("/api/restart", cfg ? { config: cfg } : {});
    $("#config-check").textContent = "starting...";
  } catch (err) {
    $("#config-check").textContent = err.message;
  }
});

showTab(location.hash.slice(1) || "session");
loadConfigCatalog().catch(() => {});
pollStatus();
refreshCameras();
