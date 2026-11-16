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

// ------------------------------------------------------------------ theme
function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  $("#theme-toggle").textContent = theme === "dark" ? "light" : "dark";
  try {
    localStorage.setItem("rcs-theme", theme);
  } catch (e) {}
}
$("#theme-toggle").addEventListener("click", () =>
  applyTheme(document.documentElement.dataset.theme === "dark" ? "light" : "dark")
);
applyTheme(document.documentElement.dataset.theme || "dark");

// ------------------------------------------------------------------ session
let knownCameras = "";
let instructionTouched = false;
let lastStatus = null;

function renderStatus(s) {
  lastStatus = s;
  const state = $("#state");
  state.textContent = s.state === "recording" ? `recording (${s.episode_steps} steps)` : s.state;
  state.className = `badge ${s.state}`;
  if (s.state === "viewer") {
    document.querySelector('[data-tab="session"]').hidden = true;
    if (!location.hash) showTab("data");
    return;
  }
  const sum = s.summary;
  const rows = [
    ["mode", s.mode],
    ["env", s.env ? `${s.env}${sum ? ` (${sum.platform}${sum.task ? `, task ${sum.task}` : ""})` : ""}` : "none, pick a config below and start it"],
    ["operator", s.operator],
    ["policy", s.policy ? `${s.policy.client} ${s.policy.host}:${s.policy.port} (${s.policy.connected ? "connected" : "not connected"}${s.policy.rtc_delay ? `, RTC delay ${s.policy.rtc_delay}` : ""}${s.policy.intervening ? (s.policy.awaiting_resume ? ", HUMAN INTERVENTION, press X to continue" : ", HUMAN INTERVENTION") : s.policy.running ? ", running" : ""})` : null],
    ["policy error", s.policy && s.policy.error ? s.policy.error : null],
    ["env reports", s.env_end ? `${s.env_end}: end the episode with B or Y (auto reset is off)` : null],
    ["data folder", s.session_dir],
    ["step rate", s.env ? `${s.step_hz} Hz` : null],
    ["episodes", `${s.episodes.successful} successful, ${s.episodes.unsuccessful} unsuccessful, ${s.episodes.truncated} ended by the robot`],
    ["last event", s.last_event],
  ].filter(([, v]) => v !== null && v !== undefined);
  $("#status").innerHTML = rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join("");
  $("#error").textContent = [s.error, s.operator_error].filter(Boolean).join("\n");
  if (!instructionTouched) $("#instruction").value = s.instruction || "";
  $("#policy-card").hidden = s.mode !== "eval";
  $("#resume-btn").hidden = s.mode !== "eval";
  if (s.mode === "eval" && !policyFormFilled) {
    policyFormFilled = true;
    fillPolicyForm(s.policy || {});
  }
  const running = !!s.env;
  if ($("#config-check").textContent === "starting..." && s.state !== "starting") {
    $("#config-check").textContent = s.state === "error" ? "start failed, see the error above" : "";
  }
  $("#start-env").textContent = running ? "restart env" : "start env";
  $("#stop-env").disabled = !running && s.state !== "starting";
  document.querySelectorAll("[data-start-key]").forEach((el) => (el.disabled = running));
  $("#settings-note").textContent = running
    ? "Mode, operator, Quest and viewer settings are used at env start: stop the env to change them."
    : "Stored in the data root and used for the next env start.";
  if (running !== envWasRunning) {
    envWasRunning = running;
    loadConfigCatalog().catch(() => {});
  }
  const cams = (s.cameras || []).join(",");
  if (cams !== knownCameras) {
    knownCameras = cams;
    $("#cameras").innerHTML = (s.cameras || [])
      .map((c) => `<figure><img data-cam="${esc(c)}" alt="${esc(c)}"><figcaption>${esc(c)}</figcaption></figure>`)
      .join("") || '<p class="muted">no cameras in the observation</p>';
  }
}
let envWasRunning = null;

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
  b.addEventListener("click", () => api("/api/button", { button: b.dataset.button }).catch((e) => alert(e.message)))
);

$("#instruction").addEventListener("input", () => (instructionTouched = true));
$("#instruction-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  await api("/api/instruction", { instruction: $("#instruction").value }).catch((err) => alert(err.message));
  instructionTouched = false;
});

// ------------------------------------------------------------------ settings
const lines = (text) => text.split("\n").map((l) => l.trim()).filter(Boolean);

async function loadSettings() {
  const res = await api("/api/settings");
  const st = res.settings;
  $("#set-mode").value = st.mode;
  $("#set-operator").value = st.operator;
  $("#set-quest-addr").value = st.quest_addr;
  $("#set-frequency").value = st.frequency;
  $("#set-switch").checked = st.quest_switched_left_right;
  $("#set-gui").checked = !st.headless;
  $("#set-auto-reset").checked = st.auto_reset;
  $("#set-config-dirs").value = st.config_dirs.join("\n");
  $("#set-env-packages").value = st.env_packages.join("\n");
  $("#env-packages-status").textContent = Object.entries(res.env_packages)
    .map(([p, err]) => `${p}: ${err ? err : "loaded"}`)
    .join(" · ");
}

$("#settings-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const body = {
    frequency: +$("#set-frequency").value,
    auto_reset: $("#set-auto-reset").checked,
    config_dirs: lines($("#set-config-dirs").value),
    env_packages: lines($("#set-env-packages").value),
  };
  if (!(lastStatus && lastStatus.env)) {
    Object.assign(body, {
      mode: $("#set-mode").value,
      operator: $("#set-operator").value,
      quest_addr: $("#set-quest-addr").value.trim(),
      quest_switched_left_right: $("#set-switch").checked,
      headless: !$("#set-gui").checked,
    });
  }
  try {
    await api("/api/settings", body);
    $("#settings-check").textContent = "saved";
    setTimeout(() => ($("#settings-check").textContent = ""), 2000);
    await new Promise((r) => setTimeout(r, 300)); // applied in the runner thread
    await loadSettings();
    await loadConfigCatalog();
  } catch (err) {
    $("#settings-check").textContent = err.message;
  }
});

// ------------------------------------------------------------------ policy
let policyFormFilled = false;

async function fillPolicyForm(p) {
  const { clients } = await api("/api/policy/clients");
  $("#policy-client").innerHTML = clients.map((c) => `<option>${esc(c)}</option>`).join("");
  const client = p.client || clients[0];
  if (client && !clients.includes(client)) $("#policy-client").insertAdjacentHTML("beforeend", `<option>${esc(client)}</option>`);
  $("#policy-client").value = client || "";
  $("#policy-host").value = p.host || "localhost";
  $("#policy-port").value = p.port || 20000;
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

// ------------------------------------------------------------------ env config
let pickedId = "";

const editor = new DiffEditor($("#editor"), {
  onChange(hunks) {
    const n = hunks.length;
    const changed = hunks.reduce((acc, h) => acc + Math.max(h.aLines.length, h.bLines.length), 0);
    $("#diff-summary").textContent = n ? `${n} change${n > 1 ? "s" : ""} (${changed} lines) against the picked config` : "no changes";
    $("#diff-summary").classList.toggle("changed", n > 0);
    $("#config-details-note").textContent = n ? `${n} change${n > 1 ? "s" : ""} against the picked config` : "no changes";
    $("#config-details-note").classList.toggle("changed", n > 0);
  },
});

function renderSummary(sum) {
  if (!sum) return ($("#config-summary").innerHTML = "");
  const chips = [
    [sum.platform, sum.platform === "hardware" ? "hw" : "sim"],
    [`creator ${sum.creator}`],
    sum.task ? [`task ${sum.task}`] : null,
    [`robots ${sum.robots.join(", ") || "-"}`],
    [`cameras ${sum.cameras.join(", ") || "none"}`],
    sum.frequency ? [`${sum.frequency} Hz`] : null,
    sum.control_mode ? [`${sum.control_mode}${sum.relative_to ? ` rel. ${sum.relative_to}` : ""}`] : null,
  ].filter(Boolean);
  $("#config-summary").innerHTML = chips.map(([t, cls]) => `<span class="chip ${cls || ""}">${esc(t)}</span>`).join("");
}

async function loadConfigCatalog() {
  const res = await api("/api/configs");
  const groups = {};
  res.configs.forEach((c) => (groups[c.group] = groups[c.group] || []).push(c));
  const keep = pickedId;
  $("#config-select").innerHTML =
    '<option value="">select a config...</option>' +
    Object.entries(groups)
      .map(([g, cs]) => `<optgroup label="${esc(g)}">${cs.map((c) => `<option value="${esc(c.id)}">${esc(c.label)}</option>`).join("")}</optgroup>`)
      .join("");
  const ids = res.configs.map((c) => c.id);
  if (keep && ids.includes(keep)) {
    $("#config-select").value = keep;
  } else if (!editor.dirty() || !editor.value()) {
    // nothing picked yet: the running env, else the config started last
    const initial = ids.includes("running:") ? "running:" : ids.includes(res.last) ? res.last : "";
    if (initial) await pickConfig(initial);
  }
}

async function pickConfig(id) {
  $("#config-check").textContent = "";
  if (!id) {
    pickedId = "";
    editor.set("");
    renderSummary(null);
    return;
  }
  $("#config-check").textContent = "loading...";
  try {
    const entry = await api(`/api/configs/entry?id=${encodeURIComponent(id)}`);
    pickedId = id;
    $("#config-select").value = id;
    editor.set(formatJson({ creator: entry.creator, config: entry.config }));
    renderSummary(entry.summary);
    $("#config-check").textContent = "";
  } catch (err) {
    $("#config-check").textContent = err.message;
    $("#config-select").value = pickedId;
  }
}

$("#config-select").addEventListener("change", (e) => {
  if (editor.dirty() && !confirm("Discard the changes in the editor?")) {
    e.target.value = pickedId;
    return;
  }
  pickConfig(e.target.value);
});
$("#reload-configs").addEventListener("click", () => loadConfigCatalog().catch((err) => ($("#config-check").textContent = err.message)));
$("#diff-prev").addEventListener("click", () => editor.step(-1));
$("#diff-next").addEventListener("click", () => editor.step(1));
$("#diff-revert-all").addEventListener("click", () => editor.revertAll());

function parseEditor() {
  const text = editor.value().trim();
  if (!text) throw new Error("select a config first");
  try {
    return JSON.parse(text);
  } catch (err) {
    throw new Error(`not valid JSON: ${err.message}`);
  }
}

$("#validate-config").addEventListener("click", async () => {
  try {
    const res = await api("/api/configs/validate", parseEditor());
    $("#config-check").textContent = res.ok ? `valid (${res.type})` : res.error;
    if (res.ok) renderSummary(res.summary);
  } catch (err) {
    $("#config-check").textContent = err.message;
  }
});

$("#start-env").addEventListener("click", async () => {
  let cfg;
  try {
    cfg = parseEditor();
  } catch (err) {
    $("#config-check").textContent = err.message;
    return;
  }
  const running = lastStatus && lastStatus.env;
  const n = editor.hunks.length;
  const what = `${running ? "Restart" : "Start"} the env with this config${n ? ` (${n} change${n > 1 ? "s" : ""} against the picked config)` : ""}?`;
  if (!confirm(running ? `${what} A running episode is stored as unsuccessful.` : what)) return;
  try {
    await api("/api/restart", { config: cfg, base_id: pickedId || null });
    $("#config-check").textContent = "starting...";
  } catch (err) {
    $("#config-check").textContent = err.message;
  }
});

$("#stop-env").addEventListener("click", async () => {
  if (!confirm("Stop the env? A running episode is stored as unsuccessful.")) return;
  await api("/api/stop_env", {}).catch((err) => alert(err.message));
});

showTab(location.hash.slice(1) || "session");
loadSettings().catch(() => {});
loadConfigCatalog().catch(() => {});
pollStatus();
refreshCameras();
