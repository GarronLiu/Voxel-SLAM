const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
let latestStatus = null;
let modelCatalog = { models: [], vecnorms: [], pairs: [] };
let vizData = { pose: null, path: [], markers: {}, stamp: 0 };
let vizView = { centerX: 0, centerY: 0, scale: 18 };
let goalDrag = null;
let interactionMode = null;
let runtimeInspection = "";

function toast(message, isError = false) {
  const element = $("#toast");
  element.replaceChildren();
  const text = document.createElement("span");
  text.textContent = message;
  element.append(text);
  if (isError) {
    const copyButton = document.createElement("button");
    copyButton.type = "button";
    copyButton.textContent = "复制";
    copyButton.addEventListener("click", () => copyText(message));
    element.append(copyButton);
  }
  element.className = `show${isError ? " error" : ""}`;
  window.setTimeout(() => { element.className = ""; }, isError ? 10000 : 3200);
}

async function copyText(text) {
  const value = String(text).trim();
  if (!value) return;
  try {
    if (navigator.clipboard?.writeText) await navigator.clipboard.writeText(value);
    else throw new Error("Clipboard API unavailable");
  } catch (_error) {
    const fallback = document.createElement("textarea");
    fallback.value = value;
    fallback.style.cssText = "position:fixed;opacity:0;pointer-events:none";
    document.body.appendChild(fallback);
    fallback.select();
    document.execCommand("copy");
    fallback.remove();
  }
  toast("已复制");
}

let selectionCopyTimer = null;
function copySelectedRuntimeText() {
  const selection = window.getSelection();
  const text = selection?.toString().trim();
  const anchor = selection?.anchorNode?.parentElement;
  if (!text || !anchor?.closest(".copy-on-select")) return;
  copyText(text);
}

document.addEventListener("selectionchange", () => {
  window.clearTimeout(selectionCopyTimer);
  selectionCopyTimer = window.setTimeout(copySelectedRuntimeText, 180);
});

async function api(path, payload = null) {
  const options = payload ? { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload) } : {};
  const response = await fetch(path, options);
  const result = await response.json();
  if (!result.ok) throw new Error(result.error || "请求失败");
  return result.data;
}

function setForm(config) {
  const form = $("#configForm");
  Object.entries(config).forEach(([key, value]) => {
    const input = form.elements[key];
    if (!input) return;
    if (input.type === "checkbox") input.checked = Boolean(value);
    else input.value = value ?? "";
  });
  ["31", "32", "36"].forEach((id) => { form.elements[`rate_${id}`].value = config.mavlink_rates?.[id] ?? ""; });
  renderRecentModels(config.recent_models || []);
  renderBagTopics(config.bag_topics || [], config.bag_topics || [], Boolean(config.bag_record_all));
  checkedTopicsCache = new Set(config.bag_topics || []);
  updateSafety(config);
}

function readForm() {
  const form = $("#configForm");
  const value = (name) => form.elements[name].value;
  const checked = (name) => form.elements[name].checked;
  return {
    ...latestStatus.config,
    pointcloud_topic: value("pointcloud_topic"), pointcloud_frame: value("pointcloud_frame"),
    livox_workspace: value("livox_workspace"), livox_launch: value("livox_launch"), livox_xfer_format: Number(value("livox_xfer_format")),
    sensor_yaw_offset_deg: Number(value("sensor_yaw_offset_deg")), merge_physical_scan: checked("merge_physical_scan"),
    scenario_yaml: value("scenario_yaml"), manual_obstacle_radius: Number(value("manual_obstacle_radius")),
    use_scenario_yaml: checked("use_scenario_yaml"), virtual_target_mode: Number(value("virtual_target_mode")),
    virtual_target_wait_dist_start: Number(value("virtual_target_wait_dist_start")),
    virtual_target_wait_dist_stop: Number(value("virtual_target_wait_dist_stop")), virtual_target_speed: Number(value("virtual_target_speed")),
    rc_unlock_channel: Number(value("rc_unlock_channel")), rc_unlock_channel_backup: Number(value("rc_unlock_channel_backup")),
    rc_unlock_threshold: Number(value("rc_unlock_threshold")), fcu_url: value("fcu_url"), gcs_url: value("gcs_url"),
    ublox_serial_port: value("ublox_serial_port"), ublox_rtcm_tcp_host: value("ublox_rtcm_tcp_host"),
    odom_topic: value("odom_topic"), rc_in_topic: value("rc_in_topic"),
    mavlink_rates: { "31": Number(value("rate_31")), "32": Number(value("rate_32")), "36": Number(value("rate_36")) },
    model_search_root: value("model_search_root"), model_path: value("model_path"), vecnorm_path: value("vecnorm_path"),
    max_abs_action: Number(value("max_abs_action")), dry_run: checked("dry_run"), no_actuation: checked("no_actuation"),
    actuation_ack: value("actuation_ack"), bag_directory: value("bag_directory"), bag_session: value("bag_session"),
    bag_tags: value("bag_tags"), bag_record_all: checked("bag_record_all"),
    bag_topics: $$("#bagTopicList input:checked").map((input) => input.value),
    open_process_terminals: checked("open_process_terminals"),
  };
}

function updateSafety(config) {
  const badge = $("#safetyMode");
  const safe = Boolean(config.no_actuation);
  badge.textContent = safe ? "禁止执行器输出" : "执行器输出已启用";
  badge.className = `mode-badge ${safe ? "safe" : "live"}`;
}

function render(status) {
  latestStatus = status;
  const labels = { roscore: "ROS master", mavros: "MAVROS", ublox: "UBlox GPS / RTCM", lidar: "Livox", camera: "MVS Camera", experiment: "控制栈", rviz: "RViz", rosbag: "Rosbag" };
  $("#processStrip").innerHTML = Object.entries(labels).map(([key, label]) => {
    const state = status.processes[key];
    const klass = state.running ? "running" : state.returncode ? "failed" : "";
    const detail = state.running ? `PID ${state.pid}` : state.returncode ? `退出 ${state.returncode}` : "已停止";
    return `<div class="process-chip copy-on-select ${klass}" title="选择文字即可复制"><i class="status-dot"></i><div><strong>${label}</strong><span>${detail}</span></div></div>`;
  }).join("");
  $("#errorList").innerHTML = status.errors.length
    ? status.errors.map(() => '<div class="error-item copy-on-select" title="选择文字即可复制"></div>').join("")
    : '<div class="empty">未检测到运行错误</div>';
  updateTerminalOutputs(status.terminals || {});
  $$(".error-item").forEach((element, index) => { element.textContent = status.errors[index]; });
  const select = $("#logSelect");
  const previous = select.value;
  select.replaceChildren(...Object.keys(status.logs).map((name) => new Option(name, name)));
  if (status.logs[previous]) select.value = previous;
  renderLog();
  updateSafety(status.config);
}

function renderLog() {
  if (window.getSelection()?.anchorNode?.parentElement?.closest("#logOutput")) return;
  if (runtimeInspection) {
    $("#logOutput").textContent = runtimeInspection;
    return;
  }
  const name = $("#logSelect").value;
  $("#logOutput").textContent = latestStatus?.logs?.[name] || "等待进程输出";
}

function renderRecentModels(recent) {
  const select = $("#recentModel");
  select.replaceChildren(new Option("请选择", ""));
  [...recent].reverse().forEach((item, index) => {
    const label = item.model?.split("/").slice(-3).join("/") || `记录 ${index + 1}`;
    select.add(new Option(label, JSON.stringify(item)));
  });
}

function renderModelCatalog(catalog) {
  modelCatalog = catalog;
  const modelSelect = $("#modelCandidate");
  const vecSelect = $("#vecnormCandidate");
  modelSelect.replaceChildren(new Option(`选择 Model（${catalog.models.length}）`, ""));
  vecSelect.replaceChildren(new Option(`选择 VecNormalize（${catalog.vecnorms.length}）`, ""));
  catalog.models.forEach((path) => modelSelect.add(new Option(path, path)));
  catalog.vecnorms.forEach((path) => vecSelect.add(new Option(path, path)));
}

function updateBagTopicSelection() {
  const master = $("#configForm").elements.bag_record_all;
  const topics = $$("#bagTopicList input");
  const selected = topics.filter((input) => input.checked).length;
  const hasFilter = !!($("#topicFilter")?.value || "").trim();
  if (hasFilter) {
    // 有筛选时仅设置 indeterminate，避免意外触发"全选"导致隐藏话题被勾选
    master.checked = false;
    master.indeterminate = selected > 0;
  } else {
    master.indeterminate = selected > 0 && selected < topics.length;
    master.checked = topics.length > 0 && selected === topics.length;
  }
}

let allTopicsCache = [];
let lastFrequencies = null;
let checkedTopicsCache = new Set();

function filterAndRenderTopics() {
  const filter = ($("#topicFilter")?.value || "").trim().toLowerCase();
  const recordAll = $("#configForm").elements.bag_record_all.checked;
  const filtered = filter ? allTopicsCache.filter((t) => t.toLowerCase().includes(filter)) : allTopicsCache;
  renderBagTopics(filtered, [...checkedTopicsCache], recordAll, lastFrequencies);
}

function renderBagTopics(topics, selectedTopics, recordAll, frequencies) {
  const selected = new Set(selectedTopics || []);
  const freqs = frequencies || {};
  const list = $("#bagTopicList");
  list.replaceChildren(...topics.map((topic) => {
    const label = document.createElement("label");
    label.className = "check";
    const input = document.createElement("input");
    input.type = "checkbox";
    input.value = topic;
    input.checked = recordAll || selected.has(topic);
    input.addEventListener("change", () => {
      if (input.checked) checkedTopicsCache.add(topic);
      else checkedTopicsCache.delete(topic);
      updateBagTopicSelection();
    });
    const text = document.createElement("span");
    text.textContent = topic;
    label.append(input, text);
    if (freqs[topic] != null) {
      const badge = document.createElement("span");
      badge.className = "hz-badge";
      badge.textContent = freqs[topic] + " Hz";
      label.append(badge);
    }
    return label;
  }));
  const master = $("#configForm").elements.bag_record_all;
  master.checked = recordAll;
  master.indeterminate = false;
}

async function refreshBagTopics() {
  try {
    const selected = $$("#bagTopicList input:checked").map((input) => input.value);
    const recordAll = $("#configForm").elements.bag_record_all.checked;
    const result = await api("/api/action", { action: "list_rosbag_topics" });
    allTopicsCache = result.topics;
    lastFrequencies = null;
    checkedTopicsCache.clear();
    $("#topicFilter").value = "";
    filterAndRenderTopics();
    toast(`已发现 ${result.topics.length} 个话题`);
  } catch (error) { toast(error.message, true); }
}

async function refreshModels(saveFirst = false) {
  try {
    if (saveFirst) {
      const config = await api("/api/config", readForm());
      latestStatus.config = config;
      setForm(config);
    }
    const catalog = await api("/api/models");
    renderModelCatalog(catalog);
    toast(`已扫描 ${catalog.models.length} 个 Model`);
  } catch (error) { toast(error.message, true); }
}

async function refresh(first = false) {
  try {
    const status = await api("/api/status");
    render(status);
    if (first) {
      setForm(status.config);
      await refreshModels(false);
    }
  } catch (error) { toast(error.message, true); }
}

async function runAction(button, payload) {
  const original = button.textContent;
  button.disabled = true;
  button.textContent = "处理中";
  try {
    const result = await api("/api/action", payload);
    if (["check_lidar_topic", "probe"].includes(payload.action)) {
      runtimeInspection = JSON.stringify(result, null, 2);
      $("#logOutput").textContent = runtimeInspection;
    } else {
      runtimeInspection = "";
    }
    if (payload.action === "stop_rosbag" && result.recording?.path) {
      toast(`录制完成：${result.recording.path}`);
    } else {
      toast(`${original}完成`);
    }
    await refresh();
  } catch (error) { toast(error.message, true); }
  finally { button.disabled = false; button.textContent = original; }
}

$$('[data-action]').forEach((button) => button.addEventListener("click", () => {
  if (button.dataset.action === "start_ublox_dialog") { openUbloxDialog(); return; }
  if (button.dataset.action === "start_lidar_dialog") { openLidarDialog(); return; }
  const payload = { action: button.dataset.action };
  if (button.dataset.action === "start_rosbag") {
    payload.bag_directory = $("#configForm").elements.bag_directory?.value || "";
    payload.bag_session = $("#configForm").elements.bag_session?.value || "";
  }
  runAction(button, payload);
}));
$$('[data-controller]').forEach((button) => button.addEventListener("click", async () => {
  await runAction(button, { action: "select_controller", controller: button.dataset.controller });
  $$("[data-controller]").forEach((item) => item.classList.toggle("selected", item === button));
  $("#controllerState").textContent = button.dataset.controller;
}));
$$('[data-scene]').forEach((button) => button.addEventListener("click", () => runAction(button, { action: "scene_command", command: button.dataset.scene })));

$("#saveConfig").addEventListener("click", async () => {
  try {
    const config = await api("/api/config", readForm());
    latestStatus.config = config;
    setForm(config);
    toast("参数已保存");
  } catch (error) { toast(error.message, true); }
});
$("#refreshModels").addEventListener("click", () => refreshModels(true));
$("#topicFilter").addEventListener("input", () => filterAndRenderTopics());
$("#refreshBagTopics").addEventListener("click", refreshBagTopics);
$("#fetchTopicHz").addEventListener("click", async () => {
  const checked = $$("#bagTopicList input:checked").map((input) => input.value);
  if (!checked.length) { toast("请先勾选需要获取频率的话题", true); return; }
  const button = $("#fetchTopicHz");
  button.disabled = true; button.textContent = "获取中…";
  try {
    const result = await api("/api/action", { action: "get_topic_frequencies", topics: checked });
    lastFrequencies = result.frequencies;
    filterAndRenderTopics();
    const freqCount = Object.keys(result.frequencies || {}).length;
    toast(`${checked.length} 个话题中 ${freqCount} 个获取到频率`);
  } catch (error) { toast(error.message, true); }
  finally { button.disabled = false; button.textContent = "获取频率"; }
});
$("#configForm").elements.bag_record_all.addEventListener("change", (event) => {
  const checked = event.target.checked;
  $$("#bagTopicList input").forEach((input) => {
    input.checked = checked;
    if (checked) checkedTopicsCache.add(input.value);
    else checkedTopicsCache.delete(input.value);
  });
  event.target.indeterminate = false;
});
$("#modelCandidate").addEventListener("change", (event) => {
  if (!event.target.value) return;
  $("#configForm").elements.model_path.value = event.target.value;
  const pair = modelCatalog.pairs.find((item) => item.model === event.target.value);
  if (pair) {
    $("#configForm").elements.vecnorm_path.value = pair.vecnorm;
    $("#vecnormCandidate").value = pair.vecnorm;
  }
});
$("#vecnormCandidate").addEventListener("change", (event) => {
  if (event.target.value) $("#configForm").elements.vecnorm_path.value = event.target.value;
});
$("#recentModel").addEventListener("change", (event) => {
  if (!event.target.value) return;
  const item = JSON.parse(event.target.value);
  $("#configForm").elements.model_path.value = item.model;
  $("#configForm").elements.vecnorm_path.value = item.vecnorm;
});
$("#logSelect").addEventListener("change", () => { runtimeInspection = ""; renderLog(); });
$("#configForm").elements.no_actuation.addEventListener("change", (event) => updateSafety({ no_actuation: event.target.checked }));

$$('.view-tab').forEach((button) => button.addEventListener("click", () => {
  $$(".view-tab").forEach((item) => item.classList.toggle("active", item === button));
  $$(".app-view").forEach((view) => view.classList.toggle("active", view.id === button.dataset.view));
  if (button.dataset.view === "visualizationView") resizeCanvas();
}));

let terminalCounter = 0;
let activeTerminalId = null;
const ansiPattern = /[\u001B\u009B][[\]()#;?]*(?:(?:(?:[a-zA-Z\d]*(?:;[-a-zA-Z\d\/#&.:=?%@~_]+)*)?\u0007)|(?:(?:\d{1,4}(?:[;:]\d{0,4})*)?[\dA-PR-TZcf-nq-uy=><~]))/g;

function updateTerminalOutputs(terminals) {
  Object.entries(terminals).forEach(([id, state]) => {
    const pane = document.querySelector(`.terminal-pane[data-terminal-id="${id}"]`);
    if (!pane) return;
    const output = pane.querySelector(".terminal-output");
    if (window.getSelection()?.anchorNode?.parentElement?.closest(".terminal-output") === output) return;
    const wasAtBottom = output.scrollHeight - output.scrollTop - output.clientHeight < 35;
    output.textContent = (state.output || "").replace(ansiPattern, "");
    pane.querySelector(".terminal-status").textContent = state.running ? "运行中" : "已退出";
    if (wasAtBottom) output.scrollTop = output.scrollHeight;
  });
}

async function createTerminal(splitDirection = null) {
  terminalCounter += 1;
  const id = `term_${Date.now()}_${terminalCounter}`;
  const workspace = $("#terminalWorkspace");
  if (splitDirection) workspace.className = `terminal-workspace split-${splitDirection}`;
  else if (!workspace.classList.contains("split-horizontal") && !workspace.classList.contains("split-vertical")) workspace.classList.add("split-horizontal");
  const pane = document.createElement("section");
  pane.className = "terminal-pane";
  pane.dataset.terminalId = id;
  pane.innerHTML = `<div class="terminal-pane-header copy-on-select" title="选择文字即可复制"><span>${id.replace(/^term_/, "终端 ")} · <b class="terminal-status">启动中</b></span><div><button class="terminal-interrupt">Ctrl+C</button><button class="terminal-close">关闭</button></div></div><pre class="terminal-output copy-on-select" title="选择文字即可复制"></pre><div class="terminal-input"><input autocomplete="off" spellcheck="false" aria-label="终端命令"><button>发送</button></div>`;
  workspace.appendChild(pane);
  pane.addEventListener("pointerdown", () => {
    $$(".terminal-pane").forEach((item) => item.classList.toggle("active", item === pane));
    activeTerminalId = id;
  });
  const input = pane.querySelector(".terminal-input input");
  const send = async () => {
    if (!input.value) return;
    const value = input.value; input.value = "";
    try { await api("/api/action", { action: "terminal_input", id, input: `${value}\n` }); }
    catch (error) { toast(error.message, true); }
  };
  pane.querySelector(".terminal-input button").addEventListener("click", send);
  input.addEventListener("keydown", (event) => { if (event.key === "Enter") { event.preventDefault(); send(); } });
  pane.querySelector(".terminal-interrupt").addEventListener("click", () => api("/api/action", { action: "terminal_input", id, input: "\u0003" }).catch((error) => toast(error.message, true)));
  pane.querySelector(".terminal-close").addEventListener("click", async () => {
    await api("/api/action", { action: "stop_terminal", id }).catch((error) => toast(error.message, true));
    pane.remove();
    activeTerminalId = $(".terminal-pane")?.dataset.terminalId || null;
  });
  try {
    await api("/api/action", { action: "create_terminal", id, cwd: "." });
    activeTerminalId = id; pane.classList.add("active"); input.focus();
  } catch (error) { pane.remove(); toast(error.message, true); }
}

$("#newTerminal").addEventListener("click", () => createTerminal());
$("#splitTerminalHorizontal").addEventListener("click", () => createTerminal("horizontal"));
$("#splitTerminalVertical").addEventListener("click", () => createTerminal("vertical"));

async function pollTerminals() {
  if (!document.querySelector(".terminal-pane")) return;
  try { updateTerminalOutputs(await api("/api/terminals")); }
  catch (error) { /* The main status poll reports connectivity failures. */ }
}

const canvas = $("#mapCanvas");
const ctx = canvas.getContext("2d");
function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.max(1, Math.round(rect.height * ratio));
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  drawVisualization();
}
function worldToScreen(point) {
  return [canvas.clientWidth / 2 + (point[0] - vizView.centerX) * vizView.scale, canvas.clientHeight / 2 - (point[1] - vizView.centerY) * vizView.scale];
}
function screenToWorld(x, y) {
  return [vizView.centerX + (x - canvas.clientWidth / 2) / vizView.scale, vizView.centerY - (y - canvas.clientHeight / 2) / vizView.scale];
}
function drawPolyline(points, color, width = 2) {
  if (!points?.length) return;
  ctx.beginPath();
  points.forEach((point, index) => { const [x, y] = worldToScreen(point); index ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke();
}
function drawVisualization() {
  const width = canvas.clientWidth, height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#f1f3f1"; ctx.fillRect(0, 0, width, height);
  const grid = vizView.scale;
  ctx.strokeStyle = "#d5dad6"; ctx.lineWidth = 1;
  const origin = worldToScreen([0, 0]);
  for (let x = origin[0] % grid; x < width; x += grid) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
  for (let y = origin[1] % grid; y < height; y += grid) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }
  drawPolyline(vizData.path, "#315f8c", 2.5);
  Object.entries(vizData.markers || {}).forEach(([topic, markers]) => markers.forEach((marker) => {
    const rgba = marker.color || [0.6, 0.4, 0.1, 1];
    const color = `rgba(${rgba[0] * 255},${rgba[1] * 255},${rgba[2] * 255},${rgba[3] || 0.85})`;
    if ((marker.points || []).length > 1) drawPolyline(marker.points, color, topic.includes("predicted") ? 2 : 1.5);
    else if (marker.points?.length) {
      const [x, y] = worldToScreen(marker.points[0]);
      const radius = Math.max(3, Math.abs(marker.scale?.[0] || 0.5) * vizView.scale / 2);
      ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
    }
  }));
  if (vizData.pose) {
    const [x, y] = worldToScreen(vizData.pose);
    const yaw = vizData.pose[2];
    const boatScale = Math.max(0.6, Math.min(1.4, vizView.scale / 18));
    ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw); ctx.scale(boatScale, boatScale);
    ctx.fillStyle = "#176b4d"; ctx.strokeStyle = "#0a412c"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(22, 0); ctx.lineTo(7, -9); ctx.lineTo(-16, -9); ctx.lineTo(-21, 0); ctx.lineTo(-16, 9); ctx.lineTo(7, 9); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.fillStyle = "#f6faf7"; ctx.fillRect(-7, -5, 13, 10);
    ctx.strokeStyle = "#e5b33f"; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(4, 0); ctx.lineTo(31, 0); ctx.stroke();
    ctx.restore();
    ctx.fillStyle = "#0d4f36"; ctx.font = "600 11px Inter, sans-serif"; ctx.fillText("USV", x + 14, y - 14);
  }
  if (goalDrag) {
    const start = worldToScreen(goalDrag.start), end = worldToScreen(goalDrag.end);
    ctx.strokeStyle = "#b3261e"; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(...start); ctx.lineTo(...end); ctx.stroke();
  }
}
async function pollVisualization() {
  if (!$("#visualizationView").classList.contains("active")) return;
  try {
    vizData = await api("/api/visualization");
    const age = vizData.stamp ? Math.max(0, Date.now() / 1000 - vizData.stamp) : null;
    $("#vizConnection").textContent = age === null ? "等待 ROS 数据" : `${vizData.frame || "map"} · ${age.toFixed(1)}s`;
    drawVisualization();
  } catch (error) { $("#vizConnection").textContent = error.message; }
}
function fitVisualization() {
  const points = [...(vizData.path || [])];
  if (vizData.pose) points.push(vizData.pose);
  Object.values(vizData.markers || {}).flat().forEach((marker) => points.push(...(marker.points || [])));
  if (!points.length) return;
  const xs = points.map((p) => p[0]), ys = points.map((p) => p[1]);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  vizView.centerX = (minX + maxX) / 2; vizView.centerY = (minY + maxY) / 2;
  vizView.scale = Math.max(2, Math.min(60, Math.min(canvas.clientWidth / Math.max(10, maxX - minX + 4), canvas.clientHeight / Math.max(10, maxY - minY + 4))));
  drawVisualization();
}
$("#fitVisualization").addEventListener("click", fitVisualization);
function setInteractionMode(mode) {
  interactionMode = interactionMode === mode ? null : mode;
  $("#navGoalMode").classList.toggle("primary", interactionMode === "goal");
  $("#poseEstimateMode").classList.toggle("primary", interactionMode === "pose");
  canvas.classList.toggle("goal-mode", Boolean(interactionMode));
}
$("#navGoalMode").addEventListener("click", () => setInteractionMode("goal"));
$("#poseEstimateMode").addEventListener("click", () => setInteractionMode("pose"));
canvas.addEventListener("pointerdown", (event) => {
  if (!interactionMode) return;
  const rect = canvas.getBoundingClientRect();
  const point = screenToWorld(event.clientX - rect.left, event.clientY - rect.top);
  goalDrag = { start: point, end: point }; canvas.setPointerCapture(event.pointerId); drawVisualization();
});
canvas.addEventListener("pointermove", (event) => {
  if (!goalDrag) return;
  const rect = canvas.getBoundingClientRect(); goalDrag.end = screenToWorld(event.clientX - rect.left, event.clientY - rect.top); drawVisualization();
});
canvas.addEventListener("pointerup", async () => {
  if (!goalDrag) return;
  const { start, end } = goalDrag; goalDrag = null; drawVisualization();
  const yaw = Math.atan2(end[1] - start[1], end[0] - start[0]);
  const action = interactionMode === "pose" ? "publish_initial_pose" : "publish_nav_goal";
  const label = interactionMode === "pose" ? "2D Pose Estimate" : "2D Nav Goal";
  try { await api("/api/action", { action, x: start[0], y: start[1], yaw }); toast(`${label} 已发布`); }
  catch (error) { toast(error.message, true); }
});
canvas.addEventListener("wheel", (event) => { event.preventDefault(); vizView.scale = Math.max(2, Math.min(100, vizView.scale * (event.deltaY > 0 ? 0.9 : 1.1))); drawVisualization(); }, { passive: false });
window.addEventListener("resize", resizeCanvas);

// ===== UBlox 启动弹窗 =====
function openUbloxDialog() {
  const config = latestStatus?.config || {};
  $("#ublox-default-port").textContent = config.ublox_serial_port || "-";
  $("#ublox-default-rtcm").textContent = config.ublox_rtcm_tcp_host || "-";
  $("#ublox-serial-port").value = "";
  $("#ublox-rtcm-host").value = "";
  $("#ublox-modal").hidden = false;
  $("#ublox-serial-port").focus();
}

function closeUbloxDialog() {
  $("#ublox-modal").hidden = true;
}

async function confirmUbloxStart() {
  const payload = { action: "start_ublox" };
  const port = $("#ublox-serial-port").value.trim();
  const rtcm = $("#ublox-rtcm-host").value.trim();
  if (port) payload.serial_port = port;
  if (rtcm) payload.rtcm_tcp_host = rtcm;
  closeUbloxDialog();
  const button = document.querySelector('[data-action="start_ublox_dialog"]');
  await runAction(button, payload);
}

$("#ublox-modal-confirm").addEventListener("click", confirmUbloxStart);
$("#ublox-modal-cancel").addEventListener("click", closeUbloxDialog);
$("#ublox-modal-close").addEventListener("click", closeUbloxDialog);
$("#ublox-modal").addEventListener("click", (e) => { if (e.target === $("#ublox-modal")) closeUbloxDialog(); });
document.addEventListener("keydown", (e) => {
  if ($("#ublox-modal").hidden) return;
  if (e.key === "Enter") confirmUbloxStart();
  if (e.key === "Escape") closeUbloxDialog();
});

// ===== Livox 点云格式选择弹窗 =====
function openLidarDialog() {
  const config = latestStatus?.config || {};
  const defaultXfer = config.livox_xfer_format ?? 2;
  $("#lidar-default-xfer").textContent = defaultXfer;
  // 选中配置默认值对应的 radio
  const radios = document.querySelectorAll('input[name="lidar-xfer"]');
  radios.forEach((r) => { r.checked = (Number(r.value) === Number(defaultXfer)); });
  $("#lidar-modal").hidden = false;
}

function closeLidarDialog() {
  $("#lidar-modal").hidden = true;
}

async function confirmLidarStart() {
  const selected = document.querySelector('input[name="lidar-xfer"]:checked');
  const payload = { action: "start_lidar" };
  if (selected) payload.xfer_format = Number(selected.value);
  closeLidarDialog();
  const button = document.querySelector('[data-action="start_lidar_dialog"]');
  await runAction(button, payload);
}

$("#lidar-modal-confirm").addEventListener("click", confirmLidarStart);
$("#lidar-modal-cancel").addEventListener("click", closeLidarDialog);
$("#lidar-modal-close").addEventListener("click", closeLidarDialog);
$("#lidar-modal").addEventListener("click", (e) => { if (e.target === $("#lidar-modal")) closeLidarDialog(); });

refresh(true);
window.setInterval(() => refresh(false), 2000);
window.setInterval(pollVisualization, 500);
window.setInterval(pollTerminals, 500);
