import * as THREE from "./vendor/three.module.js";

const LAYERS = {
  global_map: { color: 0x8ea4b8, points3d: [], enabled: true, object: null, lastFetch3d: 0 },
  mapcloud: { color: 0x49b59f, points3d: [], enabled: false, object: null, lastFetch3d: 0 },
  mapground: { color: 0xd9a441, points3d: [], enabled: true, object: null, lastFetch3d: 0 },
};

const MAPPING_LAYERS = {
  global_map: { color: 0x8ea4b8, points3d: [], enabled: true, object: null, lastFetch3d: 0 },
  ground: { color: 0xd9a441, points3d: [], enabled: true, object: null, lastFetch3d: 0 },
};

const TARGET_MARKER_COLOR = 0x0a84ff;

const state = {
  mode: "view",
  view: "3d",
  selection: null,
  activeGoal: null,
  lastGoal: null,
  odom: null,
  navigationAction: null,
  paths: {},
  mappingOdom: null,
  mappingPath: [],
  mapSceneCleared: false,
  hideCompletedPaths: false,
  showPaths: true,
  showVehicle: true,
  showMappingPath: true,
  showMappingVehicle: true,
  camera: { yaw: -0.72, pitch: 0.82, distance: 58, target: new THREE.Vector3(0, 0, 0) },
  pointer: { active: false, button: 0, start: null, previous: null, selecting: false, touches: new Map(), touchStart: null },
};

const mapCanvas3d = document.getElementById("mapCanvas3d");
const hud = document.getElementById("hud");
const VIEWER_SESSION_KEY = "dddmr_web_control_viewer_id";

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f19);
const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 5000);
const renderer = new THREE.WebGLRenderer({ canvas: mapCanvas3d, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
const raycaster = new THREE.Raycaster();
raycaster.params.Points.threshold = 0.35;
const pointerNdc = new THREE.Vector2();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);

const layerGroup = new THREE.Group();
const pathGroup = new THREE.Group();
const markerGroup = new THREE.Group();
const vehicleGroup = new THREE.Group();
const mappingLayerGroup = new THREE.Group();
const mappingPathGroup = new THREE.Group();
const mappingVehicleGroup = new THREE.Group();
scene.add(layerGroup, pathGroup, markerGroup, vehicleGroup, mappingLayerGroup, mappingPathGroup, mappingVehicleGroup);

scene.add(new THREE.GridHelper(120, 60, 0x8aa0b4, 0x334155));
const axes = new THREE.AxesHelper(4);
axes.position.set(0, 0.04, 0);
scene.add(axes);
scene.add(new THREE.HemisphereLight(0xffffff, 0x223044, 1.15));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(20, 38, 12);
scene.add(sun);

function rosToThree(x, y, z = 0) {
  return new THREE.Vector3(x, z, -y);
}

function threeToRos(point) {
  return { x: point.x, y: -point.z, z: point.y };
}

function fmt(value, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function fmtPercent(value) {
  return Number.isFinite(value) ? `${value.toFixed(1)}%` : "--";
}

function fmtStorage(kb) {
  if (!Number.isFinite(kb)) return "--";
  const gib = kb / 1024 / 1024;
  if (gib >= 1) return `${gib.toFixed(1)} GiB`;
  return `${(kb / 1024).toFixed(0)} MiB`;
}

function usageLevel(value) {
  if (!Number.isFinite(value)) return "waiting";
  if (value >= 85) return "high";
  if (value >= 65) return "medium";
  return "low";
}

async function getJson(url) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return response.json();
}

async function postJson(url, payload) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await response.json();
  if (!response.ok || !data.ok) throw new Error(data.message || response.statusText);
  return data;
}

function viewerSessionId() {
  let viewerId = sessionStorage.getItem(VIEWER_SESSION_KEY);
  if (viewerId) return viewerId;
  if (window.crypto && window.crypto.randomUUID) {
    viewerId = window.crypto.randomUUID();
  } else {
    viewerId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
  }
  sessionStorage.setItem(VIEWER_SESSION_KEY, viewerId);
  return viewerId;
}

function updateViewerCount(count) {
  const element = document.getElementById("viewerCount");
  if (!element) return;
  const viewers = Number.isFinite(count) ? Math.max(0, Math.round(count)) : 0;
  element.textContent = `${viewers} watching`;
}

async function heartbeatViewer() {
  try {
    const data = await postJson("/api/viewer", { viewer_id: viewerSessionId() });
    updateViewerCount(data.viewers);
  } catch (_error) {
    updateViewerCount(0);
  }
}

function resizeAll() {
  const rect = mapCanvas3d.getBoundingClientRect();
  renderer.setSize(Math.max(1, rect.width), Math.max(1, rect.height), false);
  camera.aspect = Math.max(1, rect.width) / Math.max(1, rect.height);
  camera.updateProjectionMatrix();
}

function updateCamera() {
  const { yaw, pitch, distance, target } = state.camera;
  const cp = Math.cos(pitch);
  camera.position.set(
    target.x + Math.cos(yaw) * cp * distance,
    target.y + Math.sin(pitch) * distance,
    target.z + Math.sin(yaw) * cp * distance,
  );
  camera.lookAt(target);
}

function render() {
  updateCamera();
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}

function canvasPoint(event) {
  const rect = mapCanvas3d.getBoundingClientRect();
  return {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top,
    button: event.button,
  };
}

function touchGesture() {
  const touches = Array.from(state.pointer.touches.values());
  if (touches.length < 2) return null;
  const first = touches[0];
  const second = touches[1];
  const center = { x: (first.x + second.x) / 2, y: (first.y + second.y) / 2 };
  const distance = Math.hypot(second.x - first.x, second.y - first.y);
  return { center, distance };
}

function groundIntersection(event) {
  const rect = mapCanvas3d.getBoundingClientRect();
  pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointerNdc, camera);
  const hit = new THREE.Vector3();
  return raycaster.ray.intersectPlane(groundPlane, hit) ? hit : null;
}

function pointCloudIntersection(event) {
  if (!LAYERS.mapground.object || !LAYERS.mapground.enabled) return null;
  const rect = mapCanvas3d.getBoundingClientRect();
  pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointerNdc, camera);
  const hits = raycaster.intersectObject(LAYERS.mapground.object, false);
  return hits.length ? hits[0].point : null;
}

function selectionIntersection(event) {
  if (state.mode === "initial") return groundIntersection(event);
  if (state.mode === "goal") return pointCloudIntersection(event);
  return null;
}

function setSelectionFromPoint(point, yaw = 0) {
  const pose = threeToRos(point);
  state.selection = { x: pose.x, y: pose.y, z: pose.z, yaw };
  updateSelectionDisplay();
  updateTargetMarkers();
}

function panCameraByScreenDelta(dx, dy) {
  const scale = state.camera.distance * 0.0018;
  const right = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 0);
  const up = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 1);
  state.camera.target.addScaledVector(right, -dx * scale);
  state.camera.target.addScaledVector(up, dy * scale);
}

function rotateCameraByScreenDelta(dx, dy) {
  state.camera.yaw += dx * 0.006;
  state.camera.pitch = Math.max(0.18, Math.min(1.38, state.camera.pitch + dy * 0.006));
}

function handleTouchPanZoom(event) {
  state.pointer.touches.set(event.pointerId, canvasPoint(event));
  const gesture = touchGesture();
  if (!gesture) return;
  if (!state.pointer.touchStart) {
    state.pointer.touchStart = { ...gesture, cameraDistance: state.camera.distance };
    return;
  }
  const startDistance = Math.max(1, state.pointer.touchStart.distance);
  const zoomRatio = startDistance / Math.max(1, gesture.distance);
  state.camera.distance = Math.max(5, Math.min(280, state.pointer.touchStart.cameraDistance * zoomRatio));
  const dx = gesture.center.x - state.pointer.touchStart.center.x;
  const dy = gesture.center.y - state.pointer.touchStart.center.y;
  panCameraByScreenDelta(dx, dy);
  state.pointer.touchStart = { ...gesture, cameraDistance: state.camera.distance };
}

function makePointCloud(points, color) {
  const geometry = new THREE.BufferGeometry();
  const positions = new Float32Array(points.length * 3);
  points.forEach((point, index) => {
    const p = rosToThree(point[0], point[1], point[2] || 0);
    positions[index * 3] = p.x;
    positions[index * 3 + 1] = p.y;
    positions[index * 3 + 2] = p.z;
  });
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  return new THREE.Points(
    geometry,
    new THREE.PointsMaterial({ color, size: 0.12, sizeAttenuation: true, transparent: true, opacity: 0.86 }),
  );
}

function updateLayerObject(name) {
  const layer = LAYERS[name];
  if (layer.object) layerGroup.remove(layer.object);
  layer.object = makePointCloud(layer.points3d, layer.color);
  layer.object.visible = layer.enabled;
  layerGroup.add(layer.object);
}

function updateMappingLayerObject(name) {
  const layer = MAPPING_LAYERS[name];
  if (layer.object) mappingLayerGroup.remove(layer.object);
  layer.object = makePointCloud(layer.points3d, layer.color);
  layer.object.visible = layer.enabled;
  mappingLayerGroup.add(layer.object);
}

function clearNavigationSceneState() {
  state.paths = {};
  state.activeGoal = null;
  state.lastGoal = null;
  state.hideCompletedPaths = true;
  updatePaths();
  updateTargetPoseDisplay();
  updateTargetMarkers();
}

function clearMapSceneState() {
  if (state.mapSceneCleared) return;
  Object.keys(LAYERS).forEach((name) => {
    LAYERS[name].points3d = [];
    LAYERS[name].lastFetch3d = 0;
    updateLayerObject(name);
  });
  state.odom = null;
  state.selection = null;
  clearNavigationSceneState();
  updateVehicleMarker();
  updateSelectionDisplay();
  state.mapSceneCleared = true;
}

function mapSourcesAreWaiting(layers) {
  return Object.keys(LAYERS).every((name) => {
    const summary = layers[name];
    return summary && summary.status === "waiting" && !summary.count;
  });
}

function clearGroup(group) {
  while (group.children.length) group.remove(group.children[0]);
}

function makeArrowMarker(pose, color) {
  const group = new THREE.Group();
  group.position.copy(rosToThree(pose.x, pose.y, (pose.z || 0) + 0.1));
  const yaw = pose.yaw || 0;
  const dir = new THREE.Vector3(Math.cos(yaw), 0, -Math.sin(yaw)).normalize();
  group.add(new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0.15, 0), 3.2, color, 0.9, 0.45));
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.9, 0.035, 10, 48),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 }),
  );
  ring.rotation.x = Math.PI / 2;
  group.add(ring);
  return group;
}

function updateTargetMarkers() {
  clearGroup(markerGroup);
  if (state.selection && state.mode !== "view") {
    markerGroup.add(makeArrowMarker(state.selection, TARGET_MARKER_COLOR));
  }
  const goal = state.activeGoal || state.lastGoal;
  if (goal) {
    markerGroup.add(makeArrowMarker(goal, TARGET_MARKER_COLOR));
  }
}

function updateVehicleMarker() {
  clearGroup(vehicleGroup);
  if (!state.showVehicle || !state.odom || !state.odom.pose) return;
  vehicleGroup.add(makeArrowMarker({ ...state.odom.pose, layer: "mapcloud" }, 0x32d74b));
}

function updateMappingVehicleMarker() {
  clearGroup(mappingVehicleGroup);
  if (!state.showMappingVehicle || !state.mappingOdom || !state.mappingOdom.pose) return;
  mappingVehicleGroup.add(makeArrowMarker(state.mappingOdom.pose, 0x32d74b));
}

function updatePaths() {
  clearGroup(pathGroup);
  if (!state.showPaths) return;
  if (state.hideCompletedPaths) return;
  Object.entries(state.paths).forEach(([topic, path]) => {
    if (!path || path.length < 2) return;
    const points = path.map((point) => rosToThree(point[0], point[1], 0.08));
    const pathCurve = new THREE.CatmullRomCurve3(points);
    const geometry = new THREE.TubeGeometry(pathCurve, Math.max(2, points.length * 2), 0.16, 8, false);
    const line = new THREE.Mesh(
      geometry,
      new THREE.MeshBasicMaterial({ color: 0x0a84ff, transparent: true, opacity: 0.9 }),
    );
    line.name = topic;
    pathGroup.add(line);
  });
}

function updateMappingPath() {
  clearGroup(mappingPathGroup);
  if (!state.showMappingPath || !state.mappingPath || state.mappingPath.length < 2) return;
  const points = state.mappingPath.map((point) => rosToThree(point[0], point[1], 0.08));
  const pathCurve = new THREE.CatmullRomCurve3(points);
  const geometry = new THREE.TubeGeometry(pathCurve, Math.max(2, points.length * 2), 0.16, 8, false);
  const line = new THREE.Mesh(
    geometry,
    new THREE.MeshBasicMaterial({ color: 0x32d74b, transparent: true, opacity: 0.9 }),
  );
  line.name = "mapping_path";
  mappingPathGroup.add(line);
}

async function pollState() {
  try {
    const data = await getJson("/api/state");
    updateStatus(data);
    updateViewerCount(data.viewers);
    document.getElementById("serverState").textContent = "active";
    document.getElementById("serverState").className = "pill active";
  } catch (_error) {
    document.getElementById("serverState").textContent = "error";
    document.getElementById("serverState").className = "pill error";
  }
}

async function pollLogs() {
  try {
    const data = await getJson("/api/logs");
    updateTerminalLog("localization", data.logs && data.logs.localization);
    updateTerminalLog("navigation", data.logs && data.logs.navigation);
    updateTerminalLog("mapping", data.logs && data.logs.mapping);
  } catch (_error) {
    updateTerminalLog("localization", null);
    updateTerminalLog("navigation", null);
    updateTerminalLog("mapping", null);
  }
}

async function pollSystem() {
  if (state.view !== "system") return;
  try {
    updateSystem(await getJson("/api/system"));
  } catch (_error) {
    updateSystem(null);
  }
}

async function pollMaps3d(force = false) {
  const now = Date.now();
  await Promise.all(Object.entries(LAYERS).map(async ([name, layer]) => {
    if (!layer.enabled && !force) return;
    if (!force && now - layer.lastFetch3d < 7000) return;
    try {
      const data = await getJson(`/api/map3d?layer=${encodeURIComponent(name)}`);
      layer.points3d = data.points || [];
      if (layer.points3d.length) state.mapSceneCleared = false;
      layer.lastFetch3d = now;
      updateLayerObject(name);
    } catch (_error) {
      layer.points3d = layer.points3d || [];
    }
  }));
}

async function pollMapping3d(force = false) {
  if (state.view !== "mapping") return;
  const now = Date.now();
  await Promise.all(Object.entries(MAPPING_LAYERS).map(async ([name, layer]) => {
    if (!layer.enabled && !force) return;
    if (!force && now - layer.lastFetch3d < 7000) return;
    try {
      const data = await getJson(`/api/mapping/map3d?layer=${encodeURIComponent(name)}`);
      layer.points3d = data.points || [];
      layer.lastFetch3d = now;
      updateMappingLayerObject(name);
    } catch (_error) {
      layer.points3d = layer.points3d || [];
    }
  }));
}

async function pollPaths(pathTopics) {
  if (!state.showPaths) return;
  if (state.hideCompletedPaths) return;
  await Promise.all(pathTopics.map(async (topic) => {
    try {
      const data = await getJson(`/api/path?topic=${encodeURIComponent(topic)}`);
      state.paths[topic] = data.points || [];
    } catch (_error) {
      state.paths[topic] = [];
    }
  }));
  updatePaths();
}

async function pollMappingState() {
  if (state.view !== "mapping") return;
  try {
    const data = await getJson("/api/mapping/state");
    updateMappingStatus(data);
  } catch (_error) {
    updateMappingStatus(null);
  }
}

async function pollMappingPath() {
  if (state.view !== "mapping" || !state.showMappingPath) return;
  try {
    const data = await getJson("/api/mapping/path");
    state.mappingPath = data.points || [];
  } catch (_error) {
    state.mappingPath = [];
  }
  updateMappingPath();
}

async function pollPose() {
  if (state.view === "system") return;
  try {
    updateLivePose(await getJson("/api/pose"));
  } catch (_error) {
    updateLivePose(null);
  }
}

function updateNavigationPose(odom) {
  state.odom = odom && odom.pose ? odom : null;
  const pose = state.odom ? state.odom.pose : null;
  document.getElementById("poseX").textContent = pose ? fmt(pose.x) : "--";
  document.getElementById("poseY").textContent = pose ? fmt(pose.y) : "--";
  document.getElementById("poseZ").textContent = pose ? fmt(pose.z) : "--";
  document.getElementById("poseYaw").textContent = pose ? fmt(pose.yaw, 3) : "--";
  document.getElementById("odomAge").textContent = state.odom && state.odom.age != null ? `${fmt(state.odom.age, 1)} s` : "--";
  updateVehicleMarker();
}

function updateMappingPose(odom) {
  state.mappingOdom = odom && odom.pose ? odom : null;
  const pose = state.mappingOdom ? state.mappingOdom.pose : null;
  document.getElementById("mappingPoseX").textContent = pose ? fmt(pose.x) : "--";
  document.getElementById("mappingPoseY").textContent = pose ? fmt(pose.y) : "--";
  document.getElementById("mappingPoseZ").textContent = pose ? fmt(pose.z) : "--";
  document.getElementById("mappingPoseYaw").textContent = pose ? fmt(pose.yaw, 3) : "--";
  document.getElementById("mappingOdomAge").textContent = state.mappingOdom && state.mappingOdom.age != null ? `${fmt(state.mappingOdom.age, 1)} s` : "--";
  updateMappingVehicleMarker();
}

function updateLivePose(data) {
  updateNavigationPose(data && data.odom);
  updateMappingPose(data && data.mapping_odom);
}

function updateStatus(data) {
  updateNavigationPose(data.odom);
  state.navigationAction = data.navigation_action || null;
  updateNavigationReadiness(data.navigation_ready);
  updateTargetPoseDisplay();
  maybeClearGoalFromActionStatus();

  const sources = document.getElementById("sources");
  sources.innerHTML = "";
  Object.entries(data.layers || {}).forEach(([name, summary]) => sources.appendChild(sourceRow(name, summary)));
  Object.entries(data.paths || {}).forEach(([name, summary]) => sources.appendChild(sourceRow(name, summary)));
  sources.appendChild(sourceRow("odom", data.odom));
  if (mapSourcesAreWaiting(data.layers || {})) {
    clearMapSceneState();
  }
  const topics = Object.keys(data.paths || {});
  if (topics.length) pollPaths(topics);
}

function updateMappingStatus(data) {
  updateMappingPose(data && data.odom);

  const sources = document.getElementById("mappingSources");
  sources.innerHTML = "";
  if (!data) {
    sources.appendChild(sourceRow("mapping", { status: "waiting", age: null, frame_id: "", count: 0 }));
    return;
  }
  Object.entries(data.layers || {}).forEach(([name, summary]) => sources.appendChild(sourceRow(name, summary)));
  sources.appendChild(sourceRow("path", data.path));
  sources.appendChild(sourceRow("odom", data.odom));
}

function sourceRow(name, summary) {
  const row = document.createElement("div");
  row.className = "source";
  const label = document.createElement("div");
  label.className = "name";
  label.textContent = name;
  const pill = document.createElement("div");
  pill.className = `pill ${summary.status}`;
  pill.textContent = summary.status;
  const detail = document.createElement("div");
  detail.className = "detail";
  const age = summary.age == null ? "--" : `${fmt(summary.age, 1)}s`;
  detail.textContent = `frame ${summary.frame_id || "--"} | age ${age} | count ${summary.count || 0}`;
  row.append(label, pill, detail);
  return row;
}

function updateNavigationReadiness(readiness) {
  const element = document.getElementById("navigationReady");
  const ready = readiness && readiness.ready;
  element.textContent = readiness && readiness.message ? readiness.message : "waiting for relocalization";
  element.className = `readiness ${ready ? "ready" : "waiting"}`;
}

function updateTerminalLog(target, summary) {
  const stateElement = document.getElementById(`${target}LogState`);
  const outputElement = document.getElementById(`${target}Logs`);
  if (!summary) {
    stateElement.textContent = "error";
    stateElement.className = "pill error";
    return;
  }
  const lines = summary.lines || [];
  const atBottom = outputElement.scrollTop + outputElement.clientHeight >= outputElement.scrollHeight - 8;
  outputElement.textContent = lines.length ? lines.join("\n") : "No output.";
  if (atBottom) outputElement.scrollTop = outputElement.scrollHeight;
  if (!summary.enabled) {
    stateElement.textContent = "disabled";
    stateElement.className = "pill waiting";
  } else if (summary.running) {
    stateElement.textContent = "running";
    stateElement.className = "pill active";
  } else if (summary.returncode === 0) {
    stateElement.textContent = "exited";
    stateElement.className = "pill loaded";
  } else if (summary.returncode == null) {
    stateElement.textContent = "waiting";
    stateElement.className = "pill waiting";
  } else {
    stateElement.textContent = `error ${summary.returncode}`;
    stateElement.className = "pill error";
  }
}

function updateMeter(id, value) {
  const element = document.getElementById(id);
  if (!element) return;
  const percent = Number.isFinite(value) ? Math.max(0, Math.min(100, value)) : 0;
  element.style.width = `${percent}%`;
}

function updateStatusBadge(id, value) {
  const element = document.getElementById(id);
  if (!element) return;
  const level = usageLevel(value);
  element.className = `system-card-status load-${level}`;
  element.textContent = level === "waiting" ? "waiting" : level;
}

function createCpuCoreRow(core) {
  const row = document.createElement("div");
  row.className = "cpu-core-row";
  row.dataset.coreId = String(core.id);
  const label = document.createElement("span");
  label.className = "cpu-core-label";
  label.textContent = `cpu${core.id}`;
  const track = document.createElement("div");
  track.className = "cpu-core-track";
  const fill = document.createElement("div");
  fill.className = "cpu-core-fill";
  track.appendChild(fill);
  const value = document.createElement("span");
  value.className = "cpu-core-value";
  row.append(label, track, value);
  return row;
}

function updateCpuCores(cores) {
  const list = document.getElementById("systemCpuCores");
  const activeIds = new Set(cores.map((core) => String(core.id)));
  list.querySelectorAll(".cpu-core-row").forEach((row) => {
    if (!activeIds.has(row.dataset.coreId)) row.remove();
  });
  const empty = list.querySelector(".cpu-core-empty");
  if (empty && cores.length) empty.remove();
  if (!cores.length) {
    if (!empty) {
      const nextEmpty = document.createElement("div");
      nextEmpty.className = "cpu-core-empty";
      nextEmpty.textContent = "waiting for CPU sample";
      list.appendChild(nextEmpty);
    }
    return;
  }
  cores.forEach((core) => {
    const percent = Number.isFinite(core.percent) ? Math.max(0, Math.min(100, core.percent)) : 0;
    let row = list.querySelector(`[data-core-id="${core.id}"]`);
    if (!row) {
      row = createCpuCoreRow(core);
      list.appendChild(row);
    }
    row.className = `cpu-core-row load-${usageLevel(core.percent)}`;
    row.style.setProperty("--cpu-percent", `${percent}%`);
    const value = row.querySelector(".cpu-core-value");
    if (value) value.textContent = fmtPercent(core.percent);
  });
}

function updateSystem(data) {
  const cpuTotal = data && data.cpu ? data.cpu.total_percent : null;
  document.getElementById("systemCpuTotal").textContent = fmtPercent(cpuTotal);
  updateMeter("systemCpuMeter", cpuTotal);
  updateStatusBadge("systemCpuStatus", cpuTotal);
  const memory = data && data.memory ? data.memory : null;
  const swap = data && data.swap ? data.swap : null;
  document.getElementById("systemMemory").textContent = memory ? `${fmtPercent(memory.percent)} | ${fmtStorage(memory.used_kb)} / ${fmtStorage(memory.total_kb)}` : "--";
  document.getElementById("systemSwap").textContent = swap ? `${fmtPercent(swap.percent)} | ${fmtStorage(swap.used_kb)} / ${fmtStorage(swap.total_kb)}` : "--";
  updateMeter("systemMemoryMeter", memory ? memory.percent : null);
  updateMeter("systemSwapMeter", swap ? swap.percent : null);
  updateStatusBadge("systemMemoryStatus", memory ? memory.percent : null);
  updateStatusBadge("systemSwapStatus", swap ? swap.percent : null);
  updateCpuCores(data && data.cpu && data.cpu.cores ? data.cpu.cores : []);
}

function updateSelectionDisplay() {
  const s = state.selection;
  document.getElementById("selX").textContent = s ? fmt(s.x) : "--";
  document.getElementById("selY").textContent = s ? fmt(s.y) : "--";
  document.getElementById("selZ").textContent = s ? fmt(s.z || 0) : "--";
  document.getElementById("selYaw").textContent = s ? fmt(s.yaw, 3) : "--";
}

function updateTargetPoseDisplay() {
  const goal = state.activeGoal || state.lastGoal;
  document.getElementById("goalX").textContent = goal ? fmt(goal.x) : "--";
  document.getElementById("goalY").textContent = goal ? fmt(goal.y) : "--";
  document.getElementById("goalZ").textContent = goal ? fmt(goal.z || 0) : "--";
  document.getElementById("goalYaw").textContent = goal ? fmt(goal.yaw, 3) : "--";
}

function maybeClearGoalFromActionStatus() {
  if (!state.activeGoal || !state.navigationAction || !state.navigationAction.data) return;
  const status = state.navigationAction.data.status;
  const stamp = state.navigationAction.stamp || 0;
  if (status === "succeeded" && stamp >= state.activeGoal.publishedAt) {
    state.activeGoal = null;
    state.lastGoal = null;
    state.paths = {};
    state.hideCompletedPaths = true;
    updateTargetPoseDisplay();
    updateTargetMarkers();
    updatePaths();
  }
}

function setMode(mode) {
  const previousMode = state.mode;
  state.mode = mode;
  document.getElementById("modeSegment").dataset.active = mode;
  let shouldClearSelection = false;
  if (previousMode !== mode) {
    shouldClearSelection = true;
  }
  if (mode === "view") {
    shouldClearSelection = true;
  }
  if (shouldClearSelection) {
    state.selection = null;
    updateSelectionDisplay();
  }
  document.querySelectorAll(".mode").forEach((button) => {
    button.classList.toggle("active", button.dataset.mode === mode);
  });
  updatePublishPoseButton();
  updateTargetMarkers();
}

function setTab(tab) {
  state.view = tab;
  document.getElementById("viewTabs").dataset.active = tab;
  document.getElementById("tab3dMap").classList.toggle("active", tab === "3d");
  document.getElementById("tabMapping").classList.toggle("active", tab === "mapping");
  document.getElementById("tabSystem").classList.toggle("active", tab === "system");
  document.getElementById("view3d").classList.toggle("active", tab === "3d" || tab === "mapping");
  document.getElementById("viewSystem").classList.toggle("active", tab === "system");
  document.querySelector(".app").classList.toggle("system-mode", tab === "system");
  const panelName = tab === "system" ? "" : (tab === "mapping" ? "mapping" : "navigation");
  document.querySelectorAll("[data-panel]").forEach((panel) => {
    panel.classList.toggle("active", panel.dataset.panel === panelName);
  });
  layerGroup.visible = tab === "3d";
  pathGroup.visible = tab === "3d";
  markerGroup.visible = tab === "3d";
  vehicleGroup.visible = tab === "3d";
  mappingLayerGroup.visible = tab === "mapping";
  mappingPathGroup.visible = tab === "mapping";
  mappingVehicleGroup.visible = tab === "mapping";
  if (tab === "3d") {
    resizeAll();
    pollMaps3d(true);
  } else if (tab === "mapping") {
    resizeAll();
    pollMappingState();
    pollMapping3d(true);
    pollMappingPath();
  } else if (tab === "system") {
    pollSystem();
  }
}

function selectedPosePayload(includeZ) {
  return { x: state.selection.x, y: state.selection.y, z: includeZ ? state.selection.z : 0, yaw: state.selection.yaw };
}

function updatePublishPoseButton() {
  const button = document.getElementById("publishPose");
  button.disabled = state.mode === "view";
  button.textContent = "Publish Pose";
}

async function publishPose() {
  const result = document.getElementById("publishResult");
  if (state.mode === "view") {
    result.textContent = "Switch to Initial Pose or Goal mode first.";
    return;
  }
  if (!state.selection) {
    result.textContent = "Select a pose in the 3D map first.";
    return;
  }
  try {
    if (state.mode === "initial") {
      const response = await postJson("/api/initialpose", selectedPosePayload(false));
      result.textContent = `${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`;
      return;
    }
    if (state.mode === "goal") {
      const payload = selectedPosePayload(true);
      const response = await postJson("/api/goal", payload);
      state.activeGoal = { ...payload, publishedAt: response.stamp };
      state.lastGoal = { ...state.activeGoal };
      state.hideCompletedPaths = false;
      state.paths = {};
      state.selection = null;
      updateSelectionDisplay();
      updatePaths();
      updateTargetPoseDisplay();
      updateTargetMarkers();
      result.textContent = `${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`;
    }
  } catch (error) {
    result.textContent = `Publish failed: ${error.message}`;
  }
}

async function stopManagedTarget(target) {
  const result = document.getElementById("publishResult");
  try {
    const response = await postJson("/api/managed/stop", { target });
    result.textContent = `${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`;
    if (target === "all" || target === "localization") {
      clearMapSceneState();
      pollMaps3d(true);
    } else if (target === "navigation") {
      clearNavigationSceneState();
    }
    pollState();
    pollLogs();
  } catch (error) {
    result.textContent = `Stop failed: ${error.message}`;
  }
}

async function startManagedTarget(target) {
  const result = document.getElementById("publishResult");
  try {
    const response = await postJson("/api/managed/start", { target });
    result.textContent = `${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`;
    pollLogs();
  } catch (error) {
    result.textContent = `Start failed: ${error.message}`;
  }
}

function attachEvents() {
  window.addEventListener("resize", resizeAll);
  document.getElementById("tab3dMap").addEventListener("click", () => setTab("3d"));
  document.getElementById("tabMapping").addEventListener("click", () => setTab("mapping"));
  document.getElementById("tabSystem").addEventListener("click", () => setTab("system"));
  document.querySelectorAll(".mode").forEach((button) => button.addEventListener("click", () => setMode(button.dataset.mode)));
  document.querySelectorAll("input[data-layer]").forEach((input) => {
    input.addEventListener("change", () => {
      LAYERS[input.dataset.layer].enabled = input.checked;
      if (LAYERS[input.dataset.layer].object) LAYERS[input.dataset.layer].object.visible = input.checked;
      pollMaps3d(true);
    });
  });
  document.getElementById("pathLayer").addEventListener("change", (event) => {
    state.showPaths = event.target.checked;
    updatePaths();
  });
  document.getElementById("vehicleLayer").addEventListener("change", (event) => {
    state.showVehicle = event.target.checked;
    updateVehicleMarker();
  });
  document.querySelectorAll("input[data-mapping-layer]").forEach((input) => {
    input.addEventListener("change", () => {
      MAPPING_LAYERS[input.dataset.mappingLayer].enabled = input.checked;
      if (MAPPING_LAYERS[input.dataset.mappingLayer].object) {
        MAPPING_LAYERS[input.dataset.mappingLayer].object.visible = input.checked;
      }
      pollMapping3d(true);
    });
  });
  document.getElementById("mappingPathLayer").addEventListener("change", (event) => {
    state.showMappingPath = event.target.checked;
    updateMappingPath();
  });
  document.getElementById("mappingVehicleLayer").addEventListener("change", (event) => {
    state.showMappingVehicle = event.target.checked;
    updateMappingVehicleMarker();
  });
  updatePublishPoseButton();
  document.getElementById("publishPose").addEventListener("click", publishPose);
  document.querySelectorAll("[data-start-target]").forEach((button) => button.addEventListener("click", () => startManagedTarget(button.dataset.startTarget)));
  document.querySelectorAll("[data-stop-target]").forEach((button) => button.addEventListener("click", () => stopManagedTarget(button.dataset.stopTarget)));

  mapCanvas3d.addEventListener("contextmenu", (event) => event.preventDefault());
  mapCanvas3d.addEventListener("wheel", (event) => {
    event.preventDefault();
    state.camera.distance = Math.max(5, Math.min(280, state.camera.distance * (event.deltaY < 0 ? 0.88 : 1.13)));
  }, { passive: false });
  mapCanvas3d.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    mapCanvas3d.setPointerCapture(event.pointerId);
    const point = canvasPoint(event);
    if (event.pointerType === "touch") {
      state.pointer.touches.set(event.pointerId, point);
      state.pointer.touchStart = touchGesture();
      if (state.pointer.touchStart) {
        state.pointer.touchStart.cameraDistance = state.camera.distance;
        state.pointer.selecting = false;
        return;
      }
    }
    state.pointer = {
      active: true,
      button: event.button,
      start: point,
      previous: point,
      selecting: false,
      touches: state.pointer.touches,
      touchStart: state.pointer.touchStart,
    };
    if (event.pointerType === "touch") {
      state.pointer.touches.set(event.pointerId, point);
    }
    if (state.mode !== "view" && event.button === 0) {
      const hit = selectionIntersection(event);
      if (hit) {
        setSelectionFromPoint(hit, 0);
        state.pointer.selecting = true;
      }
    }
  });
  mapCanvas3d.addEventListener("pointermove", (event) => {
    if (event.pointerType === "touch" && state.pointer.touches.size >= 2) {
      event.preventDefault();
      handleTouchPanZoom(event);
      return;
    }
    if (!state.pointer.active || !state.pointer.previous) return;
    event.preventDefault();
    if (event.pointerType === "touch") {
      state.pointer.touches.set(event.pointerId, canvasPoint(event));
    }
    const point = canvasPoint(event);
    const dx = point.x - state.pointer.previous.x;
    const dy = point.y - state.pointer.previous.y;
    if (state.pointer.selecting && state.selection) {
      const hit = groundIntersection(event);
      if (hit) {
        const pose = threeToRos(hit);
        state.selection.yaw = Math.atan2(pose.y - state.selection.y, pose.x - state.selection.x);
        updateSelectionDisplay();
        updateTargetMarkers();
      }
    } else if (state.pointer.button === 2 || state.pointer.button === 1) {
      panCameraByScreenDelta(dx, dy);
    } else {
      rotateCameraByScreenDelta(dx, dy);
    }
    state.pointer.previous = point;
  });
  const endPointer = (event) => {
    state.pointer.touches.delete(event.pointerId);
    state.pointer.touchStart = touchGesture();
    if (state.pointer.touchStart) {
      state.pointer.touchStart.cameraDistance = state.camera.distance;
      return;
    }
    state.pointer.active = false;
    state.pointer.selecting = false;
    state.pointer.previous = null;
  };
  mapCanvas3d.addEventListener("pointerup", endPointer);
  mapCanvas3d.addEventListener("pointercancel", endPointer);
  mapCanvas3d.addEventListener("pointerleave", endPointer);
}

function start() {
  attachEvents();
  resizeAll();
  setTab("3d");
  pollState();
  pollLogs();
  heartbeatViewer();
  pollMaps3d(true);
  pollMappingState();
  pollPose();
  setInterval(pollState, 1000);
  setInterval(pollMappingState, 1000);
  setInterval(pollPose, 100);
  setInterval(pollLogs, 1000);
  setInterval(heartbeatViewer, 3000);
  setInterval(() => pollMaps3d(false), 5000);
  setInterval(() => pollMapping3d(false), 5000);
  setInterval(pollMappingPath, 1000);
  setInterval(pollSystem, 1000);
  render();
}

start();
