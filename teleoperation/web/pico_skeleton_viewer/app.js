import * as THREE from "./vendor/three.module.min.js";
import { OrbitControls } from "./vendor/OrbitControls.js";

const sceneHost = document.querySelector("#scene");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111517);

const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 50);
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
sceneHost.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0, 0);

scene.add(new THREE.HemisphereLight(0xeafcff, 0x263034, 2.4));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.0);
keyLight.position.set(2, 3, 4);
scene.add(keyLight);

const grid = new THREE.GridHelper(2.4, 24, 0x3d4a4d, 0x273033);
grid.position.y = -0.78;
scene.add(grid);

const axes = new THREE.AxesHelper(0.22);
axes.position.set(-0.92, -0.76, 0);
scene.add(axes);

const colors = {
  left: 0x55d9e8,
  right: 0xff826d,
  torso: 0xc8d0d2,
};
const jointNames = [
  "waist", "neck", "head",
  "left_shoulder", "left_elbow", "left_wrist", "left_hand",
  "right_shoulder", "right_elbow", "right_wrist", "right_hand",
];
const connections = [
  ["waist", "neck", "torso"], ["neck", "head", "torso"],
  ["neck", "left_shoulder", "left"], ["left_shoulder", "left_elbow", "left"],
  ["left_elbow", "left_wrist", "left"], ["left_wrist", "left_hand", "left"],
  ["neck", "right_shoulder", "right"], ["right_shoulder", "right_elbow", "right"],
  ["right_elbow", "right_wrist", "right"], ["right_wrist", "right_hand", "right"],
];

const skeleton = new THREE.Group();
scene.add(skeleton);
const pointGeometry = new THREE.SphereGeometry(0.026, 18, 12);
const joints = new Map();
for (const name of jointNames) {
  const side = name.startsWith("left_") ? "left" : name.startsWith("right_") ? "right" : "torso";
  const mesh = new THREE.Mesh(
    pointGeometry,
    new THREE.MeshStandardMaterial({ color: colors[side], roughness: 0.38, metalness: 0.12 }),
  );
  mesh.visible = false;
  skeleton.add(mesh);
  joints.set(name, mesh);
}

const bones = connections.map(([from, to, side]) => {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6), 3));
  const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: colors[side] }));
  line.visible = false;
  skeleton.add(line);
  return { from, to, line };
});

let mode = "raw";
let paused = false;
let latestState = null;
let latestFrames = {};
let latestDiagnostics = null;

function resetCamera() {
  if (mode === "raw") {
    camera.position.set(1.65, 0.55, 2.15);
    controls.target.set(0, -0.02, 0);
  } else {
    camera.position.set(0, 0.12, 2.55);
    controls.target.set(0, -0.08, 0);
  }
  controls.update();
}
resetCamera();

function matrixFromState(state) {
  const values = state?.source_to_robot_rotation;
  return Array.isArray(values) && values.length === 9
    ? values
    : [0, 0, -1, -1, 0, 0, 0, 1, 0];
}

function transformPoint(pos, state, selectedMode = mode) {
  let [x, y, z] = pos.map(Number);
  if (selectedMode === "robot") {
    const r = matrixFromState(state);
    [x, y, z] = [
      r[0] * x + r[1] * y + r[2] * z,
      r[3] * x + r[4] * y + r[5] * z,
      r[6] * x + r[7] * y + r[8] * z,
    ];
  }
  return [x, y, z];
}

function displayPoint(pos, state, selectedMode = mode) {
  if (selectedMode === "raw") {
    // Preserve the original PICO/Unity motion axes in the acquisition view.
    // This layer must not pass through the robot retargeting rotation.
    return new THREE.Vector3(pos[0], pos[2], -pos[1]);
  }
  return new THREE.Vector3(-pos[1], pos[2], -pos[0]);
}

function framePosition(name, state, selectedMode = mode) {
  const diagnosticFrames = latestDiagnostics?.frames?.[selectedMode];
  const frame = ["retarget", "ik", "command", "feedback"].includes(selectedMode)
    ? diagnosticFrames?.[name]
    : latestFrames[name];
  if (!frame || !Array.isArray(frame.pos) || frame.pos.length < 3) return null;
  return ["raw", "robot"].includes(selectedMode)
    ? transformPoint(frame.pos, state, selectedMode)
    : frame.pos.map(Number);
}

function skeletonOrigin(state) {
  const left = framePosition("left_shoulder", state);
  const right = framePosition("right_shoulder", state);
  const waist = framePosition("waist", state);
  if (left && right) return left.map((value, index) => (value + right[index]) * 0.5);
  return left || right || waist || [0, 0, 0];
}

function updateSkeleton(state) {
  const origin = skeletonOrigin(state);
  for (const [name, mesh] of joints) {
    const pos = framePosition(name, state);
    mesh.visible = Boolean(pos);
    if (pos) {
      const relative = pos.map((value, index) => value - origin[index]);
      mesh.position.copy(displayPoint(relative, state));
    }
  }
  for (const { from, to, line } of bones) {
    const a = joints.get(from);
    const b = joints.get(to);
    line.visible = a.visible && b.visible;
    if (!line.visible) continue;
    const values = line.geometry.attributes.position.array;
    values.set([a.position.x, a.position.y, a.position.z, b.position.x, b.position.y, b.position.z]);
    line.geometry.attributes.position.needsUpdate = true;
    line.geometry.computeBoundingSphere();
  }
}

function distance(a, b) {
  if (!a || !b) return null;
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function metric(id, value) {
  document.querySelector(id).textContent = value == null ? "-- m" : `${value.toFixed(3)} m`;
}

function updateDiagnostics(state) {
  const packet = state.packet || {};
  latestFrames = packet.frames && typeof packet.frames === "object" ? packet.frames : {};
  latestDiagnostics = state.diagnostics;
  const diagnosticModes = ["retarget", "ik", "command", "feedback"];
  const diagnosticFrames = latestDiagnostics?.frames?.[mode];
  const diagnosticsFresh = Number.isFinite(state.diagnostics_age_ms)
    && state.diagnostics_age_ms <= 500;
  const layerUnavailable = diagnosticModes.includes(mode) && (!diagnosticsFresh || !diagnosticFrames);
  const statusDot = document.querySelector("#status-dot");
  statusDot.className = `status-dot ${state.connected ? "live" : state.packet ? "stale" : "waiting"}`;
  document.querySelector("#status-text").textContent = layerUnavailable
    ? "所选控制层暂无数据；请启动预览并等待完整输入"
    : state.connected ? "实时数据正常" : state.packet ? "数据已停止" : "等待骨架数据";
  const layerLabels = {raw:"原始采集层", robot:"固定坐标映射", retarget:"人体重定向目标",
    ik:"IK 正解结果", command:"本机计算输出", feedback:"机器人实测反馈"};
  document.querySelector("#layer-status").textContent = layerUnavailable
    ? `${layerLabels[mode]} · 等待数据` : layerLabels[mode];
  document.querySelector("#rate").textContent = `${Number(state.source_update_hz || 0).toFixed(1)} Hz`;
  document.querySelector("#sequence").textContent = packet.sequence == null ? "序列 --" : `序列 ${packet.sequence}`;
  const visibleJointCount = jointNames.filter((name) => framePosition(name, state)).length;
  document.querySelector("#valid-count").textContent = `${visibleJointCount} / ${jointNames.length}`;

  for (const side of ["left", "right"]) {
    const shoulder = framePosition(`${side}_shoulder`, state);
    const elbow = framePosition(`${side}_elbow`, state);
    const wrist = framePosition(`${side}_wrist`, state);
    metric(`#${side}-upper`, distance(shoulder, elbow));
    metric(`#${side}-fore`, distance(elbow, wrist));
  }

  const labels = [
    ["left_shoulder", "左肩", "left"], ["left_elbow", "左肘", "left"], ["left_wrist", "左腕", "left"],
    ["right_shoulder", "右肩", "right"], ["right_elbow", "右肘", "right"], ["right_wrist", "右腕", "right"],
  ];
  document.querySelector("#coordinate-rows").innerHTML = labels.map(([name, label, side]) => {
    const pos = framePosition(name, state);
    const values = pos ? pos.map((value) => value.toFixed(3)) : ["--", "--", "--"];
    return `<tr class="${side}"><td>${label}</td><td>${values[0]}</td><td>${values[1]}</td><td>${values[2]}</td></tr>`;
  }).join("");
  const modeLabels = { raw: "PICO", robot: "FIXED", retarget: "RETARGET", ik: "IK", command: "COMMAND", feedback: "FEEDBACK" };
  document.querySelector("#coordinate-frame").textContent = modeLabels[mode];
  const r = matrixFromState(state);
  document.querySelector("#transform-matrix").textContent = [0, 1, 2]
    .map((row) => `[ ${r.slice(row * 3, row * 3 + 3).map((v) => Number(v).toFixed(0).padStart(2)).join(" ")} ]`)
    .join("\n");

  const jointData = latestDiagnostics?.joints_deg || {};
  const diagnosticSide = latestDiagnostics?.side;
  document.querySelector("#joint-side-title").textContent = diagnosticSide === "left"
    ? "左臂关节角" : diagnosticSide === "right" ? "右臂关节角" : "关节角";
  const jointValue = (layer, index) => {
    const values = jointData[layer];
    return Array.isArray(values) && Number.isFinite(values[index]) ? values[index].toFixed(1) : "--";
  };
  document.querySelector("#joint-rows").innerHTML = Array.from({ length: 7 }, (_, index) =>
    `<tr><td>J${index + 1}</td><td>${jointValue("ik", index)}</td><td>${jointValue("command", index)}</td><td>${jointValue("feedback", index)}</td></tr>`
  ).join("");

  const errors = latestDiagnostics?.errors || {};
  const errorText = (layer, first, second, unit) => {
    const values = errors[layer];
    const a = Number.isFinite(values?.[first]) ? values[first].toFixed(1) : "--";
    const b = Number.isFinite(values?.[second]) ? values[second].toFixed(1) : "--";
    return `${a} / ${b} ${unit}`;
  };
  document.querySelector("#ik-position-error").textContent = errorText("ik_vs_retarget", "elbow_cm", "wrist_cm", "cm");
  document.querySelector("#ik-direction-error").textContent = errorText("ik_vs_retarget", "upper_deg", "forearm_deg", "deg");
  document.querySelector("#feedback-position-error").textContent = errorText("feedback_vs_retarget", "elbow_cm", "wrist_cm", "cm");
  document.querySelector("#feedback-direction-error").textContent = errorText("feedback_vs_retarget", "upper_deg", "forearm_deg", "deg");
}

function setMode(nextMode) {
  mode = nextMode;
  resetCamera();
  if (latestState) {
    updateDiagnostics(latestState);
    updateSkeleton(latestState);
  }
}

document.querySelector("#view-mode").addEventListener("change", (event) => setMode(event.target.value));
document.querySelector("#reset-camera").addEventListener("click", resetCamera);
document.querySelector("#pause").addEventListener("click", (event) => {
  paused = !paused;
  event.currentTarget.textContent = paused ? "继续" : "暂停";
});

async function poll() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    if (!paused) {
      latestState = state;
      updateDiagnostics(state);
      updateSkeleton(state);
    }
  } catch (_error) {
    document.querySelector("#status-dot").className = "status-dot stale";
    document.querySelector("#status-text").textContent = "调试服务断开";
  }
}
setInterval(poll, 50);
poll();

function resize() {
  const width = Math.max(sceneHost.clientWidth, 1);
  const height = Math.max(sceneHost.clientHeight, 1);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
}
new ResizeObserver(resize).observe(sceneHost);
resize();

function animate() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();
