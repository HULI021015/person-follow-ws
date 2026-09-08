import * as THREE from "./vendor/three.module.min.js";
import { OrbitControls } from "./vendor/OrbitControls.js";
import { STLLoader } from "./vendor/STLLoader.js";

const canvas = document.getElementById("arm-canvas");
const overlay = document.getElementById("urdf-overlay");
const joints = new Map();
const targetJointPositions = new Map();
const displayedJointPositions = new Map();
let latestJointState = { names: [], positions: [] };

function setOverlay(message, error = false) {
  overlay.textContent = message;
  overlay.classList.toggle("error", error);
  overlay.hidden = !message;
}

function values(text, fallback = [0, 0, 0]) {
  if (!text) return [...fallback];
  const parsed = text.trim().split(/\s+/).map(Number);
  return parsed.length === fallback.length && parsed.every(Number.isFinite)
    ? parsed
    : [...fallback];
}

function child(element, tagName) {
  return [...element.children].find((item) => item.tagName.toLowerCase() === tagName) || null;
}

function children(element, tagName) {
  return [...element.children].filter((item) => item.tagName.toLowerCase() === tagName);
}

function applyOrigin(object, origin) {
  if (!origin) return;
  object.position.fromArray(values(origin.getAttribute("xyz")));
  const [roll, pitch, yaw] = values(origin.getAttribute("rpy"));
  // URDF uses fixed-axis roll/pitch/yaw: R = Rz(yaw) * Ry(pitch) * Rx(roll).
  // THREE.Euler's order semantics are not interchangeable with that rule for
  // Piper's +/-90 degree joint origins, so compose the quaternion explicitly.
  const qx = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(1, 0, 0), roll
  );
  const qy = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 1, 0), pitch
  );
  const qz = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 0, 1), yaw
  );
  object.quaternion.copy(qz).multiply(qy).multiply(qx);
}

function modelUrl(filename) {
  const marker = "package://piper_description/meshes/";
  if (!filename.startsWith(marker)) throw new Error(`不支持的模型地址：${filename}`);
  return `/model/meshes/${encodeURIComponent(filename.slice(marker.length))}`;
}

function materialFor(visual) {
  const colorElement = child(child(visual, "material") || document.createElement("material"), "color");
  const rgba = values(colorElement?.getAttribute("rgba"), [0.79, 0.82, 0.93, 1]);
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
    opacity: rgba[3],
    transparent: rgba[3] < 1,
    roughness: 0.68,
    metalness: 0.08,
    side: THREE.DoubleSide,
  });
}

let renderer;
try {
  renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: false,
    powerPreference: "high-performance",
  });
} catch (error) {
  setOverlay(`WebGL 初始化失败：${error.message}`, true);
  throw error;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.setClearColor(0x080d11, 1);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 100);
camera.up.set(0, 0, 1);
camera.position.set(0.85, -0.95, 0.62);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0, 0.28);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.18;
controls.maxDistance = 4;

scene.add(new THREE.HemisphereLight(0xe9f4ff, 0x17232b, 2.1));
const keyLight = new THREE.DirectionalLight(0xffffff, 3.2);
keyLight.position.set(1.2, -1.0, 1.8);
scene.add(keyLight);
const fillLight = new THREE.DirectionalLight(0x7de9d8, 1.2);
fillLight.position.set(-1.1, 0.5, 0.8);
scene.add(fillLight);

const grid = new THREE.GridHelper(1.4, 14, 0x31414b, 0x1c2931);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.002;
scene.add(grid);
const axes = new THREE.AxesHelper(0.16);
scene.add(axes);

const robotRoot = new THREE.Group();
scene.add(robotRoot);

function updateJoints(names, positions) {
  latestJointState = { names: [...names], positions: [...positions] };
  names.forEach((name, index) => {
    const position = Number(positions[index]);
    if (!Number.isFinite(position)) return;
    targetJointPositions.set(name, position);
    if (!displayedJointPositions.has(name)) {
      displayedJointPositions.set(name, position);
    }
  });
}

function applyJointTransforms(deltaSeconds, snap = false) {
  const blend = snap ? 1 : 1 - Math.exp(-24 * Math.min(deltaSeconds, 0.1));
  targetJointPositions.forEach((target, name) => {
    const joint = joints.get(name);
    if (!joint) return;
    const previous = displayedJointPositions.get(name) ?? target;
    const position = previous + (target - previous) * blend;
    displayedJointPositions.set(
      name, Math.abs(target - position) < 1e-6 ? target : position
    );
    joint.motion.position.set(0, 0, 0);
    joint.motion.quaternion.identity();
    if (joint.type === "prismatic") {
      joint.motion.position.copy(joint.axis).multiplyScalar(position);
    } else if (joint.type === "revolute" || joint.type === "continuous") {
      joint.motion.quaternion.setFromAxisAngle(joint.axis, position);
    }
  });
}

window.addEventListener("piper-joint-state", (event) => {
  updateJoints(event.detail.names || [], event.detail.positions || []);
});

// EventSource keeps one lightweight HTTP connection open and receives the
// newest Piper sample at up to 30 Hz. It reconnects automatically after a
// dashboard restart; the slower /api/status event above remains a fallback.
const jointEvents = new EventSource("/stream/joints.sse");
jointEvents.onmessage = (event) => {
  try {
    const data = JSON.parse(event.data);
    updateJoints(data.names || [], data.positions || []);
  } catch (error) {
    console.warn("关节数据解析失败", error);
  }
};

function fitCamera() {
  const box = new THREE.Box3().setFromObject(robotRoot);
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const radius = Math.max(size.length() * 0.55, 0.25);
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(radius * 1.45, -radius * 1.65, radius * 1.05));
  camera.near = Math.max(radius / 100, 0.002);
  camera.far = radius * 100;
  camera.updateProjectionMatrix();
  controls.update();
}

async function loadRobot() {
  setOverlay("正在加载 Piper URDF 与 STL 模型…");
  const response = await fetch("/model/piper_description.urdf", { cache: "no-store" });
  if (!response.ok) throw new Error(`URDF HTTP ${response.status}`);
  const xml = new DOMParser().parseFromString(await response.text(), "application/xml");
  const parseError = xml.querySelector("parsererror");
  if (parseError) throw new Error("URDF XML 解析失败");
  const robot = xml.querySelector("robot");
  if (!robot) throw new Error("URDF 中没有 robot 根节点");

  const linkGroups = new Map();
  const meshLoads = [];
  const loader = new STLLoader();

  children(robot, "link").forEach((linkElement) => {
    const linkGroup = new THREE.Group();
    linkGroup.name = linkElement.getAttribute("name") || "link";
    linkGroups.set(linkGroup.name, linkGroup);

    children(linkElement, "visual").forEach((visual) => {
      const meshElement = child(child(visual, "geometry") || document.createElement("geometry"), "mesh");
      if (!meshElement) return;
      const visualGroup = new THREE.Group();
      applyOrigin(visualGroup, child(visual, "origin"));
      linkGroup.add(visualGroup);
      const scale = values(meshElement.getAttribute("scale"), [1, 1, 1]);
      const url = modelUrl(meshElement.getAttribute("filename") || "");
      meshLoads.push(new Promise((resolve, reject) => {
        loader.load(url, (geometry) => {
          geometry.computeVertexNormals();
          const mesh = new THREE.Mesh(geometry, materialFor(visual));
          mesh.name = `${linkGroup.name}_visual`;
          mesh.scale.fromArray(scale);
          visualGroup.add(mesh);
          resolve();
        }, undefined, () => reject(new Error(`STL 加载失败：${url}`)));
      }));
    });
  });

  const childLinks = new Set();
  children(robot, "joint").forEach((jointElement) => {
    const name = jointElement.getAttribute("name") || "";
    const type = jointElement.getAttribute("type") || "fixed";
    const parentName = child(jointElement, "parent")?.getAttribute("link") || "";
    const childName = child(jointElement, "child")?.getAttribute("link") || "";
    const parentLink = linkGroups.get(parentName);
    const childLink = linkGroups.get(childName);
    if (!parentLink || !childLink) return;
    childLinks.add(childName);

    const originGroup = new THREE.Group();
    originGroup.name = `${name}_origin`;
    applyOrigin(originGroup, child(jointElement, "origin"));
    const motionGroup = new THREE.Group();
    motionGroup.name = name;
    originGroup.add(motionGroup);
    motionGroup.add(childLink);
    parentLink.add(originGroup);

    const axis = new THREE.Vector3(...values(child(jointElement, "axis")?.getAttribute("xyz"), [1, 0, 0]));
    if (axis.lengthSq() === 0) axis.set(1, 0, 0);
    axis.normalize();
    joints.set(name, { type, axis, motion: motionGroup });
  });

  const baseLink = [...linkGroups.entries()].find(([name]) => !childLinks.has(name))?.[1];
  if (!baseLink) throw new Error("无法确定 URDF 根连杆");
  robotRoot.add(baseLink);
  updateJoints(latestJointState.names, latestJointState.positions);
  applyJointTransforms(0, true);
  await Promise.all(meshLoads);
  fitCamera();
  setOverlay("");
}

function resize() {
  const width = Math.max(canvas.clientWidth, 1);
  const height = Math.max(canvas.clientHeight, 1);
  const pixelRatio = renderer.getPixelRatio();
  if (canvas.width !== Math.round(width * pixelRatio) || canvas.height !== Math.round(height * pixelRatio)) {
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
}

new ResizeObserver(resize).observe(canvas);
const animationClock = new THREE.Clock();
renderer.setAnimationLoop(() => {
  resize();
  applyJointTransforms(animationClock.getDelta());
  controls.update();
  renderer.render(scene, camera);
});

loadRobot().catch((error) => {
  console.error(error);
  setOverlay(`模型加载失败：${error.message}`, true);
});
