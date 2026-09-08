const byId = (id) => document.getElementById(id);
const buttons = [...document.querySelectorAll("button[data-action]")];
const state = { joints: [] };

function ageText(value) {
  if (value === null || value === undefined) return "无数据";
  if (value < 1) return "实时";
  return `${value.toFixed(1)} 秒前`;
}

function setText(id, value) { byId(id).textContent = value; }

function update(data) {
  const f = data.follow;
  const a = data.arm;
  byId("connection-text").textContent = "ROS 2 数据连接正常";
  byId("connection-dot").parentElement.classList.add("online");
  setText("direction", f.direction);
  setText("follow-process", f.running ? "跟随进程运行中" : "跟随进程未启动");
  setText("linear", Number(f.linear).toFixed(3));
  setText("angular", Number(f.angular).toFixed(3));
  setText("yaw", `${Number(f.odom.yaw_deg).toFixed(1)}°`);
  byId("compass-arrow").style.transform = `rotate(${f.odom.yaw_deg}deg)`;
  setText("follow-camera-age", ageText(f.image_age));

  setText("arm-camera-age", ageText(a.image_age));
  const armProcessState = a.stopping ? "机械臂回零停止中" : a.running ? "机械臂运行中" : "机械臂未启动";
  setText("joint-age", `${armProcessState} · ${ageText(a.joint_age)}`);
  const armStopButton = document.querySelector('button[data-action="arm/stop"]');
  if (armStopButton) armStopButton.disabled = Boolean(a.stopping);
  setText("grasp-status", a.grasp_status || "无任务状态");
  setText("grasp-result", String(a.grasp_result));
  setText("end-position", `${a.end_pose.position.map((v) => Number(v).toFixed(3)).join(" / ")} m`);
  state.joints = a.joint_positions || [];
  const jointNames = a.joint_names || [];
  renderJointList(jointNames, state.joints);
  window.dispatchEvent(new CustomEvent("piper-joint-state", {
    detail: { names: jointNames, positions: state.joints },
  }));
  const logBox = byId("logs");
  const nearBottom = logBox.scrollHeight - logBox.scrollTop - logBox.clientHeight < 35;
  logBox.textContent = data.logs.length ? data.logs.join("\n") : "等待节点输出…";
  if (nearBottom) logBox.scrollTop = logBox.scrollHeight;
}

function renderJointList(names, positions) {
  const container = byId("joint-list");
  container.replaceChildren();
  positions.slice(0, 7).forEach((value, index) => {
    const item = document.createElement("div");
    const name = document.createElement("span");
    const number = document.createElement("b");
    name.textContent = names[index] || `joint${index + 1}`;
    number.textContent = index === 6 ? `${(value * 1000).toFixed(1)} mm` : `${(value * 180 / Math.PI).toFixed(1)}°`;
    item.append(name, number);
    container.append(item);
  });
}

function drawArm(joints) {
  const canvas = byId("arm-canvas");
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (canvas.width !== width * ratio || canvas.height !== height * ratio) {
    canvas.width = width * ratio; canvas.height = height * ratio;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const multiply = (a, b) => {
    const out = Array(16).fill(0);
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++)
      for (let k = 0; k < 4; k++) out[r * 4 + c] += a[r * 4 + k] * b[k * 4 + c];
    return out;
  };
  const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const translate = ([x, y, z]) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
  const rx = (a) => [1, 0, 0, 0, 0, Math.cos(a), -Math.sin(a), 0, 0, Math.sin(a), Math.cos(a), 0, 0, 0, 0, 1];
  const ry = (a) => [Math.cos(a), 0, Math.sin(a), 0, 0, 1, 0, 0, -Math.sin(a), 0, Math.cos(a), 0, 0, 0, 0, 1];
  const rz = (a) => [
    Math.cos(a), -Math.sin(a), 0, 0,
    Math.sin(a), Math.cos(a), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ];
  const rpy = ([r, p, y]) => multiply(multiply(rz(y), ry(p)), rx(r));
  const point = (m, [x, y, z] = [0, 0, 0]) => [
    m[0] * x + m[1] * y + m[2] * z + m[3],
    m[4] * x + m[5] * y + m[6] * z + m[7],
    m[8] * x + m[9] * y + m[10] * z + m[11],
  ];
  const scale = Math.min(width, height) * 1.12;
  const project = ([x, y, z]) => [
    width * .49 + (x - y) * scale * .72,
    height * .88 - z * scale + (x + y) * scale * .27,
  ];

  ctx.strokeStyle = "#25323a"; ctx.lineWidth = 1;
  for (let n = -4; n <= 4; n++) {
    const a = project([n * .1, -.4, 0]), b = project([n * .1, .4, 0]);
    const c = project([-.4, n * .1, 0]), d = project([.4, n * .1, 0]);
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(...c); ctx.lineTo(...d); ctx.stroke();
  }

  const q = Array.from({length: 6}, (_, i) => Number(joints[i] || 0));
  const chain = [
    {xyz: [0, 0, .123], rpy: [0, 0, 0]},
    {xyz: [0, 0, 0], rpy: [Math.PI / 2, -.1359, -Math.PI]},
    {xyz: [.28503, 0, 0], rpy: [0, 0, -1.7939]},
    {xyz: [-.021984, -.25075, 0], rpy: [Math.PI / 2, 0, 0]},
    {xyz: [0, 0, 0], rpy: [-Math.PI / 2, 0, 0]},
    {xyz: [.000088, -.091, 0], rpy: [Math.PI / 2, 0, 0]},
  ];
  let transform = identity();
  const world = [[0, 0, 0]];
  chain.forEach((joint, index) => {
    transform = multiply(transform, translate(joint.xyz));
    transform = multiply(transform, rpy(joint.rpy));
    transform = multiply(transform, rz(q[index]));
    world.push(point(transform));
  });
  const wristTransform = transform;
  world.push(point(transform, [0, 0, .1358]));
  const screen = world.map(project);

  ctx.lineCap = "round"; ctx.lineJoin = "round";
  ctx.strokeStyle = "#071013"; ctx.lineWidth = 18; ctx.beginPath();
  screen.forEach((p, i) => i ? ctx.lineTo(...p) : ctx.moveTo(...p)); ctx.stroke();
  const gradient = ctx.createLinearGradient(0, height, width, 0);
  gradient.addColorStop(0, "#5ee6d0"); gradient.addColorStop(1, "#8fa6ff");
  ctx.strokeStyle = gradient; ctx.lineWidth = 11; ctx.beginPath();
  screen.forEach((p, i) => i ? ctx.lineTo(...p) : ctx.moveTo(...p)); ctx.stroke();
  screen.forEach((p, i) => {
    ctx.fillStyle = i === screen.length - 1 ? "#ffad57" : "#edf3f6";
    ctx.beginPath(); ctx.arc(p[0], p[1], i === 0 ? 11 : 6, 0, Math.PI * 2); ctx.fill();
  });

  const camera = project(point(wristTransform, [-.08095, .00424, .04115]));
  ctx.fillStyle = "#ffad57"; ctx.fillRect(camera[0] - 6, camera[1] - 5, 12, 10);
  ctx.fillStyle = "#ffcf91"; ctx.font = "10px ui-monospace"; ctx.fillText("CAM", camera[0] + 9, camera[1] + 3);
  const origin = project([0, 0, 0]);
  [[.09, 0, 0, "#ff7069", "X"], [0, .09, 0, "#7ee787", "Y"], [0, 0, .09, "#73a9ff", "Z"]].forEach(([x, y, z, color, label]) => {
    const end = project([x, y, z]); ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(...origin); ctx.lineTo(...end); ctx.stroke();
    ctx.fillStyle = color; ctx.fillText(label, end[0] + 3, end[1]);
  });
}

async function refresh() {
  try {
    const response = await fetch("/api/status", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    update(await response.json());
  } catch (error) {
    byId("connection-text").textContent = `连接中断：${error.message}`;
    byId("connection-dot").parentElement.classList.remove("online");
  }
}

function toast(message, error = false) {
  const element = byId("toast");
  element.textContent = message;
  element.className = error ? "show error" : "show";
  setTimeout(() => { element.className = ""; }, 2850);
}

buttons.forEach((button) => button.addEventListener("click", async () => {
  const action = button.dataset.action;
  if (action === "grasp/start" && !confirm("确认夹爪周围无人、急停可用，并开始抓取 bottle？")) return;
  if (action === "arm/stop" && !confirm("机械臂将先取消当前任务并回到零点，确认到位后再停止节点。是否继续？")) return;
  button.disabled = true;
  try {
    const response = await fetch(`/api/${action}`, {method: "POST"});
    const body = await response.json();
    toast(body.message, !body.ok);
    await refresh();
  } catch (error) {
    toast(`操作失败：${error.message}`, true);
  } finally {
    button.disabled = false;
  }
}));

// A browser does not always reconnect an MJPEG <img> after the dashboard or
// camera process restarts.  Retry only on a real stream error; a stream that is
// merely waiting for its first ROS frame remains connected.
document.querySelectorAll("img[data-stream]").forEach((image) => {
  image.addEventListener("error", () => {
    window.setTimeout(() => {
      const separator = image.dataset.stream.includes("?") ? "&" : "?";
      image.src = `${image.dataset.stream}${separator}_=${Date.now()}`;
    }, 1000);
  });
});

refresh();
setInterval(refresh, 500);
