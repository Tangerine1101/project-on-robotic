/* Robot arm operator console: mode switch, live status polling, detected-object
 * list, manual controls (go-to-point via IK, joint move, grip/release/calibrate)
 * and a collapsible joint-history chart. Vanilla JS + Chart.js (vendored in
 * chart.umd.js so the page works with no internet). */
"use strict";

const JOINT_LABELS = ["J1", "J2", "J3", "J4", "Gripper"];
const JOINT_COLORS = ["#4da3ff", "#f0c674", "#7ee2a8", "#ff8f8f", "#c792ea"];
const WINDOW_SEC = 60; // chart shows the last minute

// Planner state -> plain-language text for the operator.
const STATE_TEXT = {
  WAIT_DEPS: "Waiting for camera",
  CALIBRATING: "Calibrating",
  MOVING_HOME: "Moving home",
  SCANNING: "Scanning for objects",
  MOVING_ABOVE_PICK: "Moving over object",
  MOVING_PICK: "Lowering to pick",
  GRIPPING: "Gripping",
  MOVING_LIFT: "Lifting",
  MOVING_TRAVERSE: "Moving to box",
  MOVING_DROP: "Lowering into box",
  RELEASING: "Releasing",
  MOVING_RETREAT: "Lifting from box",
  MANUAL: "Manual control",
};

/* ------------------------------------------------------------- chart */
const chart = new Chart(document.getElementById("chart"), {
  type: "line",
  data: {
    datasets: JOINT_LABELS.map((label, i) => ({
      label,
      borderColor: JOINT_COLORS[i],
      backgroundColor: JOINT_COLORS[i],
      borderWidth: 1.5,
      pointRadius: 0,
      data: [],
    })),
  },
  options: {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    interaction: { intersect: false, mode: "nearest" },
    scales: {
      x: {
        type: "linear",
        title: { display: true, text: "seconds ago" },
        min: -WINDOW_SEC,
        max: 0,
        ticks: { color: "#8592a3" },
        grid: { color: "#222b38" },
      },
      y: {
        title: { display: true, text: "deg" },
        ticks: { color: "#8592a3" },
        grid: { color: "#222b38" },
      },
    },
    plugins: {
      legend: { labels: { color: "#dde3ea", boxWidth: 14 } },
    },
  },
});

let lastSample = 0; // epoch seconds of the newest sample we have

async function pollHistory() {
  try {
    const res = await fetch(`/api/history?since=${lastSample}`);
    const data = await res.json();
    if (data.t.length) {
      lastSample = data.t[data.t.length - 1];
      for (let k = 0; k < data.t.length; k++) {
        for (let j = 0; j < 5; j++) {
          chart.data.datasets[j].data.push({ t: data.t[k], v: data.joints[k][j] });
        }
      }
    }
    // re-base x to "seconds ago" and drop points beyond the window
    const now = Date.now() / 1000;
    for (let j = 0; j < 5; j++) {
      const ds = chart.data.datasets[j].data;
      while (ds.length && now - ds[0].t > WINDOW_SEC + 2) ds.shift();
      for (const p of ds) { p.x = p.t - now; p.y = p.v; }
    }
    chart.update();
  } catch (e) { /* server briefly unreachable: keep polling */ }
}
setInterval(pollHistory, 500);

/* ------------------------------------------------------------- elements */
const pillMode = document.getElementById("pill-mode");
const pillState = document.getElementById("pill-state");
const pillLink = document.getElementById("pill-link");
const btnAuto = document.getElementById("btn-auto");
const btnManual = document.getElementById("btn-manual");
const manualFields = document.getElementById("manual-fields");
const manualHint = document.getElementById("manual-hint");
const detList = document.getElementById("detections");
const result = document.getElementById("action-result");

let isManual = false; // last known mode
let busy = false;     // a manual command is in flight

/* ------------------------------------------------------------- status poll */
async function pollStatus() {
  try {
    const res = await fetch("/api/status");
    const s = await res.json();

    isManual = s.mode === "manual";
    pillMode.textContent = "mode: " + s.mode;
    pillMode.className = "pill " + (s.mode === "auto" ? "good" : "warn");
    pillState.textContent = STATE_TEXT[s.state] || s.state;
    pillLink.textContent = `link: ${s.link_status[0]}/${s.link_status[1]}`;

    btnManual.classList.toggle("active", isManual);
    btnAuto.classList.toggle("active", !isManual);
    manualHint.textContent = isManual
      ? "You can move the arm below."
      : "Auto mode is running the sorting cycle. Switch to Manual to control the arm.";
    // Disable controls in auto mode or while a command is running.
    manualFields.disabled = !isManual || busy;

    s.joints.forEach((v, i) => {
      document.getElementById("j" + i).textContent = v.toFixed(2);
    });
    if (s.limit_switches) {
      ["A", "B", "C"].forEach((name, i) => {
        const el = document.getElementById("sw-" + name);
        const hit = s.limit_switches[i];
        el.textContent = `${name}: ${hit ? "AT LIMIT" : "open"}`;
        el.className = "sw" + (hit ? " on" : "");
      });
    }

    renderDetections(s.detections, s.vision);

    if (!s.vision) {
      document.getElementById("video").hidden = true;
      document.getElementById("video-off").hidden = false;
    }
  } catch (e) { /* keep polling */ }
}
setInterval(pollStatus, 500);
pollStatus();

function renderDetections(dets, vision) {
  if (!vision) {
    detList.innerHTML = '<li class="muted">Vision disabled.</li>';
    return;
  }
  if (!dets || !dets.length) {
    detList.innerHTML = '<li class="muted">No objects.</li>';
    return;
  }
  detList.innerHTML = dets.map((d) =>
    `<li><span class="det-name">${escapeHtml(d.name)}</span>` +
    `<span class="det-pos">(${d.x.toFixed(0)}, ${d.y.toFixed(0)}) mm</span></li>`
  ).join("");
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}

/* ------------------------------------------------------------- mode switch */
async function setMode(auto) {
  try {
    await fetch("/api/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ auto }),
    });
  } catch (e) { /* pollStatus will resync */ }
  pollStatus();
}
btnAuto.addEventListener("click", () => setMode(true));
btnManual.addEventListener("click", () => setMode(false));

/* ------------------------------------------------------------- actions */
function showResult(text, kind) {
  result.textContent = text;
  result.className = kind || "";
}

/* Run a manual action: block the panel, POST, report ok/error, unblock. */
async function runAction(label, url, body) {
  if (busy) return;
  busy = true;
  manualFields.disabled = true;
  showResult(`${label}…`, "working");
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: body ? JSON.stringify(body) : undefined,
    });
    const data = await res.json().catch(() => ({}));
    if (res.ok && data.ok) {
      showResult(`${label}: done`, "ok");
    } else {
      showResult(`${label}: ${data.error || "failed / timed out"}`, "err");
    }
  } catch (e) {
    showResult(`${label}: request failed`, "err");
  } finally {
    busy = false;
    manualFields.disabled = !isManual;
  }
}

/* Go to point (X,Y,Z mm) via inverse kinematics */
document.getElementById("goto-form").addEventListener("submit", (ev) => {
  ev.preventDefault();
  const body = {};
  for (const name of ["x", "y", "z"]) {
    const val = ev.target.elements[name].value.trim();
    if (val === "") { showResult("Enter X, Y and Z", "err"); return; }
    body[name] = parseFloat(val);
  }
  runAction("Move", "/api/goto", body);
});

/* Advanced joint move (J1-J4; gripper is handled by the buttons) */
document.getElementById("move-form").addEventListener("submit", (ev) => {
  ev.preventDefault();
  const body = {};
  for (const name of ["x", "y", "z", "w"]) {
    const val = ev.target.elements[name].value.trim();
    if (val !== "") body[name] = parseFloat(val);
  }
  if (!Object.keys(body).length) { showResult("Enter at least one joint angle", "err"); return; }
  runAction("Move joints", "/api/move", body);
});

document.getElementById("btn-grip").addEventListener("click", () => runAction("Grip", "/api/grip"));
document.getElementById("btn-release").addEventListener("click", () => runAction("Release", "/api/release"));
document.getElementById("btn-calibrate").addEventListener("click", () => runAction("Calibrate", "/api/calibrate"));
