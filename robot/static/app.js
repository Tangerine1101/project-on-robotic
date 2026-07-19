/* Robot arm dashboard front-end: status polling, joint-history chart,
 * auto/manual toggle and the manual joint-move form. Vanilla JS + Chart.js
 * (vendored in chart.umd.js so the page works with no internet). */
"use strict";

const JOINT_LABELS = ["J1 (a)", "J2 (b)", "J3 (c)", "J4 (d)", "Grip (e)"];
const JOINT_COLORS = ["#4da3ff", "#f0c674", "#7ee2a8", "#ff8f8f", "#c792ea"];
const WINDOW_SEC = 60; // chart shows the last minute

/* ------------------------------------------------------------- tabs */
document.querySelectorAll(".tab").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".tabpane").forEach((p) => p.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
  });
});

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

/* ------------------------------------------------------------- status */
const pillMode = document.getElementById("pill-mode");
const pillState = document.getElementById("pill-state");
const pillLink = document.getElementById("pill-link");
const autoToggle = document.getElementById("auto-toggle");
const modeHint = document.getElementById("mode-hint");
const sendBtn = document.getElementById("send-btn");

async function pollStatus() {
  try {
    const res = await fetch("/api/status");
    const s = await res.json();
    pillMode.textContent = "mode: " + s.mode;
    pillMode.className = "pill " + (s.mode === "auto" ? "good" : "warn");
    pillState.textContent = "state: " + s.state;
    pillLink.textContent = `link: ${s.link_status[0]}/${s.link_status[1]}`;
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
    if (document.activeElement !== autoToggle) autoToggle.checked = s.mode === "auto";
    sendBtn.disabled = s.mode === "auto";
    modeHint.textContent = s.mode === "auto"
      ? "Planner is running the sorting cycle; manual moves are blocked."
      : "Planner is idle; manual moves are allowed.";
    if (!s.vision) {
      document.getElementById("video").hidden = true;
      document.getElementById("video-off").hidden = false;
    }
  } catch (e) { /* keep polling */ }
}
setInterval(pollStatus, 500);
pollStatus();

/* ------------------------------------------------------------- mode toggle */
autoToggle.addEventListener("change", async () => {
  await fetch("/api/mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ auto: autoToggle.checked }),
  });
  pollStatus();
});

/* ------------------------------------------------------------- manual move */
document.getElementById("move-form").addEventListener("submit", async (ev) => {
  ev.preventDefault();
  const result = document.getElementById("move-result");
  const body = {};
  for (const name of ["x", "y", "z", "w", "e"]) {
    const val = ev.target.elements[name].value.trim();
    if (val !== "") body[name] = parseFloat(val);
  }
  if (!Object.keys(body).length) {
    result.textContent = "enter at least one angle";
    result.className = "err";
    return;
  }
  sendBtn.disabled = true;
  result.textContent = "moving…";
  result.className = "";
  try {
    const res = await fetch("/api/move", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await res.json();
    if (res.ok && data.ok) {
      result.textContent = "done";
      result.className = "ok";
    } else {
      result.textContent = data.error || "move failed / timed out";
      result.className = "err";
    }
  } catch (e) {
    result.textContent = "request failed";
    result.className = "err";
  } finally {
    sendBtn.disabled = autoToggle.checked;
  }
});
