/* ══════════════════════════════════════════════════════════════════════════
   app.js — Main Application Logic + WebSocket Manager
   ══════════════════════════════════════════════════════════════════════════ */

const App = {
  ws: null,
  connected: false,
  controllerMode: 'pid',  // 'pid' or 'lqr'
  startTime: Date.now(),
  telemetryHistory: {
    timestamps: [],
    pos_x: [], pos_y: [], pos_z: [],
    sp_x: [], sp_y: [], sp_z: [],
    err_x: [], err_y: [], err_z: [],
    acc_x: [], acc_y: [], acc_z: [],
    // For comparison mode
    pid_err_x: [], pid_err_y: [], pid_err_z: [],
    lqr_err_x: [], lqr_err_y: [], lqr_err_z: [],
    actual_path_x: [], actual_path_y: [],
  },
  maxHistoryLen: 600,  // 30 seconds at 20Hz

  init() {
    this.connectWebSocket();
    this.updateUptime();
    setInterval(() => this.updateUptime(), 1000);
  },

  connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const url = `${protocol}//${window.location.host}/ws`;

    this.ws = new WebSocket(url);

    this.ws.onopen = () => {
      this.connected = true;
      this.updateConnectionStatus(true);
      showToast('Connected to GCS server', 'success');
    };

    this.ws.onclose = () => {
      this.connected = false;
      this.updateConnectionStatus(false);
      // Reconnect after 2 seconds
      setTimeout(() => this.connectWebSocket(), 2000);
    };

    this.ws.onerror = () => {
      this.connected = false;
      this.updateConnectionStatus(false);
    };

    this.ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === 'telemetry') {
          this.handleTelemetry(msg.data, msg.processes);
        }
      } catch (e) {
        // Ignore parse errors
      }
    };
  },

  send(data) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data));
    }
  },

  handleTelemetry(data, processes) {
    const now = Date.now() / 1000;
    const h = this.telemetryHistory;

    // ── Push position data ────────────────────────────────────────
    h.timestamps.push(now);
    h.pos_x.push(data.position.x || 0);
    h.pos_y.push(data.position.y || 0);
    h.pos_z.push(data.position.z || 0);

    // Track actual path for XY plot
    h.actual_path_x.push(data.position.x || 0);
    h.actual_path_y.push(data.position.y || 0);

    // ── Parse debug data ──────────────────────────────────────────
    const debug = data.pid_debug || data.lqr_debug;
    if (debug) {
      h.err_x.push(debug.pos_err ? debug.pos_err[0] : 0);
      h.err_y.push(debug.pos_err ? debug.pos_err[1] : 0);
      h.err_z.push(debug.pos_err ? debug.pos_err[2] : 0);
      h.acc_x.push(debug.acc_cmd ? debug.acc_cmd[0] : 0);
      h.acc_y.push(debug.acc_cmd ? debug.acc_cmd[1] : 0);
      h.acc_z.push(debug.acc_cmd ? debug.acc_cmd[2] : 0);

      // Setpoint = position + error (approximate)
      h.sp_x.push((data.position.x || 0) + (debug.pos_err ? debug.pos_err[0] : 0));
      h.sp_y.push((data.position.y || 0) + (debug.pos_err ? debug.pos_err[1] : 0));
      h.sp_z.push((data.position.z || 0) + (debug.pos_err ? debug.pos_err[2] : 0));
    } else {
      h.err_x.push(0); h.err_y.push(0); h.err_z.push(0);
      h.acc_x.push(0); h.acc_y.push(0); h.acc_z.push(0);
      h.sp_x.push(data.position.x || 0);
      h.sp_y.push(data.position.y || 0);
      h.sp_z.push(data.position.z || 0);
    }

    // ── Trim history ──────────────────────────────────────────────
    const keys = Object.keys(h);
    for (const k of keys) {
      if (Array.isArray(h[k]) && h[k].length > this.maxHistoryLen) {
        h[k] = h[k].slice(-this.maxHistoryLen);
      }
    }

    // ── Update UI elements ────────────────────────────────────────
    this.updateStatusBar(data);
    this.updatePositionInfo(data);
    this.updateTelemetryPanel(data);
    this.updateProcessList(processes);

    // ── Update charts (throttled to ~5Hz for performance) ─────────
    if (!this._lastChartUpdate || now - this._lastChartUpdate > 0.2) {
      this._lastChartUpdate = now;
      if (typeof Charts !== 'undefined') Charts.update(h, data);
      if (typeof TrajectoryCanvas !== 'undefined') TrajectoryCanvas.updateDronePosition(data);
    }
  },

  updateConnectionStatus(connected) {
    const pill = document.getElementById('pill-connection');
    if (connected) {
      pill.className = 'pill active';
      pill.querySelector('.pill-label').textContent = 'Connected';
    } else {
      pill.className = 'pill danger';
      pill.querySelector('.pill-label').textContent = 'Disconnected';
    }
  },

  updateStatusBar(data) {
    // Armed status
    const pillArmed = document.getElementById('pill-armed');
    const armState = data.status.arming_state;
    if (armState === 2) {
      pillArmed.className = 'pill warning';
      pillArmed.querySelector('.pill-label').textContent = 'Armed';
    } else {
      pillArmed.className = 'pill';
      pillArmed.querySelector('.pill-label').textContent = 'Disarmed';
    }

    // Nav mode
    const pillMode = document.getElementById('pill-mode');
    const navState = data.status.nav_state;
    const modeNames = { 0: 'Manual', 2: 'Altitude', 3: 'Position', 14: 'Offboard', 4: 'Mission', 5: 'RTL', 8: 'Land' };
    const modeName = modeNames[navState] || `Nav:${navState}`;
    pillMode.querySelector('.pill-label').textContent = modeName;
    pillMode.className = navState === 14 ? 'pill active' : 'pill';

    // Battery
    const bat = data.battery;
    const fill = document.getElementById('battery-fill');
    const text = document.getElementById('battery-text');
    const pct = Math.max(0, Math.min(100, bat.remaining || 0));
    fill.style.width = pct + '%';
    fill.style.background = pct > 30 ? '#34d399' : pct > 15 ? '#fbbf24' : '#ef4444';
    text.textContent = (bat.voltage || 0).toFixed(1) + 'V';
  },

  updatePositionInfo(data) {
    const fmt = (v) => (v || 0).toFixed(2);
    document.getElementById('pos-x').textContent = fmt(data.position.x);
    document.getElementById('pos-y').textContent = fmt(data.position.y);
    document.getElementById('pos-z').textContent = fmt(data.position.z);
    document.getElementById('vel-x').textContent = fmt(data.velocity.vx);
    document.getElementById('vel-y').textContent = fmt(data.velocity.vy);
    document.getElementById('vel-z').textContent = fmt(data.velocity.vz);
  },

  updateTelemetryPanel(data) {
    const debug = data.pid_debug || data.lqr_debug;
    if (debug) {
      const fmt = (v) => (v || 0).toFixed(2);
      if (debug.acc_cmd) {
        document.getElementById('acc-x').textContent = fmt(debug.acc_cmd[0]);
        document.getElementById('acc-y').textContent = fmt(debug.acc_cmd[1]);
        document.getElementById('acc-z').textContent = fmt(debug.acc_cmd[2]);
      }
      if (debug.pos_err) {
        document.getElementById('err-x').textContent = fmt(debug.pos_err[0]);
        document.getElementById('err-y').textContent = fmt(debug.pos_err[1]);
        document.getElementById('err-z').textContent = fmt(debug.pos_err[2]);
      }
    }

    // Trajectory status
    const ts = data.trajectory_status;
    if (ts) {
      const info = document.getElementById('traj-info');
      const bar = document.getElementById('traj-progress-bar');
      info.textContent = `WP: ${ts.current_wp}/${ts.total_wp}` + (ts.done ? ' ✓ Done' : '');
      const pct = ts.total_wp > 0 ? (ts.current_wp / ts.total_wp * 100) : 0;
      bar.style.width = pct + '%';
    }
  },

  updateProcessList(processes) {
    if (!processes) return;
    const list = document.getElementById('process-list');
    if (!list._initialized) {
      list.innerHTML = '';
      for (const [key, proc] of Object.entries(processes)) {
        const item = document.createElement('div');
        item.className = 'process-item';
        item.id = `proc-${key}`;
        item.innerHTML = `
          <span class="process-dot ${proc.status}"></span>
          <span class="process-name">${proc.name}</span>
          <button class="process-btn" data-proc="${key}" data-action="toggle">
            ${proc.status === 'running' ? 'Stop' : 'Start'}
          </button>
        `;
        list.appendChild(item);
      }
      list._initialized = true;

      // Event listeners for process buttons
      list.addEventListener('click', (e) => {
        const btn = e.target.closest('.process-btn');
        if (!btn) return;
        const procName = btn.dataset.proc;
        const running = btn.textContent.trim() === 'Stop';
        App.send({ cmd: running ? 'stop_process' : 'launch_process', name: procName });
      });
    } else {
      // Update existing items
      for (const [key, proc] of Object.entries(processes)) {
        const item = document.getElementById(`proc-${key}`);
        if (item) {
          const dot = item.querySelector('.process-dot');
          dot.className = `process-dot ${proc.status}`;
          const btn = item.querySelector('.process-btn');
          btn.textContent = proc.status === 'running' ? 'Stop' : 'Start';
        }
      }
    }
  },

  updateUptime() {
    const elapsed = Math.floor((Date.now() - this.startTime) / 1000);
    const h = Math.floor(elapsed / 3600);
    const m = Math.floor((elapsed % 3600) / 60);
    const s = elapsed % 60;
    document.getElementById('uptime').textContent =
      `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;
  },

  clearHistory() {
    const h = this.telemetryHistory;
    for (const k of Object.keys(h)) {
      if (Array.isArray(h[k])) h[k] = [];
    }
    showToast('Data cleared', 'success');
  },

  exportCSV() {
    const h = this.telemetryHistory;
    let csv = 'timestamp,pos_x,pos_y,pos_z,sp_x,sp_y,sp_z,err_x,err_y,err_z,acc_x,acc_y,acc_z\n';
    for (let i = 0; i < h.timestamps.length; i++) {
      csv += [
        h.timestamps[i], h.pos_x[i], h.pos_y[i], h.pos_z[i],
        h.sp_x[i], h.sp_y[i], h.sp_z[i],
        h.err_x[i], h.err_y[i], h.err_z[i],
        h.acc_x[i], h.acc_y[i], h.acc_z[i]
      ].join(',') + '\n';
    }
    const blob = new Blob([csv], { type: 'text/csv' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `px4_telemetry_${new Date().toISOString().slice(0,19)}.csv`;
    a.click();
    showToast('CSV exported', 'success');
  }
};

/* ── Toast Notification ────────────────────────────────────────────────── */
function showToast(message, type = 'info') {
  const container = document.getElementById('toast-container');
  const toast = document.createElement('div');
  toast.className = `toast ${type}`;
  toast.textContent = message;
  container.appendChild(toast);
  setTimeout(() => toast.remove(), 4000);
}

/* ── Sidebar Navigation ────────────────────────────────────────────────── */
document.querySelectorAll('.nav-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.nav-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.sidebar-panel').forEach(p => p.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById(btn.dataset.panel).classList.add('active');
  });
});

/* ── Main Content Tab Navigation ───────────────────────────────────────── */
document.querySelectorAll('.main-tab').forEach(tab => {
  tab.addEventListener('click', () => {
    document.querySelectorAll('.main-tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.main-tab-panel').forEach(p => p.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById(tab.dataset.mainTab).classList.add('active');

    // Trigger resize for canvas when switching to trajectory tab
    if (tab.dataset.mainTab === 'tab-trajectory') {
      setTimeout(() => {
        if (typeof TrajectoryCanvas !== 'undefined') TrajectoryCanvas.resize();
      }, 50);
    }
    // Trigger Plotly relayout when switching to telemetry tab
    if (tab.dataset.mainTab === 'tab-telemetry') {
      setTimeout(() => {
        if (typeof Charts !== 'undefined' && Charts.relayoutAll) Charts.relayoutAll();
        // Fallback: manually trigger resize on Plotly divs
        ['chart-position', 'chart-error', 'chart-control', 'chart-xy'].forEach(id => {
          const el = document.getElementById(id);
          if (el && typeof Plotly !== 'undefined') {
            Plotly.Plots.resize(el);
          }
        });
      }, 50);
    }
  });
});

/* ── Theme Toggle ──────────────────────────────────────────────────────── */
function initTheme() {
  const saved = localStorage.getItem('gcs-theme') || 'dark';
  applyTheme(saved);
}

function applyTheme(theme) {
  document.documentElement.setAttribute('data-theme', theme);
  const btn = document.getElementById('theme-toggle');
  if (btn) btn.textContent = theme === 'light' ? '☀️' : '🌙';
  localStorage.setItem('gcs-theme', theme);

  // Re-draw canvas with new colors
  setTimeout(() => {
    if (typeof TrajectoryCanvas !== 'undefined') TrajectoryCanvas.resize();
    if (typeof Charts !== 'undefined') Charts.reInit();
  }, 50);
}

/* ── Init on load ──────────────────────────────────────────────────────── */
function bootGCS() {
  initTheme();

  // Theme toggle button
  const toggleBtn = document.getElementById('theme-toggle');
  if (toggleBtn) {
    toggleBtn.addEventListener('click', () => {
      const current = document.documentElement.getAttribute('data-theme') || 'dark';
      applyTheme(current === 'dark' ? 'light' : 'dark');
    });
  }

  App.init();
}

// Scripts are at bottom of <body>, so DOM is likely already ready.
// Handle both cases:
if (document.readyState === 'loading') {
  window.addEventListener('DOMContentLoaded', bootGCS);
} else {
  bootGCS();
}
