/* ══════════════════════════════════════════════════════════════════════════
   charts.js — Real-time Plotly.js Charts
   ══════════════════════════════════════════════════════════════════════════ */

const Charts = {
  initialized: false,
  chartIds: ['chart-position', 'chart-error', 'chart-control', 'chart-xy'],

  // Light theme layout — hardcoded for report-ready screenshots
  getLayoutDefaults() {
    return {
      paper_bgcolor: 'rgba(0,0,0,0)',
      plot_bgcolor: '#ffffff',
      font: { family: 'Inter, sans-serif', size: 10, color: '#475569' },
      margin: { l: 45, r: 15, t: 30, b: 30 },
      xaxis: {
        gridcolor: 'rgba(0,0,0,0.08)',
        zerolinecolor: 'rgba(0,0,0,0.15)',
        tickfont: { family: 'JetBrains Mono', size: 9, color: '#64748b' },
      },
      yaxis: {
        gridcolor: 'rgba(0,0,0,0.08)',
        zerolinecolor: 'rgba(0,0,0,0.15)',
        tickfont: { family: 'JetBrains Mono', size: 9, color: '#64748b' },
      },
      legend: {
        orientation: 'h',
        x: 0, y: 1.15,
        font: { size: 9, color: '#475569' },
        bgcolor: 'rgba(0,0,0,0)',
      },
      showlegend: true,
    };
  },

  getColors() {
    return {
      teal:   '#0891b2',
      blue:   '#2563eb',
      purple: '#7c3aed',
      orange: '#ea580c',
      red:    '#dc2626',
      yellow: '#d97706',
      green:  '#059669',
    };
  },

  init() {
    if (this.initialized) return;
    this._createCharts();
    this.initialized = true;
  },

  // Re-create charts when theme changes
  reInit() {
    this.initialized = false;
    this._createCharts();
    this.initialized = true;
  },

  _createCharts() {
    const ld = this.getLayoutDefaults();
    const c = this.getColors();
    const titleColor = '#1e293b';

    const config = {
      displayModeBar: false,
      responsive: true,
    };

    // ── Chart 1: Position Tracking ────────────────────────────────
    Plotly.newPlot('chart-position', [
      { y: [], name: 'X actual', line: { color: c.teal, width: 1.5 } },
      { y: [], name: 'Y actual', line: { color: c.blue, width: 1.5 } },
      { y: [], name: 'Z actual', line: { color: c.purple, width: 1.5 } },
      { y: [], name: 'X setpoint', line: { color: c.teal, width: 1, dash: 'dot' } },
      { y: [], name: 'Y setpoint', line: { color: c.blue, width: 1, dash: 'dot' } },
      { y: [], name: 'Z setpoint', line: { color: c.purple, width: 1, dash: 'dot' } },
    ], {
      ...ld,
      title: { text: 'Position vs Setpoint', font: { size: 12, color: titleColor } },
      yaxis: { ...ld.yaxis, title: { text: 'm', font: { size: 9 } } },
    }, config);

    // ── Chart 2: Error Magnitude ─────────────────────────────────
    Plotly.newPlot('chart-error', [
      { y: [], name: 'Error X', line: { color: c.teal, width: 1.5 } },
      { y: [], name: 'Error Y', line: { color: c.blue, width: 1.5 } },
      { y: [], name: 'Error Z', line: { color: c.purple, width: 1.5 } },
      { y: [], name: '|Error|', line: { color: c.orange, width: 2 } },
    ], {
      ...ld,
      title: { text: 'Position Error', font: { size: 12, color: titleColor } },
      yaxis: { ...ld.yaxis, title: { text: 'm', font: { size: 9 } } },
    }, config);

    // ── Chart 3: Controller Output ───────────────────────────────
    Plotly.newPlot('chart-control', [
      { y: [], name: 'Acc X', line: { color: c.teal, width: 1.5 } },
      { y: [], name: 'Acc Y', line: { color: c.blue, width: 1.5 } },
      { y: [], name: 'Acc Z', line: { color: c.purple, width: 1.5 } },
    ], {
      ...ld,
      title: { text: 'Controller Output (Acceleration)', font: { size: 12, color: titleColor } },
      yaxis: { ...ld.yaxis, title: { text: 'm/s²', font: { size: 9 } } },
    }, config);

    // ── Chart 4: XY Trajectory ───────────────────────────────────
    Plotly.newPlot('chart-xy', [
      { x: [], y: [], name: 'Actual Path', mode: 'lines',
        line: { color: c.orange, width: 2 } },
      { x: [], y: [], name: 'Planned Path', mode: 'lines+markers',
        line: { color: c.teal, width: 1.5, dash: 'dot' },
        marker: { size: 5, color: c.teal } },
      { x: [], y: [], name: 'Drone', mode: 'markers',
        marker: { size: 10, color: c.orange, symbol: 'triangle-up' } },
    ], {
      ...ld,
      title: { text: '2D Trajectory (XY Plane)', font: { size: 12, color: titleColor } },
      xaxis: { ...ld.xaxis, title: { text: 'Y (East) [m]', font: { size: 9 } }, scaleanchor: 'y' },
      yaxis: { ...ld.yaxis, title: { text: 'X (North) [m]', font: { size: 9 } } },
    }, config);
  },

  update(history, data) {
    if (!this.initialized) this.init();

    const maxPoints = 400;
    const slice = (arr) => arr.length > maxPoints ? arr.slice(-maxPoints) : arr;

    // ── Chart 1: Position vs Setpoint ─────────────────────────────
    Plotly.update('chart-position', {
      y: [
        slice(history.pos_x), slice(history.pos_y), slice(history.pos_z),
        slice(history.sp_x), slice(history.sp_y), slice(history.sp_z),
      ],
    });

    // ── Chart 2: Error ────────────────────────────────────────────
    const errMag = history.err_x.map((ex, i) => {
      const ey = history.err_y[i] || 0;
      const ez = history.err_z[i] || 0;
      return Math.sqrt(ex * ex + ey * ey + ez * ez);
    });
    Plotly.update('chart-error', {
      y: [
        slice(history.err_x), slice(history.err_y), slice(history.err_z),
        slice(errMag),
      ],
    });

    // ── Chart 3: Controller Output ────────────────────────────────
    Plotly.update('chart-control', {
      y: [
        slice(history.acc_x), slice(history.acc_y), slice(history.acc_z),
      ],
    });

    // ── Chart 4: XY Trajectory ────────────────────────────────────
    const plannedX = [];
    const plannedY = [];
    if (typeof TrajectoryCanvas !== 'undefined' && TrajectoryCanvas.waypoints.length > 0) {
      for (const wp of TrajectoryCanvas.waypoints) {
        plannedX.push(wp.y);  // East
        plannedY.push(wp.x);  // North
      }
    }

    Plotly.update('chart-xy', {
      x: [
        slice(history.actual_path_y),  // East
        plannedX,
        [data.position.y || 0],
      ],
      y: [
        slice(history.actual_path_x),  // North
        plannedY,
        [data.position.x || 0],
      ],
    });
  },
};

/* ── Init ──────────────────────────────────────────────────────────────── */
window.addEventListener('DOMContentLoaded', () => {
  // Delay chart init slightly to ensure DOM is ready
  setTimeout(() => Charts.init(), 100);
});
