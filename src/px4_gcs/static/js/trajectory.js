/* ══════════════════════════════════════════════════════════════════════════
   trajectory.js — Canvas-based Trajectory Designer
   ══════════════════════════════════════════════════════════════════════════ */

const TrajectoryCanvas = {
  canvas: null,
  ctx: null,
  currentShape: 'square',
  waypoints: [],         // Generated/drawn waypoints [{x, y, z}]
  customPoints: [],      // Free-draw points (canvas coords)
  dronePos: null,        // Current drone position
  plannedPath: [],       // Waypoints in NED to overlay
  isDragging: false,
  dragIndex: -1,
  scale: 20,             // pixels per meter (auto-adjusted)
  offsetX: 0,
  offsetY: 0,
  canvasW: 0,
  canvasH: 0,

  init() {
    this.canvas = document.getElementById('trajectory-canvas');
    if (!this.canvas) return;
    this.ctx = this.canvas.getContext('2d');

    // Handle high-DPI displays
    this.resize();
    window.addEventListener('resize', () => this.resize());

    // Mouse events for free-draw
    this.canvas.addEventListener('mousedown', (e) => this.onMouseDown(e));
    this.canvas.addEventListener('mousemove', (e) => this.onMouseMove(e));
    this.canvas.addEventListener('mouseup', () => this.onMouseUp());
    this.canvas.addEventListener('dblclick', (e) => this.onDoubleClick(e));

    // Generate initial shape
    this.generateShape();
  },

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;
    if (w < 10 || h < 10) return; // hidden tab, skip
    this.canvas.width = w * window.devicePixelRatio;
    this.canvas.height = h * window.devicePixelRatio;
    this.ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
    this.canvasW = w;
    this.canvasH = h;
    this.autoScale();
    this.draw();
  },

  autoScale() {
    const w = this.canvasW;
    const h = this.canvasH;
    const padding = 40; // px margin on each side

    if (this.waypoints.length < 2) {
      this.scale = 20;
      this.offsetX = w / 2;
      this.offsetY = h / 2;
      return;
    }

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    for (const wp of this.waypoints) {
      if (wp.x < minX) minX = wp.x;
      if (wp.x > maxX) maxX = wp.x;
      if (wp.y < minY) minY = wp.y;
      if (wp.y > maxY) maxY = wp.y;
    }
    // Include drone position in bounds
    if (this.dronePos) {
      if (this.dronePos.x < minX) minX = this.dronePos.x;
      if (this.dronePos.x > maxX) maxX = this.dronePos.x;
      if (this.dronePos.y < minY) minY = this.dronePos.y;
      if (this.dronePos.y > maxY) maxY = this.dronePos.y;
    }

    const rangeX = maxX - minX || 1;
    const rangeY = maxY - minY || 1;
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;

    // scale: pixels per meter (North maps to cy via -x*scale, East maps to cx via +y*scale)
    const scaleH = (h - 2 * padding) / rangeX; // North range maps to vertical
    const scaleW = (w - 2 * padding) / rangeY; // East range maps to horizontal
    this.scale = Math.min(scaleH, scaleW, 60); // cap at 60 px/m
    if (this.scale < 2) this.scale = 2;

    // Offset so center of waypoints is center of canvas
    this.offsetX = w / 2 - centerY * this.scale;
    this.offsetY = h / 2 + centerX * this.scale;
  },

  // Convert NED (x=North, y=East) to canvas pixels
  nedToCanvas(x, y) {
    return {
      cx: this.offsetX + y * this.scale,  // East → right
      cy: this.offsetY - x * this.scale,  // North → up
    };
  },

  // Convert canvas pixels to NED
  canvasToNed(cx, cy) {
    return {
      x: (this.offsetY - cy) / this.scale,  // North
      y: (cx - this.offsetX) / this.scale,   // East
    };
  },

  generateShape() {
    const numPoints = parseInt(document.getElementById('traj-points').value) || 8;
    const size = parseFloat(document.getElementById('traj-size').value) || 3;
    const alt = parseFloat(document.getElementById('traj-alt').value) || 3;
    const cx = parseFloat(document.getElementById('traj-cx').value) || 0;
    const cy = parseFloat(document.getElementById('traj-cy').value) || 0;

    if (this.currentShape === 'custom') {
      // Don't regenerate for custom mode
      this.draw();
      return;
    }

    // Generate waypoints locally for preview
    this.waypoints = this._generateLocal(this.currentShape, cx, cy, -alt, size, numPoints);
    this.autoScale();
    this.draw();
  },

  _generateLocal(shape, cx, cy, alt, size, n) {
    const wps = [];
    const push = (x, y) => wps.push({ x, y, z: alt });

    switch (shape) {
      case 'square': {
        const h = size / 2;
        const corners = [[cx-h,cy-h],[cx+h,cy-h],[cx+h,cy+h],[cx-h,cy+h]];
        for (const [x, y] of corners) push(x, y);
        push(corners[0][0], corners[0][1]); // close
        break;
      }
      case 'circle':
        for (let i = 0; i <= n; i++) {
          const a = 2 * Math.PI * i / n;
          push(cx + size * Math.cos(a), cy + size * Math.sin(a));
        }
        break;
      case 'triangle':
        for (let i = 0; i < 3; i++) {
          const a = 2 * Math.PI * i / 3 - Math.PI / 2;
          push(cx + size * Math.cos(a), cy + size * Math.sin(a));
        }
        push(cx + size * Math.cos(-Math.PI/2), cy + size * Math.sin(-Math.PI/2));
        break;
      case 'hexagon':
        for (let i = 0; i <= 6; i++) {
          const a = 2 * Math.PI * i / 6;
          push(cx + size * Math.cos(a), cy + size * Math.sin(a));
        }
        break;
      case 'octagon':
        for (let i = 0; i <= 8; i++) {
          const a = 2 * Math.PI * i / 8;
          push(cx + size * Math.cos(a), cy + size * Math.sin(a));
        }
        break;
      case 'figure8':
        for (let i = 0; i <= n; i++) {
          const t = 2 * Math.PI * i / n;
          const d = 1 + Math.sin(t) ** 2;
          push(cx + size * Math.cos(t) / d, cy + size * Math.sin(t) * Math.cos(t) / d);
        }
        break;
      case 'star': {
        const pts = 5;
        for (let i = 0; i <= pts * 2; i++) {
          const a = Math.PI * i / pts - Math.PI / 2;
          const r = i % 2 === 0 ? size : size * 0.4;
          push(cx + r * Math.cos(a), cy + r * Math.sin(a));
        }
        break;
      }
      case 'helix':
        for (let i = 0; i < n; i++) {
          const t = i / Math.max(1, n - 1);
          const a = 2 * Math.PI * 2 * t;
          const z = alt - 3 * t; // Descend 3m
          wps.push({ x: cx + size * Math.cos(a), y: cy + size * Math.sin(a), z });
        }
        break;
      default:
        push(cx, cy);
    }

    return wps;
  },

  // Helper: read CSS variable from document root
  _css(varName) {
    return getComputedStyle(document.documentElement).getPropertyValue(varName).trim();
  },

  _isLight() {
    return document.documentElement.getAttribute('data-theme') !== 'dark';
  },

  draw() {
    if (!this.ctx) return;
    const ctx = this.ctx;
    const w = this.canvas.width / window.devicePixelRatio;
    const h = this.canvas.height / window.devicePixelRatio;
    const light = this._isLight();

    // Clear
    ctx.fillStyle = this._css('--bg-secondary');
    ctx.fillRect(0, 0, w, h);

    // Grid
    this.drawGrid(ctx, w, h);

    // Axis labels
    ctx.fillStyle = this._css('--text-muted');
    ctx.font = '10px Inter';
    ctx.fillText('N (x)', this.offsetX - 10, 14);
    ctx.fillText('E (y)', w - 24, this.offsetY + 14);

    // Planned trajectory path
    if (this.waypoints.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = this._css('--accent');
      ctx.lineWidth = 2;
      ctx.setLineDash([]);
      const first = this.nedToCanvas(this.waypoints[0].x, this.waypoints[0].y);
      ctx.moveTo(first.cx, first.cy);
      for (let i = 1; i < this.waypoints.length; i++) {
        const p = this.nedToCanvas(this.waypoints[i].x, this.waypoints[i].y);
        ctx.lineTo(p.cx, p.cy);
      }
      ctx.stroke();

      // Waypoint dots
      for (let i = 0; i < this.waypoints.length; i++) {
        const p = this.nedToCanvas(this.waypoints[i].x, this.waypoints[i].y);
        ctx.beginPath();
        ctx.arc(p.cx, p.cy, 4, 0, 2 * Math.PI);
        ctx.fillStyle = this._css('--accent');
        ctx.fill();

        // Waypoint number
        ctx.fillStyle = this._css('--text-secondary');
        ctx.font = '9px JetBrains Mono';
        ctx.fillText(i.toString(), p.cx + 6, p.cy - 6);
      }
    }

    // Custom draw points
    if (this.currentShape === 'custom' && this.customPoints.length > 0) {
      ctx.beginPath();
      ctx.strokeStyle = this._css('--purple');
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 3]);
      const fp = this.customPoints[0];
      ctx.moveTo(fp.cx, fp.cy);
      for (let i = 1; i < this.customPoints.length; i++) {
        ctx.lineTo(this.customPoints[i].cx, this.customPoints[i].cy);
      }
      ctx.stroke();
      ctx.setLineDash([]);

      for (let i = 0; i < this.customPoints.length; i++) {
        const p = this.customPoints[i];
        ctx.beginPath();
        ctx.arc(p.cx, p.cy, 5, 0, 2 * Math.PI);
        ctx.fillStyle = i === this.dragIndex ? this._css('--yellow') : this._css('--purple');
        ctx.fill();
        ctx.strokeStyle = light ? '#333' : '#fff';
        ctx.lineWidth = 1;
        ctx.stroke();
      }
    }

    // Actual drone position
    if (this.dronePos) {
      const dp = this.nedToCanvas(this.dronePos.x, this.dronePos.y);
      ctx.save();
      ctx.translate(dp.cx, dp.cy);
      ctx.beginPath();
      ctx.moveTo(0, -8);
      ctx.lineTo(-6, 6);
      ctx.lineTo(6, 6);
      ctx.closePath();
      ctx.fillStyle = this._css('--orange');
      ctx.fill();
      ctx.shadowColor = this._css('--orange');
      ctx.shadowBlur = light ? 4 : 12;
      ctx.fill();
      ctx.restore();
    }

    // Origin cross
    ctx.strokeStyle = light ? 'rgba(0,0,0,0.15)' : '#2a3450';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(this.offsetX, 0);
    ctx.lineTo(this.offsetX, h);
    ctx.moveTo(0, this.offsetY);
    ctx.lineTo(w, this.offsetY);
    ctx.stroke();
  },

  drawGrid(ctx, w, h) {
    const light = this._isLight();
    ctx.strokeStyle = light ? 'rgba(0,0,0,0.07)' : 'rgba(255,255,255,0.03)';
    ctx.lineWidth = 1;
    const step = this.scale; // 1 meter grid

    for (let x = this.offsetX % step; x < w; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let y = this.offsetY % step; y < h; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }
  },

  updateDronePosition(data) {
    if (data.position) {
      this.dronePos = { x: data.position.x, y: data.position.y };
      this.draw();
    }
  },

  // ── Mouse handlers for free-draw mode ──────────────────────────────
  onMouseDown(e) {
    if (this.currentShape !== 'custom') return;
    const rect = this.canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;

    // Check if clicking on existing point
    for (let i = 0; i < this.customPoints.length; i++) {
      const p = this.customPoints[i];
      if (Math.hypot(cx - p.cx, cy - p.cy) < 10) {
        this.isDragging = true;
        this.dragIndex = i;
        return;
      }
    }

    // Add new point
    this.customPoints.push({ cx, cy });
    this._updateCustomWaypoints();
    this.draw();
  },

  onMouseMove(e) {
    if (!this.isDragging || this.dragIndex < 0) return;
    const rect = this.canvas.getBoundingClientRect();
    this.customPoints[this.dragIndex].cx = e.clientX - rect.left;
    this.customPoints[this.dragIndex].cy = e.clientY - rect.top;
    this._updateCustomWaypoints();
    this.draw();
  },

  onMouseUp() {
    this.isDragging = false;
    this.dragIndex = -1;
  },

  onDoubleClick(e) {
    if (this.currentShape !== 'custom') return;
    const rect = this.canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;

    for (let i = 0; i < this.customPoints.length; i++) {
      if (Math.hypot(cx - this.customPoints[i].cx, cy - this.customPoints[i].cy) < 10) {
        this.customPoints.splice(i, 1);
        this._updateCustomWaypoints();
        this.draw();
        return;
      }
    }
  },

  _updateCustomWaypoints() {
    const alt = -(parseFloat(document.getElementById('traj-alt').value) || 3);
    this.waypoints = this.customPoints.map(p => {
      const ned = this.canvasToNed(p.cx, p.cy);
      return { x: ned.x, y: ned.y, z: alt };
    });
  },

  getWaypointsFlat() {
    const flat = [];
    for (const wp of this.waypoints) {
      flat.push(wp.x, wp.y, wp.z);
    }
    return flat;
  },

  uploadTrajectory() {
    if (this.waypoints.length < 2) {
      showToast('Need at least 2 waypoints!', 'warning');
      return;
    }

    const flat = this.getWaypointsFlat();

    // Option 1: Send via generate_trajectory command (server-side generation)
    // Option 2: Send flat waypoints directly
    App.send({
      cmd: 'set_waypoints',
      waypoints: flat,
    });

    showToast(`Uploaded ${this.waypoints.length} waypoints to drone`, 'success');
  }
};

/* ── Init ──────────────────────────────────────────────────────────────── */
window.addEventListener('DOMContentLoaded', () => {
  TrajectoryCanvas.init();

  // Shape buttons
  document.querySelectorAll('.shape-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.shape-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      TrajectoryCanvas.currentShape = btn.dataset.shape;
      TrajectoryCanvas.customPoints = [];
      TrajectoryCanvas.generateShape();
    });
  });

  // Trajectory parameter sliders
  const trajSliders = ['traj-points', 'traj-size', 'traj-alt', 'traj-cx', 'traj-cy'];
  for (const id of trajSliders) {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('input', () => {
        document.getElementById(id + '-val').textContent = el.value;
        TrajectoryCanvas.generateShape();
      });
    }
  }

  // Upload button
  document.getElementById('btn-upload-traj')?.addEventListener('click', () => {
    TrajectoryCanvas.uploadTrajectory();
  });
});
