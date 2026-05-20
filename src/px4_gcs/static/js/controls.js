/* ══════════════════════════════════════════════════════════════════════════
   controls.js — UI Controls: buttons, sliders, controller switching
   ══════════════════════════════════════════════════════════════════════════ */

window.addEventListener('DOMContentLoaded', () => {

  // ══════════════════════════════════════════════════════════════════════
  //  Flight Command Buttons
  // ══════════════════════════════════════════════════════════════════════

  document.getElementById('btn-arm')?.addEventListener('click', () => {
    if (confirm('ARM the drone?')) {
      App.send({ cmd: 'arm' });
      showToast('ARM command sent', 'warning');
    }
  });

  document.getElementById('btn-disarm')?.addEventListener('click', () => {
    if (confirm('DISARM the drone?')) {
      App.send({ cmd: 'disarm' });
      showToast('DISARM command sent', 'success');
    }
  });

  document.getElementById('btn-offboard')?.addEventListener('click', () => {
    App.send({ cmd: 'offboard' });
    showToast('OFFBOARD mode sent', 'info');
  });

  document.getElementById('btn-land')?.addEventListener('click', () => {
    App.send({ cmd: 'land' });
    showToast('LAND command sent', 'info');
  });

  document.getElementById('btn-rtl')?.addEventListener('click', () => {
    App.send({ cmd: 'rtl' });
    showToast('Return to Launch sent', 'info');
  });

  document.getElementById('btn-reset-traj')?.addEventListener('click', () => {
    App.send({ cmd: 'reset_trajectory' });
    showToast('Trajectory reset', 'info');
  });

  // ══════════════════════════════════════════════════════════════════════
  //  Process Manager Buttons
  // ══════════════════════════════════════════════════════════════════════

  document.getElementById('btn-launch-all')?.addEventListener('click', () => {
    App.send({ cmd: 'launch_all' });
    showToast('Launching all processes...', 'info');
  });

  document.getElementById('btn-stop-all')?.addEventListener('click', () => {
    if (confirm('Stop ALL processes?')) {
      App.send({ cmd: 'stop_all' });
      showToast('Stopping all processes...', 'warning');
    }
  });

  // ══════════════════════════════════════════════════════════════════════
  //  Controller Switching
  // ══════════════════════════════════════════════════════════════════════

  const controllerToggle = document.getElementById('controller-toggle');
  const labelPid = document.getElementById('label-pid');
  const labelLqr = document.getElementById('label-lqr');
  const indicator = document.getElementById('controller-indicator');
  const pillCtrl = document.getElementById('pill-controller');

  function updateControllerUI(isLqr) {
    if (isLqr) {
      App.controllerMode = 'lqr';
      labelPid.classList.remove('active-label');
      labelLqr.classList.add('active-label');
      indicator.classList.add('lqr-active');
      indicator.querySelector('.indicator-text').textContent = 'LQR Controller Active';
      pillCtrl.querySelector('.pill-label').textContent = 'LQR Active';
      pillCtrl.className = 'pill warning';
      // Show LQR tuning, hide PID
      document.getElementById('pid-tuning').style.display = 'none';
      document.getElementById('lqr-tuning').style.display = 'block';
    } else {
      App.controllerMode = 'pid';
      labelPid.classList.add('active-label');
      labelLqr.classList.remove('active-label');
      indicator.classList.remove('lqr-active');
      indicator.querySelector('.indicator-text').textContent = 'PID Controller Active';
      pillCtrl.querySelector('.pill-label').textContent = 'PID Active';
      pillCtrl.className = 'pill active';
      // Show PID tuning, hide LQR
      document.getElementById('pid-tuning').style.display = 'block';
      document.getElementById('lqr-tuning').style.display = 'none';
    }
  }

  controllerToggle?.addEventListener('change', (e) => {
    const isLqr = e.target.checked;
    updateControllerUI(isLqr);
    App.send({ cmd: 'switch_controller', mode: isLqr ? 'lqr' : 'pid' });
    showToast(`Switched to ${isLqr ? 'LQR' : 'PID'} controller`, 'success');
  });

  // Initialize PID as active
  labelPid.classList.add('active-label');

  // ══════════════════════════════════════════════════════════════════════
  //  PID Tuning Sliders
  // ══════════════════════════════════════════════════════════════════════

  const pidSliders = [
    { slider: 'slider-kp-xy', display: 'val-kp-xy' },
    { slider: 'slider-ki-xy', display: 'val-ki-xy' },
    { slider: 'slider-kd-xy', display: 'val-kd-xy' },
    { slider: 'slider-kp-z',  display: 'val-kp-z' },
    { slider: 'slider-ki-z',  display: 'val-ki-z' },
    { slider: 'slider-kd-z',  display: 'val-kd-z' },
  ];

  for (const { slider, display } of pidSliders) {
    const el = document.getElementById(slider);
    const disp = document.getElementById(display);
    if (el && disp) {
      el.addEventListener('input', () => {
        disp.textContent = parseFloat(el.value).toFixed(2);
      });
    }
  }

  document.getElementById('btn-apply-pid')?.addEventListener('click', () => {
    const params = {
      kp_xy: parseFloat(document.getElementById('slider-kp-xy').value),
      ki_xy: parseFloat(document.getElementById('slider-ki-xy').value),
      kd_xy: parseFloat(document.getElementById('slider-kd-xy').value),
      kp_z:  parseFloat(document.getElementById('slider-kp-z').value),
      ki_z:  parseFloat(document.getElementById('slider-ki-z').value),
      kd_z:  parseFloat(document.getElementById('slider-kd-z').value),
    };
    App.send({ cmd: 'set_pid_params', params });
    showToast('PID gains applied', 'success');
  });

  document.getElementById('btn-reset-pid')?.addEventListener('click', () => {
    const defaults = { 'slider-kp-xy': 0.6, 'slider-ki-xy': 0.02, 'slider-kd-xy': 0.3,
                        'slider-kp-z': 1.5, 'slider-ki-z': 0.1, 'slider-kd-z': 0.5 };
    for (const [id, val] of Object.entries(defaults)) {
      const el = document.getElementById(id);
      if (el) { el.value = val; el.dispatchEvent(new Event('input')); }
    }
    showToast('PID gains reset to default', 'info');
  });

  // ══════════════════════════════════════════════════════════════════════
  //  LQR Tuning Sliders
  // ══════════════════════════════════════════════════════════════════════

  const lqrSliders = [
    { slider: 'slider-q-pos-xy', display: 'val-q-pos-xy' },
    { slider: 'slider-q-pos-z',  display: 'val-q-pos-z' },
    { slider: 'slider-q-vel-xy', display: 'val-q-vel-xy' },
    { slider: 'slider-q-vel-z',  display: 'val-q-vel-z' },
    { slider: 'slider-r-xy',     display: 'val-r-xy' },
    { slider: 'slider-r-z',      display: 'val-r-z' },
  ];

  for (const { slider, display } of lqrSliders) {
    const el = document.getElementById(slider);
    const disp = document.getElementById(display);
    if (el && disp) {
      el.addEventListener('input', () => {
        disp.textContent = parseFloat(el.value).toFixed(1);
      });
    }
  }

  document.getElementById('btn-apply-lqr')?.addEventListener('click', () => {
    const params = {
      q_pos_xy: parseFloat(document.getElementById('slider-q-pos-xy').value),
      q_pos_z:  parseFloat(document.getElementById('slider-q-pos-z').value),
      q_vel_xy: parseFloat(document.getElementById('slider-q-vel-xy').value),
      q_vel_z:  parseFloat(document.getElementById('slider-q-vel-z').value),
      r_xy:     parseFloat(document.getElementById('slider-r-xy').value),
      r_z:      parseFloat(document.getElementById('slider-r-z').value),
    };
    App.send({ cmd: 'set_lqr_params', params });
    showToast('LQR parameters applied — recomputing gain K', 'success');
  });

  document.getElementById('btn-reset-lqr')?.addEventListener('click', () => {
    const defaults = { 'slider-q-pos-xy': 4.0, 'slider-q-pos-z': 6.0,
                        'slider-q-vel-xy': 2.0, 'slider-q-vel-z': 3.0,
                        'slider-r-xy': 1.0, 'slider-r-z': 1.0 };
    for (const [id, val] of Object.entries(defaults)) {
      const el = document.getElementById(id);
      if (el) { el.value = val; el.dispatchEvent(new Event('input')); }
    }
    showToast('LQR params reset to default', 'info');
  });

  // ══════════════════════════════════════════════════════════════════════
  //  Data Export & Clear
  // ══════════════════════════════════════════════════════════════════════

  document.getElementById('btn-export-csv')?.addEventListener('click', () => {
    App.exportCSV();
  });

  document.getElementById('btn-clear-data')?.addEventListener('click', () => {
    App.clearHistory();
    if (typeof Charts !== 'undefined') {
      Charts.initialized = false;
      Charts.init();
    }
  });
});
