/**
 * app.js - Mobile-First (iPhone BLE + Laptop USB) Dual-IMU 3D Knee & Joint Kinematics Suite
 * =========================================================================================
 * Interfaces with the Seeed Studio XIAO 1.14" IPS Display (nRF52840 Plus) and
 * 2x Adafruit ICM-20948 9-DOF IMUs (0x69 Lateral Thigh & 0x68 Lateral Lower Leg) via:
 *   - Web Bluetooth API (BLE Nordic UART Service - Direct iPhone Bluefy & Desktop BLE)
 *   - Web Serial API (USB-C CDC @ 115200 baud)
 *
 * Features:
 *   - Automatic Standing Neutral Calibration (0°, 0°, 0°) on Start Recording
 *   - Dual Packet Parser: Full 24-field USB ('D,...') & Compact 13-field iPhone BLE ('B,...')
 *   - Native iOS Share Sheet (AirDrop / Save to Files) + Standard CSV Download
 *
 * Developed for LTU Experimental Biomechanics Laboratory (EBL)
 */

document.addEventListener('DOMContentLoaded', () => {
  // --- Connection & Mode State ---
  let serialPort = null;
  let serialReader = null;
  let isConnected = false;
  let connectionType = null; // 'usb' | 'ble' | null
  let isDemoMode = false;
  let isStreaming = true;
  let isRecording = false;
  let isChartPaused = false;
  let isCalibrated = false;
  let xiaoScreenIndex = 0;

  // BLE Nordic UART Service UUIDs
  const NUS_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
  const NUS_RX_CHAR_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // Write to XIAO
  const NUS_TX_CHAR_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // Notify from XIAO
  let bleDevice = null;
  let bleTxChar = null;
  let bleRxChar = null;
  let bleLineBuffer = '';
  const utf8Decoder = new TextDecoder('utf-8');
  const utf8Encoder = new TextEncoder();

  // Anatomical Joint Presets (Default: 'knee')
  const JOINT_PRESETS = {
    knee: {
      key: 'knee',
      name: 'KNEE',
      segments: 'Thigh S1 [0x69] → Lower Leg S2 [0x68]',
      p1Short: 'Flex / Ext',
      p2Short: 'Varus / Valgus',
      p3Short: 'Int / Ext Rot',
      p1Full: 'Knee Flexion (+) / Extension (-)',
      p2Full: 'Varus (+) / Valgus (-) Ab/Add',
      p3Full: 'Tibial Int (+) / Ext (-) Rotation',
      lcdP1: 'Flx/Ext',
      lcdP2: 'Var/Val',
      lcdP3: 'Int/Ext'
    },
    wrist: {
      key: 'wrist',
      name: 'WRIST',
      segments: 'Forearm S1 [0x69] → Hand S2 [0x68]',
      p1Short: 'Flex / Ext',
      p2Short: 'Rad / Uln Dev',
      p3Short: 'Pro / Sup',
      p1Full: 'Wrist Flexion (+) / Extension (-)',
      p2Full: 'Radial (+) / Ulnar (-) Deviation',
      p3Full: 'Forearm Pronation (+) / Supination (-)',
      lcdP1: 'Flx/Ext',
      lcdP2: 'Rad/Uln',
      lcdP3: 'Pro/Sup'
    },
    elbow: {
      key: 'elbow',
      name: 'ELBOW',
      segments: 'Upper Arm S1 [0x69] → Forearm S2 [0x68]',
      p1Short: 'Flex / Ext',
      p2Short: 'Valgus / Varus',
      p3Short: 'Pro / Sup',
      p1Full: 'Elbow Flexion (+) / Extension (-)',
      p2Full: 'Carrying Angle Valgus / Varus',
      p3Full: 'Forearm Pronation / Supination',
      lcdP1: 'Flx/Ext',
      lcdP2: 'Val/Var',
      lcdP3: 'Pro/Sup'
    },
    ankle: {
      key: 'ankle',
      name: 'ANKLE',
      segments: 'Lower Leg S1 [0x69] → Foot S2 [0x68]',
      p1Short: 'Dorsi / Plantar',
      p2Short: 'Inv / Eversion',
      p3Short: 'Int / Ext Rot',
      p1Full: 'Dorsiflexion (+) / Plantarflexion (-)',
      p2Full: 'Inversion (+) / Eversion (-)',
      p3Full: 'Abduction (+) / Adduction (-)',
      lcdP1: 'Dor/Pln',
      lcdP2: 'Inv/Evr',
      lcdP3: 'Int/Ext'
    }
  };
  let currentJoint = 'knee';

  // Client-side Neutral Calibration Offsets (Zeroed automatically on Start Recording or Zero button)
  let rawAnglesLatest = { fe: 0, ru: 0, ps: 0 };
  let neutralOffset = { fe: 0, ru: 0, ps: 0 };

  // Smooth Derivative State for Synthesizing Gyro Rates from Compact BLE Packets ('B,...')
  let bleSynthPrev = { t: 0, fe: 0, ru: 0, ps: 0, gx2: 0, gy2: 0, gz2: 0 };

  // Sampling Rate Counter
  let sampleCountHz = 0;
  let currentHz = 0;

  // Recording State
  let recordedData = [];
  let recordStartTime = 0;
  let recordTimerInterval = null;
  let toastTimeout = null;
  let memDumpDebounceTimer = null;

  // Recorded Trials History & 3D Replay State
  const TRIALS_STORAGE_KEY = 'ltu_ebl_xiao_trials_v1';
  let savedTrials = [];
  let selectedTrialId = null;
  let isReplayingTrial = false;
  let isTrialPlaying = false;
  let trialPlaybackSpeed = 1.0;
  let trialScrubIndex = 0;
  let trialPlayRafId = null;
  let trialPlayLastMs = 0;
  let trialPlayFloatIdx = 0;

  // Real-Time Biomechanical Range of Motion (ROM) & Cycle Analytics
  const romStats = {
    fe: { min: Infinity, max: -Infinity, sum: 0, count: 0, peakVel: 0 },
    ru: { min: Infinity, max: -Infinity, sum: 0, count: 0, peakVel: 0 },
    ps: { min: Infinity, max: -Infinity, sum: 0, count: 0, peakVel: 0 },
    femur: { min: Infinity, max: -Infinity, sum: 0, count: 0, peakVel: 0 },
    reps: 0,
    inFlexCycle: false
  };
  let prevAngles = { fe: 0, ru: 0, ps: 0, femur: 0, t: 0 };

  // Real-Time Chart Buffers
  let activeChartTab = 'angles'; // 'angles' | 'gyro' | 'accel'
  let timeWindowSec = 10;
  const maxPoints = 400;
  const buf = {
    t: [],
    fe: [], ru: [], ps: [], femur: [], tot: [],
    gx1: [], gy1: [], gz1: [], gx2: [], gy2: [], gz2: [], jVel: [],
    ax1: [], ay1: [], az1: [], ax2: [], ay2: [], az2: []
  };

  // --- Initialize 3D Articulated Dual-Segment Model ---
  const joint3D = new DualIMUJoint3D('canvas-container');

  // --- DOM Elements ---
  const iosBleBanner = document.getElementById('ios-ble-banner');
  const btnDismissIosBanner = document.getElementById('btn-dismiss-ios-banner');
  const statusToast = document.getElementById('status-toast');
  const statusToastText = document.getElementById('status-toast-text');

  const jointSelect = document.getElementById('joint-preset-select');
  const statusPill = document.getElementById('connection-status');
  const statusText = document.getElementById('connection-text');
  const calBadge = document.getElementById('cal-status-badge');

  const btnConnectBle = document.getElementById('btn-connect-ble');
  const btnConnectUsb = document.getElementById('btn-connect-usb');
  const btnDemo = document.getElementById('btn-demo-mode') || document.getElementById('btn-demo');
  const btnRecord = document.getElementById('btn-record');
  const btnRecordLabel = document.getElementById('btn-record-label');
  const btnCalibrate = document.getElementById('btn-calibrate-neutral') || document.getElementById('btn-calibrate');
  const btnResetSys = document.getElementById('btn-reset-sys');
  const btnExport = document.getElementById('btn-export-csv') || document.getElementById('btn-export');
  const btnExportReport = document.getElementById('btn-export-report');
  const btnExportPng = document.getElementById('btn-export-png');
  const btnToggleStream = document.getElementById('btn-toggle-stream');
  const btnDumpMem = document.getElementById('btn-dump-mem');
  const btnOpenHelp = document.getElementById('btn-open-help');
  const btnCloseHelp = document.getElementById('btn-close-help');
  const modalHelp = document.getElementById('modal-help');

  // 3D HUD & Virtual XIAO Buttons
  const btnHudCal = document.getElementById('btn-hud-cal');
  const btnToggleAxes = document.getElementById('btn-toggle-axes');
  const btnResetCam = document.getElementById('btn-reset-cam');
  const hudReplayPill = document.getElementById('hud-replay-pill');
  const btnHwScreen = document.getElementById('btn-hw-screen');
  const btnHwCal = document.getElementById('btn-hw-cal');
  const btnHwStream = document.getElementById('btn-hw-stream');
  const btnHwRec = document.getElementById('btn-hw-rec');

  // Live Readouts
  const valTime = document.getElementById('val-time');
  const valFe = document.getElementById('val-fe');
  const valRu = document.getElementById('val-ru');
  const valPs = document.getElementById('val-ps');
  const valFemurPitch = document.getElementById('val-femur-pitch');
  const valTotAngle = document.getElementById('val-tot-angle');
  const valAngVel = document.getElementById('val-ang-vel');
  const valRepsTop = document.getElementById('val-reps-top');
  const valS1Amag = document.getElementById('val-s1-amag');
  const valS2Amag = document.getElementById('val-s2-amag');
  const valBatt = document.getElementById('val-batt');
  const valBattPct = document.getElementById('val-batt-pct');
  const valHz = document.getElementById('val-hz');

  // Trial Review & Playback DOM Elements
  const trialCountBadge = document.getElementById('trial-count-badge');
  const trialSelect = document.getElementById('trial-select');
  const btnTrialDelete = document.getElementById('btn-trial-delete');
  const btnTrialPlay = document.getElementById('btn-trial-play');
  const btnTrialStop = document.getElementById('btn-trial-stop');
  const trialScrubTime = document.getElementById('trial-scrub-time');
  const trialScrubber = document.getElementById('trial-scrubber');
  const scrubValFe = document.getElementById('scrub-val-fe');
  const scrubValRu = document.getElementById('scrub-val-ru');
  const scrubValPs = document.getElementById('scrub-val-ps');
  const scrubValFemur = document.getElementById('scrub-val-femur');
  const scrubValVel = document.getElementById('scrub-val-vel');
  const trialReviewCanvas = document.getElementById('trial-review-chart');
  const tstatFeRom = document.getElementById('tstat-fe-rom');
  const tstatFeMinmax = document.getElementById('tstat-fe-minmax');
  const tstatRuRom = document.getElementById('tstat-ru-rom');
  const tstatRuMinmax = document.getElementById('tstat-ru-minmax');
  const tstatPsRom = document.getElementById('tstat-ps-rom');
  const tstatPsMinmax = document.getElementById('tstat-ps-minmax');
  const tstatFemurRom = document.getElementById('tstat-femur-rom');
  const tstatFemurMinmax = document.getElementById('tstat-femur-minmax');
  const btnTrialExportReport = document.getElementById('btn-trial-export-report');
  const btnTrialExportPng = document.getElementById('btn-trial-export-png');
  const btnTrialExportCsv = document.getElementById('btn-trial-export-csv');

  // XIAO 1.14" Landscape LCD Mirror DOM
  const lcdHdrLeft = document.getElementById('lcd-hdr-left');
  const lcdHdrRight = document.getElementById('lcd-hdr-right');
  const lcdLine1 = document.getElementById('lcd-line-1');
  const lcdLine2 = document.getElementById('lcd-line-2');
  const lcdLine3 = document.getElementById('lcd-line-3');
  const lcdFtrLeft = document.getElementById('lcd-ftr-left');
  const lcdFtrRight = document.getElementById('lcd-ftr-right');

  // Horizon Gauges
  const hudFeVal = document.getElementById('hud-fe-val');
  const hudRuVal = document.getElementById('hud-ru-val');
  const hudPsVal = document.getElementById('hud-ps-val');
  const barFeFill = document.getElementById('bar-fe-fill');
  const barRuFill = document.getElementById('bar-ru-fill');
  const barPsFill = document.getElementById('bar-ps-fill');

  // Terminal & Recording DOM
  const terminalOut = document.getElementById('terminal-output');
  const autoscrollCheck = document.getElementById('autoscroll-toggle');
  const btnClearTerminal = document.getElementById('btn-clear-terminal');
  const cmdInput = document.getElementById('cmd-input');
  const btnSendCmd = document.getElementById('btn-send-cmd');
  const recordCountBadge = document.getElementById('record-count-badge');
  const recordTimeBadge = document.getElementById('record-time-badge');
  const recordTableBody = document.getElementById('record-table-body');
  const btnClearRecord = document.getElementById('btn-clear-record');
  const btnResetRom = document.getElementById('btn-reset-rom');
  const repCounterBadge = document.getElementById('rep-counter-badge');

  // --- iOS Web Bluetooth Detection & Helpful Banner ---
  const isIOS = /iPhone|iPad|iPod/i.test(navigator.userAgent) ||
    (navigator.platform === 'MacIntel' && navigator.maxTouchPoints > 1);
  if (isIOS && !('bluetooth' in navigator) && iosBleBanner) {
    iosBleBanner.style.display = 'flex';
  }
  if (btnDismissIosBanner && iosBleBanner) {
    btnDismissIosBanner.addEventListener('click', () => {
      iosBleBanner.style.display = 'none';
    });
  }

  // --- Toast Helper ---
  function showStatusToast(message, durationMs = 4500) {
    if (!statusToast || !statusToastText) return;
    statusToastText.textContent = message;
    statusToast.style.display = 'flex';
    if (toastTimeout) clearTimeout(toastTimeout);
    if (durationMs > 0) {
      toastTimeout = setTimeout(() => {
        statusToast.style.display = 'none';
      }, durationMs);
    }
  }

  // --- Initialize Chart.js ---
  const chartCtx = document.getElementById('live-chart').getContext('2d');
  const chart = new Chart(chartCtx, {
    type: 'line',
    data: { labels: buf.t, datasets: [] },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      normalized: true,
      plugins: {
        legend: {
          position: 'top',
          labels: { font: { size: 11, weight: '700' }, boxWidth: 12, usePointStyle: true }
        },
        tooltip: { mode: 'index', intersect: false }
      },
      scales: {
        x: {
          title: { display: true, text: 'Elapsed Time (s)', color: '#64748b' },
          grid: { color: '#e2e8f0' },
          ticks: { maxTicksLimit: 7, color: '#64748b' }
        },
        y: {
          title: { display: true, text: 'Joint Angle (deg)', color: '#64748b' },
          grid: { color: '#e2e8f0' },
          ticks: { color: '#64748b' }
        }
      },
      elements: {
        point: { radius: 0, hitRadius: 5 },
        line: { borderWidth: 2.2, tension: 0.14 }
      }
    }
  });

  function getPreset(key) {
    const k = String(key || 'knee').toLowerCase();
    return JOINT_PRESETS[k] || JOINT_PRESETS.knee;
  }

  function updateChartDatasets() {
    const preset = getPreset(currentJoint);
    if (activeChartTab === 'angles') {
      chart.options.scales.y.title.text = `${preset.name} 3D Angles (°)`;
      chart.data.datasets = [
        { label: `${preset.p1Short} (°)`, data: buf.fe, borderColor: '#0284c7', backgroundColor: 'transparent' },
        { label: `${preset.p2Short} (°)`, data: buf.ru, borderColor: '#10b981', backgroundColor: 'transparent' },
        { label: `${preset.p3Short} (°)`, data: buf.ps, borderColor: '#f59e0b', backgroundColor: 'transparent' },
        { label: 'Femur Tilt S1 (°)', data: buf.femur, borderColor: '#ec4899', borderDash: [2, 2], backgroundColor: 'transparent' },
        { label: 'Total 3D (°)', data: buf.tot, borderColor: '#8b5cf6', borderDash: [4, 4], backgroundColor: 'transparent' }
      ];
    } else if (activeChartTab === 'gyro') {
      chart.options.scales.y.title.text = 'Angular Velocity (°/s)';
      chart.data.datasets = [
        { label: 'Joint |ω| (°/s)', data: buf.jVel, borderColor: '#8b5cf6', backgroundColor: 'transparent' },
        { label: 'S1 Thigh Gx (°/s)', data: buf.gx1, borderColor: '#0284c7', backgroundColor: 'transparent' },
        { label: 'S2 Shank Gx (°/s)', data: buf.gx2, borderColor: '#10b981', backgroundColor: 'transparent' },
        { label: 'S2 Shank Gz (°/s)', data: buf.gz2, borderColor: '#f59e0b', backgroundColor: 'transparent' }
      ];
    } else if (activeChartTab === 'accel') {
      chart.options.scales.y.title.text = 'Acceleration (m/s²)';
      chart.data.datasets = [
        { label: 'S1 (0x69) Ax', data: buf.ax1, borderColor: '#ef4444', borderDash: [3, 3], backgroundColor: 'transparent' },
        { label: 'S1 (0x69) Az', data: buf.az1, borderColor: '#3b82f6', borderDash: [3, 3], backgroundColor: 'transparent' },
        { label: 'S2 (0x68) Ax', data: buf.ax2, borderColor: '#dc2626', backgroundColor: 'transparent' },
        { label: 'S2 (0x68) Ay', data: buf.ay2, borderColor: '#10b981', backgroundColor: 'transparent' },
        { label: 'S2 (0x68) Az', data: buf.az2, borderColor: '#2563eb', backgroundColor: 'transparent' }
      ];
    }
    chart.update('none');
  }

  function clearChartBuffers() {
    Object.values(buf).forEach((arr) => { arr.length = 0; });
    chart.update('none');
  }

  // --- Anatomical Joint Preset Switcher (Defaults to Knee Joint) ---
  function applyJointPreset(presetKey, notifyHardware = true) {
    const p = getPreset(presetKey);
    currentJoint = p.key;
    if (jointSelect && jointSelect.value !== p.key) {
      jointSelect.value = p.key;
    }

    document.getElementById('lbl-3d-segments').textContent = p.segments;
    document.getElementById('lbl-metric-p1').textContent = p.p1Short;
    document.getElementById('lbl-metric-p2').textContent = p.p2Short;
    document.getElementById('lbl-metric-p3').textContent = p.p3Short;

    document.getElementById('lbl-gauge-p1').textContent = p.p1Full;
    document.getElementById('lbl-gauge-p2').textContent = p.p2Full;
    document.getElementById('lbl-gauge-p3').textContent = p.p3Full;

    document.getElementById('rom-name-p1').textContent = p.p1Short;
    document.getElementById('rom-name-p2').textContent = p.p2Short;
    document.getElementById('rom-name-p3').textContent = p.p3Short;

    if (joint3D && typeof joint3D.setJointPreset === 'function') {
      joint3D.setJointPreset(p.key);
    }

    updateChartDatasets();
    if (notifyHardware && isConnected) {
      sendDeviceCommand(`CMD:JOINT:${p.name}`);
    }
  }

  if (jointSelect) {
    jointSelect.addEventListener('change', (e) => {
      applyJointPreset(e.target.value, true);
    });
  }

  // --- Chart Tab & Window Controls ---
  document.querySelectorAll('.tab-btn[data-tab]').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.tab-btn[data-tab]').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      activeChartTab = btn.getAttribute('data-tab');
      updateChartDatasets();
    });
  });

  document.querySelectorAll('.tab-btn[data-window]').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.tab-btn[data-window]').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      timeWindowSec = parseInt(btn.getAttribute('data-window'), 10) || 10;
    });
  });

  const btnPauseChart = document.getElementById('btn-pause-chart');
  if (btnPauseChart) {
    btnPauseChart.addEventListener('click', () => {
      isChartPaused = !isChartPaused;
      btnPauseChart.textContent = isChartPaused ? 'Resume' : 'Pause';
    });
  }

  // --- Rate (Hz) Timer ---
  setInterval(() => {
    currentHz = sampleCountHz;
    sampleCountHz = 0;
    valHz.textContent = (isConnected || isDemoMode) && isStreaming ? `${currentHz} Hz` : '--';
    if (lcdFtrRight) lcdFtrRight.textContent = `${currentHz} Hz`;
  }, 1000);

  // --- Send Remote Control Command to XIAO over USB Serial or BLE ---
  async function sendDeviceCommand(cmd) {
    appendTerminalLine(`→ TX: ${cmd}`, 'cmd');
    const payload = utf8Encoder.encode(`${cmd.trim()}\n`);
    if (isConnected && connectionType === 'usb' && serialPort && serialPort.writable) {
      try {
        const writer = serialPort.writable.getWriter();
        await writer.write(payload);
        writer.releaseLock();
      } catch (err) {
        console.warn('USB write error:', err);
      }
    } else if (isConnected && connectionType === 'ble' && bleRxChar) {
      try {
        if (typeof bleRxChar.writeValueWithoutResponse === 'function') {
          await bleRxChar.writeValueWithoutResponse(payload);
        } else {
          await bleRxChar.writeValue(payload);
        }
      } catch (err) {
        try {
          await bleRxChar.writeValue(payload);
        } catch (err2) {
          console.warn('BLE write error:', err2);
        }
      }
    }
  }

  // --- Neutral Standing Pose Calibration (0°, 0°, 0°) ---
  function calibrateClientStandingZero(badgeLabel = 'Calibrated Standing (0°, 0°, 0°)') {
    neutralOffset = { ...rawAnglesLatest };
    isCalibrated = true;
    if (calBadge) {
      calBadge.textContent = badgeLabel;
      calBadge.style.background = '#f0fdf4';
      calBadge.style.color = '#16a34a';
      calBadge.style.borderColor = '#bbf7d0';
    }
    if (joint3D && typeof joint3D.tare === 'function') {
      joint3D.tare();
    }
    resetRomAnalytics();
  }

  function triggerNeutralCalibration() {
    calibrateClientStandingZero('Calibrated Standing (0°, 0°, 0°)');
    sendDeviceCommand('CMD:CAL_NEUTRAL');
    showStatusToast('✓ Calibrated Standing Neutral Pose (0.0°, 0.0°, 0.0°)');
    appendTerminalLine('✓ Standing Neutral 3D Pose & Gyro Bias Calibrated (0.0°, 0.0°, 0.0°)', 'success');
  }

  if (btnCalibrate) btnCalibrate.addEventListener('click', triggerNeutralCalibration);
  if (btnHudCal) btnHudCal.addEventListener('click', triggerNeutralCalibration);
  if (btnHwCal) btnHwCal.addEventListener('click', triggerNeutralCalibration);

  // --- Reset ROM Analytics ---
  function resetRomAnalytics() {
    ['fe', 'ru', 'ps', 'femur'].forEach((k) => {
      romStats[k].min = Infinity;
      romStats[k].max = -Infinity;
      romStats[k].sum = 0;
      romStats[k].count = 0;
      romStats[k].peakVel = 0;
      const elCur = document.getElementById(`rom-cur-${k}`);
      const elMin = document.getElementById(`rom-min-${k}`);
      const elMax = document.getElementById(`rom-max-${k}`);
      const elSpan = document.getElementById(`rom-span-${k}`);
      const elMean = document.getElementById(`rom-mean-${k}`);
      const elPkv = document.getElementById(`rom-pkv-${k}`);
      if (elCur) elCur.textContent = '0.0';
      if (elMin) elMin.textContent = '0.0';
      if (elMax) elMax.textContent = '0.0';
      if (elSpan) elSpan.textContent = '0.0';
      if (elMean) elMean.textContent = '0.0';
      if (elPkv) elPkv.textContent = '0.0';
    });
    romStats.reps = 0;
    romStats.inFlexCycle = false;
    if (repCounterBadge) repCounterBadge.textContent = 'Reps: 0';
    if (valRepsTop) valRepsTop.textContent = '0';
  }
  if (btnResetRom) btnResetRom.addEventListener('click', resetRomAnalytics);

  // --- Update ROM Table ---
  function updateRomStats(fe, ru, ps, femur, dt) {
    const vals = { fe, ru, ps, femur };
    const vels = {
      fe: dt > 0 ? Math.abs(fe - prevAngles.fe) / dt : 0,
      ru: dt > 0 ? Math.abs(ru - prevAngles.ru) / dt : 0,
      ps: dt > 0 ? Math.abs(ps - prevAngles.ps) / dt : 0,
      femur: dt > 0 ? Math.abs(femur - (prevAngles.femur || 0)) / dt : 0
    };

    ['fe', 'ru', 'ps', 'femur'].forEach((k) => {
      const v = vals[k];
      const st = romStats[k];
      if (v < st.min) st.min = v;
      if (v > st.max) st.max = v;
      st.sum += v;
      st.count += 1;
      if (vels[k] < 1800 && vels[k] > st.peakVel) {
        st.peakVel = vels[k];
      }
      const span = st.max - st.min;
      const mean = st.sum / st.count;

      const elCur = document.getElementById(`rom-cur-${k}`);
      const elMin = document.getElementById(`rom-min-${k}`);
      const elMax = document.getElementById(`rom-max-${k}`);
      const elSpan = document.getElementById(`rom-span-${k}`);
      const elMean = document.getElementById(`rom-mean-${k}`);
      const elPkv = document.getElementById(`rom-pkv-${k}`);
      if (elCur) elCur.textContent = v.toFixed(1);
      if (elMin) elMin.textContent = st.min.toFixed(1);
      if (elMax) elMax.textContent = st.max.toFixed(1);
      if (elSpan) elSpan.textContent = span.toFixed(1);
      if (elMean) elMean.textContent = mean.toFixed(1);
      if (elPkv) elPkv.textContent = st.peakVel.toFixed(1);
    });

    // Hysteresis repetition detector on primary Flexion/Extension axis
    if (!romStats.inFlexCycle && Math.abs(fe) > 22.0) {
      romStats.inFlexCycle = true;
    } else if (romStats.inFlexCycle && Math.abs(fe) < 7.0) {
      romStats.inFlexCycle = false;
      romStats.reps += 1;
      if (repCounterBadge) repCounterBadge.textContent = `Reps: ${romStats.reps}`;
      if (valRepsTop) valRepsTop.textContent = String(romStats.reps);
    }
  }

  // --- Update Live XIAO 1.14" Landscape LCD Mirror ---
  function updateXiaoLcdMirror(t, s1, s2, fe, ru, ps, vbat, battPct) {
    const p = getPreset(currentJoint);
    const calTag = isCalibrated ? 'CAL' : 'RAW';
    lcdHdrRight.textContent = `BAT:${battPct}%`;
    lcdFtrLeft.textContent = `STRM:${isStreaming ? 'ON' : 'OFF'} | ${isRecording ? 'REC' : 'IDLE'}`;

    if (xiaoScreenIndex === 0) {
      lcdHdrLeft.textContent = `1/4 ${p.name} 3D [${calTag}]`;
      lcdLine1.textContent = `${p.lcdP1}: ${(fe >= 0 ? '+' : '') + fe.toFixed(1)} deg`;
      lcdLine2.textContent = `${p.lcdP2}: ${(ru >= 0 ? '+' : '') + ru.toFixed(1)} deg`;
      lcdLine3.textContent = `${p.lcdP3}: ${(ps >= 0 ? '+' : '') + ps.toFixed(1)} deg`;
    } else if (xiaoScreenIndex === 1) {
      lcdHdrLeft.textContent = '2/4 S1(0x69) vs S2(0x68)';
      lcdLine1.textContent = `S1A:${s1.ax.toFixed(1)} ${s1.ay.toFixed(1)} ${s1.az.toFixed(1)}`;
      lcdLine2.textContent = `S2A:${s2.ax.toFixed(1)} ${s2.ay.toFixed(1)} ${s2.az.toFixed(1)}`;
      lcdLine3.textContent = `G12:${s1.gx.toFixed(0)}/${s2.gx.toFixed(0)} dps`;
    } else if (xiaoScreenIndex === 2) {
      lcdHdrLeft.textContent = '3/4 LOCAL MEMORY & REC';
      lcdLine1.textContent = `REC: ${isRecording ? 'ACTIVE' : 'READY'}`;
      lcdLine2.textContent = `SAMPLES: ${recordedData.length}`;
      lcdLine3.textContent = `TIME: ${t.toFixed(1)} s`;
    } else {
      lcdHdrLeft.textContent = '4/4 POWER & HEALTH';
      lcdLine1.textContent = `BAT: ${vbat.toFixed(2)}V (${battPct}%)`;
      lcdLine2.textContent = `LINK: ${connectionType ? connectionType.toUpperCase() : 'DEMO'}`;
      lcdLine3.textContent = `RATE: ${currentHz} Hz`;
    }
  }

  // --- Process Incoming Dual-IMU Telemetry Frame ---
  function handleTelemetryFrame(frame) {
    if (!isStreaming || isReplayingTrial) return;
    sampleCountHz++;

    const { t, s1, s2, rawFe, rawRu, rawPs, vbat } = frame;
    rawAnglesLatest = { fe: rawFe, ru: rawRu, ps: rawPs };

    // Apply Neutral Standing Calibration Offset
    const fe = rawFe - neutralOffset.fe;
    const ru = rawRu - neutralOffset.ru;
    const ps = rawPs - neutralOffset.ps;
    const totAngle = Math.hypot(fe, ru, ps);

    // Compute Segment & Relative Angular Velocity
    const dt = prevAngles.t > 0 && t > prevAngles.t ? (t - prevAngles.t) : 0.015;
    const relGx = s2.gx - s1.gx;
    const relGy = s2.gy - s1.gy;
    const relGz = s2.gz - s1.gz;
    const jointVel = Math.hypot(relGx, relGy, relGz);

    const a1Mag = Math.hypot(s1.ax, s1.ay, s1.az);
    const a2Mag = Math.hypot(s2.ax, s2.ay, s2.az);
    const g1Mag = Math.hypot(s1.gx, s1.gy, s1.gz);
    const g2Mag = Math.hypot(s2.gx, s2.gy, s2.gz);
    const battPct = Math.max(0, Math.min(100, Math.round(((vbat - 3.3) / (4.18 - 3.3)) * 100)));

    // 1. Update 3D Anatomical Joint Model & Extract Proximal Femur Sagittal Inclination (S1 0x69)
    joint3D.updateKinematics(s1, s2, fe, ru, ps, battPct, getPreset(currentJoint).name, isCalibrated);
    const femurPitch = (joint3D && typeof joint3D.currentFemurPitchDeg === 'number' && Number.isFinite(joint3D.currentFemurPitchDeg))
      ? joint3D.currentFemurPitchDeg
      : 0;

    // 2. Update Live Angle Cards & Top Metrics
    valTime.textContent = t.toFixed(2);
    valFe.textContent = fe.toFixed(1);
    valRu.textContent = ru.toFixed(1);
    valPs.textContent = ps.toFixed(1);
    if (valFemurPitch) valFemurPitch.textContent = femurPitch.toFixed(1);
    valTotAngle.textContent = totAngle.toFixed(1);
    valAngVel.textContent = jointVel.toFixed(1);
    valS1Amag.textContent = a1Mag.toFixed(2);
    valS2Amag.textContent = a2Mag.toFixed(2);
    valBatt.textContent = vbat.toFixed(2);
    valBattPct.textContent = battPct;

    // 3. Update Horizon Gauges
    hudFeVal.textContent = `${fe >= 0 ? '+' : ''}${fe.toFixed(1)}°`;
    hudRuVal.textContent = `${ru >= 0 ? '+' : ''}${ru.toFixed(1)}°`;
    hudPsVal.textContent = `${ps >= 0 ? '+' : ''}${ps.toFixed(1)}°`;
    // Flexion gauge spans -20°..+140° for knee or -90°..+90° general
    const fePct = currentJoint === 'knee'
      ? ((fe + 15) / 155) * 100
      : ((fe + 90) / 180) * 100;
    barFeFill.style.width = `${Math.max(2, Math.min(100, fePct))}%`;
    barRuFill.style.width = `${Math.max(2, Math.min(100, ((ru + 45) / 90) * 100))}%`;
    barPsFill.style.width = `${Math.max(2, Math.min(100, ((ps + 45) / 90) * 100))}%`;

    // 4. Update XIAO 1.14" Landscape LCD Mirror
    updateXiaoLcdMirror(t, s1, s2, fe, ru, ps, vbat, battPct);

    // 5. Update ROM & Cycle Analytics
    updateRomStats(fe, ru, ps, femurPitch, dt);
    prevAngles = { fe, ru, ps, femur: femurPitch, t };

    // 6. Push to Real-Time Chart Buffers
    if (!isChartPaused) {
      buf.t.push(t.toFixed(2));
      buf.fe.push(fe);
      buf.ru.push(ru);
      buf.ps.push(ps);
      buf.femur.push(femurPitch);
      buf.tot.push(totAngle);

      buf.jVel.push(jointVel);
      buf.gx1.push(s1.gx); buf.gy1.push(s1.gy); buf.gz1.push(s1.gz);
      buf.gx2.push(s2.gx); buf.gy2.push(s2.gy); buf.gz2.push(s2.gz);

      buf.ax1.push(s1.ax); buf.ay1.push(s1.ay); buf.az1.push(s1.az);
      buf.ax2.push(s2.ax); buf.ay2.push(s2.ay); buf.az2.push(s2.az);

      const maxPts = Math.max(60, Math.min(maxPoints, (currentHz || 50) * timeWindowSec));
      while (buf.t.length > maxPts) {
        Object.values(buf).forEach((arr) => arr.shift());
      }
      chart.update('none');
    }

    // 7. Record Sample if Active
    if (isRecording) {
      const recRow = {
        iso: new Date().toISOString(),
        t: Number(t.toFixed(3)),
        joint: getPreset(currentJoint).name,
        fe: Number(fe.toFixed(2)),
        ru: Number(ru.toFixed(2)),
        ps: Number(ps.toFixed(2)),
        femur: Number(femurPitch.toFixed(2)),
        tot: Number(totAngle.toFixed(2)),
        jVel: Number(jointVel.toFixed(2)),
        ax1: Number(s1.ax.toFixed(3)), ay1: Number(s1.ay.toFixed(3)), az1: Number(s1.az.toFixed(3)),
        gx1: Number(s1.gx.toFixed(2)), gy1: Number(s1.gy.toFixed(2)), gz1: Number(s1.gz.toFixed(2)),
        mx1: Number((s1.mx || 0).toFixed(1)), my1: Number((s1.my || 0).toFixed(1)), mz1: Number((s1.mz || 0).toFixed(1)),
        ax2: Number(s2.ax.toFixed(3)), ay2: Number(s2.ay.toFixed(3)), az2: Number(s2.az.toFixed(3)),
        gx2: Number(s2.gx.toFixed(2)), gy2: Number(s2.gy.toFixed(2)), gz2: Number(s2.gz.toFixed(2)),
        mx2: Number((s2.mx || 0).toFixed(1)), my2: Number((s2.my || 0).toFixed(1)), mz2: Number((s2.mz || 0).toFixed(1)),
        a1Mag: Number(a1Mag.toFixed(2)),
        a2Mag: Number(a2Mag.toFixed(2)),
        g1Mag: Number(g1Mag.toFixed(1)),
        g2Mag: Number(g2Mag.toFixed(1)),
        vbat: Number(vbat.toFixed(2))
      };
      recordedData.push(recRow);
      recordCountBadge.textContent = `${recordedData.length} samples`;
      btnExport.disabled = false;
      appendRecordTableRow(recRow);
    }
  }

  // --- Parse Incoming USB ('D,...') and High-Speed iPhone BLE ('B,...') Lines ---
  function parseIncomingLine(line) {
    line = line.trim();
    if (!line) return;

    // 1. Handle Board Status / Button Events
    if (line.startsWith('#')) {
      appendTerminalLine(line, 'info');
      if (line.includes('EVENT:CAL_DONE') || line.includes('CAL_NEUTRAL')) {
        isCalibrated = true;
        neutralOffset = { fe: 0, ru: 0, ps: 0 };
        if (calBadge) {
          calBadge.textContent = 'Calibrated Standing (0°, 0°, 0°)';
          calBadge.style.background = '#f0fdf4';
          calBadge.style.color = '#16a34a';
          calBadge.style.borderColor = '#bbf7d0';
        }
      } else if (line.includes('EVENT:REC_ON') && !isRecording) {
        setRecordingState(true, false);
      } else if (line.includes('EVENT:REC_OFF') && isRecording) {
        setRecordingState(false, false);
      } else if (line.includes('EVENT:STREAM_ON')) {
        isStreaming = true;
        if (btnToggleStream) btnToggleStream.textContent = 'Pause Stream';
      } else if (line.includes('EVENT:STREAM_OFF')) {
        isStreaming = false;
        if (btnToggleStream) btnToggleStream.textContent = 'Resume Stream';
      } else if (line.includes('EVENT:SCREEN_')) {
        const idx = parseInt(line.split('EVENT:SCREEN_')[1], 10);
        if (!isNaN(idx)) xiaoScreenIndex = idx % 4;
      } else if (line.includes('EVENT:RESET_DONE')) {
        resetRomAnalytics();
      }
      return;
    }

    // 2. Handle Compact High-Speed iPhone BLE Packet (13 fields):
    // B,t,fe,ru,ps,ax1,ay1,az1,ax2,ay2,az2,vbat,rec
    if (line.startsWith('B,')) {
      const p = line.substring(2).split(',').map(Number);
      if (p.length >= 11) {
        const t = p[0];
        const rawFe = p[1];
        const rawRu = p[2];
        const rawPs = p[3];
        const ax1 = p[4], ay1 = p[5], az1 = p[6];
        const ax2 = p[7], ay2 = p[8], az2 = p[9];
        const vbat = p[10] || 4.12;

        // Synthesize smooth gyro rates from angle derivatives so all charts & CSVs work identically
        const dt = (bleSynthPrev.t > 0 && t > bleSynthPrev.t)
          ? Math.max(0.005, t - bleSynthPrev.t)
          : 0.02;
        const instGx2 = Math.max(-1200, Math.min(1200, (rawFe - bleSynthPrev.fe) / dt));
        const instGy2 = Math.max(-1200, Math.min(1200, (rawRu - bleSynthPrev.ru) / dt));
        const instGz2 = Math.max(-1200, Math.min(1200, (rawPs - bleSynthPrev.ps) / dt));

        const gx2 = 0.65 * bleSynthPrev.gx2 + 0.35 * instGx2;
        const gy2 = 0.65 * bleSynthPrev.gy2 + 0.35 * instGy2;
        const gz2 = 0.65 * bleSynthPrev.gz2 + 0.35 * instGz2;
        bleSynthPrev = { t, fe: rawFe, ru: rawRu, ps: rawPs, gx2, gy2, gz2 };

        handleTelemetryFrame({
          t,
          s1: { ax: ax1, ay: ay1, az: az1, gx: 0, gy: 0, gz: 0, mx: 25, my: 0, mz: -40 },
          s2: { ax: ax2, ay: ay2, az: az2, gx: gx2, gy: gy2, gz: gz2, mx: 25, my: 0, mz: -40 },
          rawFe,
          rawRu,
          rawPs,
          vbat
        });
      }
      return;
    }

    // 3. Handle Full USB 24-Field Packet:
    // D,t,ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,ax2,ay2,az2,gx2,gy2,gz2,mx2,my2,mz2,fe,ru,ps,vbat,rec
    if (line.startsWith('D,')) {
      const p = line.substring(2).split(',').map(Number);
      if (p.length >= 23) {
        handleTelemetryFrame({
          t: p[0],
          s1: { ax: p[1], ay: p[2], az: p[3], gx: p[4], gy: p[5], gz: p[6], mx: p[7], my: p[8], mz: p[9] },
          s2: { ax: p[10], ay: p[11], az: p[12], gx: p[13], gy: p[14], gz: p[15], mx: p[16], my: p[17], mz: p[18] },
          rawFe: p[19],
          rawRu: p[20],
          rawPs: p[21],
          vbat: p[22] || 4.12
        });
      } else if (p.length >= 17) {
        handleTelemetryFrame({
          t: p[0],
          s1: { ax: p[1], ay: p[2], az: p[3], gx: p[4], gy: p[5], gz: p[6], mx: 25, my: 0, mz: -40 },
          s2: { ax: p[7], ay: p[8], az: p[9], gx: p[10], gy: p[11], gz: p[12], mx: 25, my: 0, mz: -40 },
          rawFe: p[13],
          rawRu: p[14],
          rawPs: p[15],
          vbat: p[16] || 4.12
        });
      } else if (p.length >= 10) {
        // Simplified 10-field packet from code_simple.py: D,t,fe,ru,ps,ax1,ay1,az1,ax2,ay2,az2[,vbat]
        handleTelemetryFrame({
          t: p[0],
          s1: { ax: p[4], ay: p[5], az: p[6], gx: 0, gy: 0, gz: 0, mx: 25, my: 0, mz: -40 },
          s2: { ax: p[7], ay: p[8], az: p[9], gx: 0, gy: 0, gz: 0, mx: 25, my: 0, mz: -40 },
          rawFe: p[1],
          rawRu: p[2],
          rawPs: p[3],
          vbat: p[10] || 4.12
        });
      }
      return;
    }

    // 4. Handle Memory Dump Line: MEM,i,t,fe,ru,ps,a1,a2,g1,g2
    if (line.startsWith('MEM,')) {
      const parts = line.split(',');
      if (parts.length >= 10) {
        const feVal = parseFloat(parts[3]);
        const ruVal = parseFloat(parts[4]);
        const psVal = parseFloat(parts[5]);
        const estFemur = Number((feVal * 0.44).toFixed(2));
        const recRow = {
          iso: new Date().toISOString(),
          t: parseFloat(parts[2]),
          joint: getPreset(currentJoint).name,
          fe: feVal,
          ru: ruVal,
          ps: psVal,
          femur: estFemur,
          tot: Math.hypot(feVal, ruVal, psVal),
          jVel: Math.abs(parseFloat(parts[9]) - parseFloat(parts[8])),
          ax1: Number((-9.81 * Math.sin((estFemur * Math.PI) / 180)).toFixed(3)),
          ay1: 0,
          az1: Number((9.81 * Math.cos((estFemur * Math.PI) / 180)).toFixed(3)),
          gx1: parseFloat(parts[8]), gy1: 0, gz1: 0,
          mx1: 0, my1: 0, mz1: 0,
          ax2: 0, ay2: 0, az2: parseFloat(parts[7]),
          gx2: parseFloat(parts[9]), gy2: 0, gz2: 0,
          mx2: 0, my2: 0, mz2: 0,
          a1Mag: parseFloat(parts[6]),
          a2Mag: parseFloat(parts[7]),
          g1Mag: parseFloat(parts[8]),
          g2Mag: parseFloat(parts[9]),
          vbat: 4.12
        };
        recordedData.push(recRow);
        recordCountBadge.textContent = `${recordedData.length} samples (Memory Dump)`;
        btnExport.disabled = false;
        appendRecordTableRow(recRow);

        if (memDumpDebounceTimer) clearTimeout(memDumpDebounceTimer);
        memDumpDebounceTimer = setTimeout(() => {
          if (recordedData.length > 0) {
            saveRecordedTrial(recordedData, 'XIAO Memory Dump');
            showStatusToast(`✓ Memory Dump Saved to Recorded Trials (${recordedData.length} samples)`, 4500);
          }
        }, 420);
      }
      return;
    }

    appendTerminalLine(line);
  }

  // --- Recording Table & Automatic Standing Calibration on Start Recording ---
  function appendRecordTableRow(r) {
    if (recordedData.length === 1) {
      recordTableBody.innerHTML = '';
    }
    const femurVal = typeof r.femur === 'number' && Number.isFinite(r.femur) ? r.femur : 0;
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${r.t.toFixed(3)}</td>
      <td style="color:var(--plane-fe);">${r.fe.toFixed(1)}</td>
      <td style="color:var(--plane-ru);">${r.ru.toFixed(1)}</td>
      <td style="color:var(--plane-ps);">${r.ps.toFixed(1)}</td>
      <td style="color:var(--plane-femur); font-weight:700;">${femurVal.toFixed(1)}</td>
      <td>${(r.a1Mag ?? 9.81).toFixed(2)}</td>
      <td>${(r.a2Mag ?? 9.81).toFixed(2)}</td>
      <td>${(r.g2Mag ?? r.jVel ?? 0).toFixed(1)}</td>
    `;
    recordTableBody.insertBefore(tr, recordTableBody.firstChild);
    while (recordTableBody.children.length > 60) {
      recordTableBody.removeChild(recordTableBody.lastChild);
    }
  }

  function setRecordingState(active, notifyBoard = true) {
    isRecording = active;
    if (isRecording) {
      if (isReplayingTrial) {
        stopTrialReplay(false);
      }
      // 1. Immediately zero client-side standing offsets, clear drift/samples, reset ROM stats
      neutralOffset = { ...rawAnglesLatest };
      isCalibrated = true;
      recordedData = [];
      recordCountBadge.textContent = '0 samples';
      recordTimeBadge.textContent = '00:00.0';
      recordTableBody.innerHTML = '';
      clearChartBuffers();
      resetRomAnalytics();
      if (joint3D && typeof joint3D.tare === 'function') {
        joint3D.tare();
      }
      if (isDemoMode) {
        demoStart = performance.now();
      }

      // 2. Update Status Badge & Show Toast
      if (calBadge) {
        calBadge.textContent = 'Calibrated Standing Pose (0°, 0°, 0°) — Recording Live';
        calBadge.style.background = '#f0fdf4';
        calBadge.style.color = '#16a34a';
        calBadge.style.borderColor = '#bbf7d0';
      }
      showStatusToast('Calibrated Standing Pose (0°, 0°, 0°) — Recording Live', 5000);
      appendTerminalLine('▶ Auto-Calibrated Standing Pose (0°, 0°, 0°) — Recording Started', 'success');

      // 3. Update Record Button & Timer
      btnRecord.classList.add('recording');
      if (btnRecordLabel) {
        btnRecordLabel.textContent = 'Stop Recording (Recording Live...)';
      } else {
        btnRecord.textContent = 'Stop Recording';
      }
      recordStartTime = performance.now();
      if (recordTimerInterval) clearInterval(recordTimerInterval);
      recordTimerInterval = setInterval(() => {
        const sec = (performance.now() - recordStartTime) / 1000;
        const mins = String(Math.floor(sec / 60)).padStart(2, '0');
        const rem = (sec % 60).toFixed(1).padStart(4, '0');
        recordTimeBadge.textContent = `${mins}:${rem}`;
      }, 100);

      // 4. Notify XIAO Board to Perform Hardware Stationary Gyro-Bias + Gravity Calibration & Reset t=0
      if (notifyBoard) {
        sendDeviceCommand('CMD:REC_START');
      }
    } else {
      btnRecord.classList.remove('recording');
      if (btnRecordLabel) {
        btnRecordLabel.textContent = 'Start Recording (Auto-Calibrates Standing 0°)';
      } else {
        btnRecord.textContent = 'Start Recording (Auto-Calibrates Standing 0°)';
      }
      if (recordTimerInterval) clearInterval(recordTimerInterval);
      if (calBadge) {
        calBadge.textContent = `Recording Complete (${recordedData.length} samples)`;
      }
      if (recordedData.length > 0) {
        saveRecordedTrial(recordedData);
      }
      showStatusToast(`■ Recording Stopped & Saved to Trials (${recordedData.length} samples)`, 4000);
      if (notifyBoard) {
        sendDeviceCommand('CMD:REC_STOP');
      }
    }
  }

  btnRecord.addEventListener('click', () => setRecordingState(!isRecording, true));
  if (btnHwRec) btnHwRec.addEventListener('click', () => setRecordingState(!isRecording, true));

  if (btnClearRecord) {
    btnClearRecord.addEventListener('click', () => {
      recordedData = [];
      recordCountBadge.textContent = '0 samples';
      recordTimeBadge.textContent = '00:00.0';
      recordTableBody.innerHTML = `<tr><td colspan="8" style="text-align:center; color:#94a3b8; padding:1.1rem;">Buffer cleared.</td></tr>`;
    });
  }

  // --- Export / Share CSV (Supports Native iPhone iOS Share Sheet + Standard Download) ---
  async function exportSamplesToCsv(samplesArray, labelTag = '') {
    const dataToExport = (samplesArray && samplesArray.length)
      ? samplesArray
      : (recordedData.length ? recordedData : (getSelectedTrial()?.samples || []));
    if (!dataToExport.length) {
      showStatusToast('No recorded samples available to export.', 3000);
      return;
    }
    const headers = [
      'Timestamp_ISO', 'Elapsed_s', 'Joint_Preset',
      'FlexExt_deg', 'VarusValgus_AbAd_deg', 'IntExtRot_ProSup_deg', 'FemurPitch_S1_deg', 'Total3DAngle_deg', 'JointAngVel_dps',
      'S1_Ax_ms2', 'S1_Ay_ms2', 'S1_Az_ms2', 'S1_Gx_dps', 'S1_Gy_dps', 'S1_Gz_dps', 'S1_Mx_uT', 'S1_My_uT', 'S1_Mz_uT',
      'S2_Ax_ms2', 'S2_Ay_ms2', 'S2_Az_ms2', 'S2_Gx_dps', 'S2_Gy_dps', 'S2_Gz_dps', 'S2_Mx_uT', 'S2_My_uT', 'S2_Mz_uT',
      'Battery_V'
    ];
    const rows = dataToExport.map((r) => [
      r.iso || new Date().toISOString(),
      r.t,
      r.joint || getPreset(currentJoint).name,
      r.fe,
      r.ru,
      r.ps,
      r.femur ?? 0,
      r.tot ?? Number(Math.hypot(r.fe || 0, r.ru || 0, r.ps || 0).toFixed(2)),
      r.jVel ?? 0,
      r.ax1 ?? 0, r.ay1 ?? 0, r.az1 ?? 9.81, r.gx1 ?? 0, r.gy1 ?? 0, r.gz1 ?? 0, r.mx1 ?? 0, r.my1 ?? 0, r.mz1 ?? 0,
      r.ax2 ?? 0, r.ay2 ?? 0, r.az2 ?? 9.81, r.gx2 ?? 0, r.gy2 ?? 0, r.gz2 ?? 0, r.mx2 ?? 0, r.my2 ?? 0, r.mz2 ?? 0,
      r.vbat ?? 4.12
    ].join(','));
    const csv = [headers.join(','), ...rows].join('\n');
    const cleanTag = labelTag ? `_${labelTag.replace(/[^a-zA-Z0-9_-]/g, '_')}` : '';
    const filename = `XIAO_2IMU_${getPreset(currentJoint).name}${cleanTag}_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`;

    // Try iPhone / iOS Native Share Sheet first if supported on mobile
    try {
      if (isIOS && navigator.canShare && navigator.share) {
        const csvFile = new File([csv], filename, { type: 'text/csv' });
        if (navigator.canShare({ files: [csvFile] })) {
          await navigator.share({
            files: [csvFile],
            title: 'XIAO Dual-IMU 3D Joint Kinematics CSV',
            text: `Recorded ${dataToExport.length} samples (${getPreset(currentJoint).name})`
          });
          return;
        }
      }
    } catch (shareErr) {
      if (shareErr.name === 'AbortError') return;
    }

    // Standard browser CSV download
    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    showStatusToast(`✓ Exported ${dataToExport.length} samples to ${filename}`, 3500);
  }

  btnExport.addEventListener('click', () => {
    const activeSamples = recordedData.length ? recordedData : (getSelectedTrial()?.samples || []);
    exportSamplesToCsv(activeSamples);
  });

  // --- Hardware Sync Buttons (Stream, Screen, Dump Memory, Reset) ---
  function toggleStreaming() {
    isStreaming = !isStreaming;
    if (btnToggleStream) btnToggleStream.textContent = isStreaming ? 'Pause Stream' : 'Resume Stream';
    sendDeviceCommand(isStreaming ? 'CMD:START_STREAM' : 'CMD:STOP_STREAM');
  }
  if (btnToggleStream) btnToggleStream.addEventListener('click', toggleStreaming);
  if (btnHwStream) btnHwStream.addEventListener('click', toggleStreaming);

  if (btnHwScreen) {
    btnHwScreen.addEventListener('click', () => {
      xiaoScreenIndex = (xiaoScreenIndex + 1) % 4;
      sendDeviceCommand('CMD:SCREEN');
    });
  }

  if (btnDumpMem) {
    btnDumpMem.addEventListener('click', () => {
      sendDeviceCommand('CMD:DUMP_MEM');
    });
  }

  if (btnResetSys) {
    btnResetSys.addEventListener('click', () => {
      clearChartBuffers();
      resetRomAnalytics();
      neutralOffset = { fe: 0, ru: 0, ps: 0 };
      isCalibrated = false;
      if (calBadge) {
        calBadge.textContent = 'Reset — Stand Still & Record to Auto-Calibrate';
        calBadge.style.background = '';
        calBadge.style.color = '';
        calBadge.style.borderColor = '';
      }
      sendDeviceCommand('CMD:RESET');
      showStatusToast('System & Orientation Filters Reset', 3000);
    });
  }

  // 3D Viewport Controls
  if (btnToggleAxes) {
    btnToggleAxes.addEventListener('click', () => {
      const vis = joint3D.toggleAxes();
      btnToggleAxes.textContent = vis ? 'Hide Axes' : 'Show Axes';
    });
  }
  if (btnResetCam) {
    btnResetCam.addEventListener('click', () => joint3D.resetView());
  }

  // --- Terminal Console ---
  function appendTerminalLine(text, type = '') {
    if (!terminalOut) return;
    const div = document.createElement('div');
    div.className = `terminal-line ${type}`;
    div.textContent = text;
    terminalOut.appendChild(div);
    while (terminalOut.childNodes.length > 150) {
      terminalOut.removeChild(terminalOut.firstChild);
    }
    if (!autoscrollCheck || autoscrollCheck.checked) {
      terminalOut.scrollTop = terminalOut.scrollHeight;
    }
  }

  if (btnClearTerminal) {
    btnClearTerminal.addEventListener('click', () => {
      terminalOut.innerHTML = '';
    });
  }

  if (btnSendCmd && cmdInput) {
    btnSendCmd.addEventListener('click', () => {
      if (cmdInput.value.trim()) {
        sendDeviceCommand(cmdInput.value.trim());
        cmdInput.value = '';
      }
    });
    cmdInput.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && cmdInput.value.trim()) {
        sendDeviceCommand(cmdInput.value.trim());
        cmdInput.value = '';
      }
    });
  }

  // --- Direct iPhone / Web Bluetooth Connection (BLE NUS) ---
  async function connectBle() {
    if (!('bluetooth' in navigator)) {
      if (iosBleBanner) iosBleBanner.style.display = 'flex';
      alert(
        'iPhone Direct Bluetooth Tip:\n\n' +
        'Open this page in the free "Bluefy – Web BLE Browser" app from the iOS App Store (or Chrome/Edge on Mac/PC) to connect directly via Bluetooth.'
      );
      return;
    }
    try {
      if (isDemoMode) stopDemoMode();
      try {
        bleDevice = await navigator.bluetooth.requestDevice({
          filters: [
            { services: [NUS_SERVICE_UUID] },
            { namePrefix: 'XIAO' }
          ],
          optionalServices: [NUS_SERVICE_UUID]
        });
      } catch (filterErr) {
        if (filterErr.name === 'NotFoundError') throw filterErr;
        bleDevice = await navigator.bluetooth.requestDevice({
          acceptAllDevices: true,
          optionalServices: [NUS_SERVICE_UUID]
        });
      }

      bleDevice.addEventListener('gattserverdisconnected', () => {
        if (connectionType === 'ble') {
          isConnected = false;
          connectionType = null;
          statusPill.className = 'status-pill';
          statusText.textContent = 'Disconnected';
          btnConnectBle.textContent = 'Connect Bluetooth (iPhone / BLE)';
          btnConnectBle.className = 'btn btn-ble btn-connect-main';
          appendTerminalLine('BLE connection closed.', 'info');
        }
      });

      const server = await bleDevice.gatt.connect();
      const service = await server.getPrimaryService(NUS_SERVICE_UUID);
      bleTxChar = await service.getCharacteristic(NUS_TX_CHAR_UUID);
      try {
        bleRxChar = await service.getCharacteristic(NUS_RX_CHAR_UUID);
      } catch (e) {
        bleRxChar = null;
      }

      await bleTxChar.startNotifications();
      bleTxChar.addEventListener('characteristicvaluechanged', (ev) => {
        bleLineBuffer += utf8Decoder.decode(ev.target.value);
        const lines = bleLineBuffer.split(/\r?\n/);
        bleLineBuffer = lines.pop();
        for (const l of lines) parseIncomingLine(l);
      });

      isConnected = true;
      connectionType = 'ble';
      statusPill.className = 'status-pill bluetooth';
      statusText.textContent = `BLE: ${bleDevice.name || 'XIAO-IMU'}`;
      btnConnectBle.textContent = 'Disconnect Bluetooth';
      btnConnectBle.classList.replace('btn-ble', 'btn-danger');
      showStatusToast(`✓ Connected via Bluetooth to ${bleDevice.name || 'XIAO-IMU'}`, 3500);
      appendTerminalLine(`✓ Connected wirelessly over BLE to ${bleDevice.name || 'XIAO-IMU'}`, 'success');
    } catch (err) {
      if (err.name !== 'NotFoundError') {
        appendTerminalLine(`BLE Error: ${err.message}`, 'error');
      }
    }
  }

  async function disconnectBle() {
    isConnected = false;
    connectionType = null;
    if (bleDevice && bleDevice.gatt.connected) {
      try { bleDevice.gatt.disconnect(); } catch (e) {}
    }
    statusPill.className = 'status-pill';
    statusText.textContent = 'Disconnected';
    btnConnectBle.textContent = 'Connect Bluetooth (iPhone / BLE)';
    btnConnectBle.className = 'btn btn-ble btn-connect-main';
    appendTerminalLine('BLE disconnected.', 'info');
  }

  btnConnectBle.addEventListener('click', () => {
    if (isConnected && connectionType === 'ble') disconnectBle();
    else connectBle();
  });

  // --- Web Serial Connection (USB-C) ---
  async function connectUsb() {
    if (!('serial' in navigator)) {
      alert('USB Web Serial requires Chrome or Edge on a laptop/desktop. On iPhone, tap "Connect Bluetooth (iPhone / BLE)".');
      return;
    }
    try {
      if (isDemoMode) stopDemoMode();
      serialPort = await navigator.serial.requestPort();
      await serialPort.open({ baudRate: 115200 });
      isConnected = true;
      connectionType = 'usb';
      statusPill.className = 'status-pill connected';
      statusText.textContent = 'USB Connected';
      btnConnectUsb.textContent = 'Disconnect USB';
      btnConnectUsb.classList.add('btn-danger');
      showStatusToast('✓ Connected via USB Serial (115200 baud)', 3000);
      appendTerminalLine('✓ Connected to Seeed XIAO nRF52840 via USB Serial', 'success');
      readUsbLoop();
    } catch (err) {
      if (err.name !== 'NotFoundError') {
        appendTerminalLine(`USB Error: ${err.message}`, 'error');
      }
    }
  }

  async function readUsbLoop() {
    let lineBuffer = '';
    while (serialPort && serialPort.readable && isConnected && connectionType === 'usb') {
      const textDecoder = new TextDecoderStream();
      serialPort.readable.pipeTo(textDecoder.writable).catch(() => {});
      serialReader = textDecoder.readable.getReader();
      try {
        while (true) {
          const { value, done } = await serialReader.read();
          if (done) break;
          if (value) {
            lineBuffer += value;
            const lines = lineBuffer.split(/\r?\n/);
            lineBuffer = lines.pop();
            for (const l of lines) parseIncomingLine(l);
          }
        }
      } catch (e) {
        break;
      } finally {
        serialReader.releaseLock();
      }
    }
  }

  async function disconnectUsb() {
    isConnected = false;
    connectionType = null;
    if (serialReader) {
      try { await serialReader.cancel(); } catch (e) {}
      serialReader = null;
    }
    if (serialPort) {
      try { await serialPort.close(); } catch (e) {}
      serialPort = null;
    }
    statusPill.className = 'status-pill';
    statusText.textContent = 'Disconnected';
    btnConnectUsb.textContent = 'Connect USB';
    btnConnectUsb.className = 'btn btn-secondary-usb';
    appendTerminalLine('USB Serial disconnected.', 'info');
  }

  if (btnConnectUsb) {
    btnConnectUsb.addEventListener('click', () => {
      if (isConnected && connectionType === 'usb') disconnectUsb();
      else connectUsb();
    });
  }

  // --- Realistic Knee Flexion/Extension Dual-IMU Biomechanical Simulation (3D Demo Mode) ---
  let demoTimer = null;
  let demoStart = 0;

  function startDemoMode() {
    if (isReplayingTrial) {
      stopTrialReplay(false);
    }
    isDemoMode = true;
    demoStart = performance.now();
    if (joint3D) {
      joint3D.s1LocalTare.set(0, 0, 1);
      joint3D.hasTaredS1 = true;
      joint3D.s1TableYaw = 0;
    }
    statusPill.className = 'status-pill demo';
    statusText.textContent = '3D Demo (60 Hz)';
    btnDemo.textContent = 'Stop Demo';
    btnDemo.classList.add('btn-danger');
    appendTerminalLine('▶ Started 3D Knee Flexion/Extension Biomechanical Simulation @ 60 Hz', 'info');

    demoTimer = setInterval(() => {
      if (isReplayingTrial) return;
      const t = (performance.now() - demoStart) / 1000;

      // Compound Femur + Tibia Squat / High-Knee Step Cycle (0..1..0)
      const cyc = 0.5 * (1 - Math.cos(t * 1.6));

      // Simulate Proximal Thigh (Sensor #1 @ 0x69) sagittal inclination (0° standing -> 42° flexion)
      const s1Pitch = currentJoint === 'knee'
        ? 42.0 * cyc
        : 14.0 * Math.sin(t * 1.4);
      const s1Roll = 2.5 * Math.sin(t * 0.8);
      const ax1 = -9.81 * Math.sin((s1Pitch * Math.PI) / 180);
      const ay1 = 9.81 * Math.sin((s1Roll * Math.PI) / 180);
      const az1 = 9.81 * Math.cos((s1Pitch * Math.PI) / 180) * Math.cos((s1Roll * Math.PI) / 180);
      const gx1 = currentJoint === 'knee'
        ? 21.0 * 1.6 * Math.sin(t * 1.6)
        : 14.0 * Math.cos(t * 1.4);
      const gy1 = 2.0 * Math.cos(t * 0.8);
      const gz1 = 0.25 * Math.sin(t * 0.5);

      // Simulate Distal Lower Leg / Shank (Sensor #2 @ 0x68):
      // Anatomical Knee Flexion (0° standing extension -> 92° knee flexion)
      const rawFe = currentJoint === 'knee'
        ? 92.0 * cyc
        : 42.0 * Math.sin(t * 1.6);
      const rawRu = 6.5 * Math.sin(t * 1.6);   // Frontal plane Varus/Valgus excursion (+-6.5°)
      const rawPs = 11.0 * Math.sin(t * 0.8);  // Transverse Tibial Int/Ext rotation (+-11°)

      const totalPitch2 = s1Pitch - rawFe;
      const totalRoll2 = s1Roll + rawRu;
      const ax2 = -9.81 * Math.sin((totalPitch2 * Math.PI) / 180);
      const ay2 = 9.81 * Math.sin((totalRoll2 * Math.PI) / 180);
      const az2 = 9.81 * Math.cos((totalPitch2 * Math.PI) / 180) * Math.cos((totalRoll2 * Math.PI) / 180);
      const gx2 = gx1 + 46.0 * 1.6 * Math.sin(t * 1.6);
      const gy2 = gy1 + 6.5 * 1.6 * Math.cos(t * 1.6);
      const gz2 = gz1 + 11.0 * 0.8 * Math.cos(t * 0.8);

      handleTelemetryFrame({
        t,
        s1: { ax: ax1, ay: ay1, az: az1, gx: gx1, gy: gy1, gz: gz1, mx: 24.5, my: 4.2, mz: -41.0 },
        s2: { ax: ax2, ay: ay2, az: az2, gx: gx2, gy: gy2, gz: gz2, mx: 22.0, my: 8.0, mz: -39.5 },
        rawFe,
        rawRu,
        rawPs,
        vbat: 4.12
      });
    }, 16.6); // ~60 Hz
  }

  function stopDemoMode() {
    isDemoMode = false;
    if (demoTimer) clearInterval(demoTimer);
    demoTimer = null;
    statusPill.className = 'status-pill';
    statusText.textContent = 'Disconnected';
    btnDemo.textContent = '3D Demo';
    btnDemo.classList.remove('btn-danger');
    appendTerminalLine('■ Stopped 3D Demo Simulation.', 'info');
  }

  if (btnDemo) {
    btnDemo.addEventListener('click', () => {
      if (isDemoMode) stopDemoMode();
      else startDemoMode();
    });
  }

  // ============================================================================
  // RECORDED TRIALS REVIEW, 3D PLAYBACK & PUBLICATION-GRADE GRAPHICAL REPORTS
  // ============================================================================

  function computeTrialStats(samples) {
    const n = samples.length;
    const emptyAxis = { min: 0, max: 0, span: 0, mean: 0, sd: 0, peakVel: 0, minTime: 0, maxTime: 0, minIdx: 0, maxIdx: 0 };
    if (!n) {
      return {
        fe: { ...emptyAxis },
        ru: { ...emptyAxis },
        ps: { ...emptyAxis },
        femur: { ...emptyAxis },
        tot: { ...emptyAxis },
        peakJVel: 0,
        meanJVel: 0,
        reps: 0,
        sampleCount: 0,
        durationSec: 0,
        avgHz: 0
      };
    }

    const t0 = samples[0].t || 0;
    const durationSec = Math.max(0.02, (samples[n - 1].t || 0) - t0);
    const avgHz = n > 1 ? Math.round((n - 1) / durationSec) : 25;

    const axes = ['fe', 'ru', 'ps', 'femur', 'tot'];
    const result = {};

    axes.forEach((k) => {
      let min = Infinity;
      let max = -Infinity;
      let minTime = 0;
      let maxTime = 0;
      let minIdx = 0;
      let maxIdx = 0;
      let sum = 0;
      let peakVel = 0;

      for (let i = 0; i < n; i++) {
        const v = Number(samples[i][k] ?? 0);
        const tRel = Number((samples[i].t || 0) - t0);
        if (v < min) { min = v; minTime = tRel; minIdx = i; }
        if (v > max) { max = v; maxTime = tRel; maxIdx = i; }
        sum += v;

        if (i > 0) {
          const dt = Math.max(0.004, (samples[i].t - samples[i - 1].t) || (durationSec / n));
          const prevV = Number(samples[i - 1][k] ?? 0);
          const vel = Math.abs(v - prevV) / dt;
          if (vel < 1800 && vel > peakVel) peakVel = vel;
        }
      }

      const mean = sum / n;
      let sqDiffSum = 0;
      for (let i = 0; i < n; i++) {
        const d = Number(samples[i][k] ?? 0) - mean;
        sqDiffSum += d * d;
      }
      const sd = Math.sqrt(sqDiffSum / n);

      result[k] = {
        min,
        max,
        span: max - min,
        mean,
        sd,
        peakVel,
        minTime,
        maxTime,
        minIdx,
        maxIdx
      };
    });

    // Count flexion repetitions & peak joint angular velocity
    let reps = 0;
    let inCycle = false;
    let peakJVel = 0;
    let sumJVel = 0;
    for (let i = 0; i < n; i++) {
      const fe = Math.abs(Number(samples[i].fe || 0));
      if (!inCycle && fe > 22.0) {
        inCycle = true;
      } else if (inCycle && fe < 7.5) {
        inCycle = false;
        reps += 1;
      }
      const jv = Number(samples[i].jVel || 0);
      if (jv > peakJVel) peakJVel = jv;
      sumJVel += jv;
    }

    result.peakJVel = Math.max(peakJVel, result.fe.peakVel);
    result.meanJVel = sumJVel / n;
    result.reps = reps;
    result.sampleCount = n;
    result.durationSec = durationSec;
    result.avgHz = avgHz;
    return result;
  }

  function generatePreSeededSampleTrial() {
    const samples = [];
    const totalSamples = 250;
    const duration = 10.0; // 10.0s @ 25 Hz
    const baseMs = Date.now() - 10000;

    // 3 smooth anatomical squat / high-knee flexion cycles at centers 2.0s, 5.0s, 8.0s
    const repCenters = [2.0, 5.0, 8.0];
    const repPeaksFe = [91.5, 96.4, 93.8];
    const repPeaksFemur = [40.2, 43.5, 41.8];
    const halfWidth = 1.25;

    for (let i = 0; i < totalSamples; i++) {
      const t = Number(((i / (totalSamples - 1)) * duration).toFixed(3));
      let fe = 0.6 * Math.sin(t * 1.1);
      let femur = 0.3 * Math.sin(t * 1.1);
      let feVel = 0;
      let femurVel = 0;

      for (let r = 0; r < repCenters.length; r++) {
        const dtRep = t - repCenters[r];
        if (Math.abs(dtRep) <= halfWidth) {
          const phase = (dtRep / halfWidth) * Math.PI; // -pi .. +pi
          const env = 0.5 * (1 + Math.cos(phase));     // raised cosine 0..1..0
          const dEnv = -0.5 * (Math.PI / halfWidth) * Math.sin(phase);
          fe += repPeaksFe[r] * env;
          femur += repPeaksFemur[r] * env;
          feVel += repPeaksFe[r] * dEnv;
          femurVel += repPeaksFemur[r] * dEnv;
        }
      }

      // Physiological frontal plane Varus/Valgus & transverse Tibial Int/Ext rotation coupled with knee flexion
      const normFlex = Math.max(0, fe / 95.0);
      const ru = Number((6.4 * normFlex * Math.sin(t * 2.1 + 0.4) + 1.8 * normFlex).toFixed(2));
      const ps = Number((11.2 * normFlex * Math.cos(t * 1.05 - 0.3) - 2.4 * normFlex).toFixed(2));
      fe = Number(fe.toFixed(2));
      femur = Number(femur.toFixed(2));
      const tot = Number(Math.hypot(fe, ru, ps).toFixed(2));

      const gx1 = Number(femurVel.toFixed(2));
      const gy1 = Number((2.2 * Math.cos(t * 1.5)).toFixed(2));
      const gz1 = 0;
      const gx2 = Number((femurVel - feVel).toFixed(2));
      const gy2 = Number((6.5 * Math.cos(t * 2.1)).toFixed(2));
      const gz2 = Number((-11.2 * Math.sin(t * 1.05)).toFixed(2));
      const jVel = Number(Math.hypot(gx2 - gx1, gy2 - gy1, gz2 - gz1).toFixed(2));

      const ax1 = Number((-9.81 * Math.sin((femur * Math.PI) / 180)).toFixed(3));
      const ay1 = Number((9.81 * Math.sin((2.0 * normFlex * Math.PI) / 180)).toFixed(3));
      const az1 = Number((9.81 * Math.cos((femur * Math.PI) / 180)).toFixed(3));

      const shankPitch = femur - fe;
      const ax2 = Number((-9.81 * Math.sin((shankPitch * Math.PI) / 180)).toFixed(3));
      const ay2 = Number((9.81 * Math.sin((ru * Math.PI) / 180)).toFixed(3));
      const az2 = Number((9.81 * Math.cos((shankPitch * Math.PI) / 180)).toFixed(3));

      samples.push({
        iso: new Date(baseMs + t * 1000).toISOString(),
        t,
        joint: 'KNEE',
        fe,
        ru,
        ps,
        femur,
        tot,
        jVel,
        ax1, ay1, az1,
        gx1, gy1, gz1,
        mx1: 24.5, my1: 4.1, mz1: -41.0,
        ax2, ay2, az2,
        gx2, gy2, gz2,
        mx2: 22.0, my2: 7.8, mz2: -39.5,
        a1Mag: 9.81,
        a2Mag: 9.81,
        g1Mag: Number(Math.hypot(gx1, gy1, gz1).toFixed(1)),
        g2Mag: Number(Math.hypot(gx2, gy2, gz2).toFixed(1)),
        vbat: 4.14
      });
    }

    const stats = computeTrialStats(samples);
    return {
      id: 'sample_trial_1',
      title: 'Sample Trial #1 — Knee Squat & High-Knee Step (3 Reps)',
      createdAt: new Date().toISOString(),
      joint: 'knee',
      jointName: 'KNEE',
      durationSec: stats.durationSec,
      sampleRateHz: stats.avgHz,
      samples,
      stats
    };
  }

  function loadSavedTrialsFromStorage() {
    try {
      const raw = localStorage.getItem(TRIALS_STORAGE_KEY);
      if (raw) {
        const parsed = JSON.parse(raw);
        if (Array.isArray(parsed) && parsed.length > 0) {
          savedTrials = parsed.map((tr) => {
            if (!tr.stats || !tr.stats.femur) {
              tr.stats = computeTrialStats(tr.samples || []);
            }
            return tr;
          });
        }
      }
    } catch (e) {
      console.warn('Could not load saved trials from localStorage:', e);
    }

    if (!savedTrials.length) {
      savedTrials = [generatePreSeededSampleTrial()];
      persistTrialsToStorage();
    }

    selectedTrialId = savedTrials[0].id;
    refreshTrialSelectorUI();
    selectTrialById(selectedTrialId, false);
    if (btnExport) btnExport.disabled = false;
  }

  function persistTrialsToStorage() {
    try {
      // Keep up to 12 most recent trials in localStorage
      const trimmed = savedTrials.slice(0, 12);
      localStorage.setItem(TRIALS_STORAGE_KEY, JSON.stringify(trimmed));
    } catch (e) {
      console.warn('localStorage quota warning while saving trials:', e);
    }
  }

  function getSelectedTrial() {
    return savedTrials.find((tr) => tr.id === selectedTrialId) || savedTrials[0] || null;
  }

  function saveRecordedTrial(rawSamples, customTag = '') {
    if (!rawSamples || !rawSamples.length) return null;
    const t0 = Number(rawSamples[0].t || 0);
    const normalizedSamples = rawSamples.map((r, idx) => {
      const tNorm = Number(Math.max(0, (r.t || idx * 0.02) - t0).toFixed(3));
      const fe = Number((r.fe ?? 0).toFixed(2));
      const ru = Number((r.ru ?? 0).toFixed(2));
      const ps = Number((r.ps ?? 0).toFixed(2));
      const femur = Number((r.femur ?? (fe * 0.44)).toFixed(2));
      return {
        ...r,
        t: tNorm,
        fe,
        ru,
        ps,
        femur,
        tot: Number((r.tot ?? Math.hypot(fe, ru, ps)).toFixed(2)),
        jVel: Number((r.jVel ?? 0).toFixed(2))
      };
    });

    const stats = computeTrialStats(normalizedSamples);
    const preset = getPreset(currentJoint);
    const timeStamp = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    const trialNum = savedTrials.length + 1;
    const tagPart = customTag ? ` [${customTag}]` : '';
    const title = `Trial #${trialNum} — ${preset.name}${tagPart} (${stats.durationSec.toFixed(1)}s • ${stats.reps} Reps • ${normalizedSamples.length} pts • ${timeStamp})`;

    const newTrial = {
      id: `trial_${Date.now()}_${Math.random().toString(36).slice(2, 6)}`,
      title,
      createdAt: new Date().toISOString(),
      joint: currentJoint,
      jointName: preset.name,
      durationSec: stats.durationSec,
      sampleRateHz: stats.avgHz,
      samples: normalizedSamples,
      stats
    };

    savedTrials.unshift(newTrial);
    persistTrialsToStorage();
    selectedTrialId = newTrial.id;
    refreshTrialSelectorUI();
    selectTrialById(newTrial.id, false);
    return newTrial;
  }

  function refreshTrialSelectorUI() {
    if (trialCountBadge) {
      trialCountBadge.textContent = `${savedTrials.length} Saved Trial${savedTrials.length === 1 ? '' : 's'}`;
    }
    if (!trialSelect) return;
    trialSelect.innerHTML = '';
    savedTrials.forEach((tr) => {
      const opt = document.createElement('option');
      opt.value = tr.id;
      opt.textContent = tr.title;
      if (tr.id === selectedTrialId) opt.selected = true;
      trialSelect.appendChild(opt);
    });
  }

  function selectTrialById(trialId, engage3D = false) {
    const tr = savedTrials.find((item) => item.id === trialId) || savedTrials[0];
    if (!tr) return;
    selectedTrialId = tr.id;
    if (trialSelect && trialSelect.value !== tr.id) {
      trialSelect.value = tr.id;
    }

    if (isTrialPlaying) {
      pauseTrialPlayback();
    }

    const st = tr.stats || computeTrialStats(tr.samples || []);
    tr.stats = st;

    // Populate Quick Stats Cards
    if (tstatFeRom) tstatFeRom.textContent = `${st.fe.span.toFixed(1)}°`;
    if (tstatFeMinmax) tstatFeMinmax.textContent = `Min ${st.fe.min.toFixed(1)}° / Max ${st.fe.max.toFixed(1)}°`;
    if (tstatRuRom) tstatRuRom.textContent = `${st.ru.span.toFixed(1)}°`;
    if (tstatRuMinmax) tstatRuMinmax.textContent = `Min ${st.ru.min.toFixed(1)}° / Max ${st.ru.max.toFixed(1)}°`;
    if (tstatPsRom) tstatPsRom.textContent = `${st.ps.span.toFixed(1)}°`;
    if (tstatPsMinmax) tstatPsMinmax.textContent = `Min ${st.ps.min.toFixed(1)}° / Max ${st.ps.max.toFixed(1)}°`;
    if (tstatFemurRom) tstatFemurRom.textContent = `${st.femur.span.toFixed(1)}°`;
    if (tstatFemurMinmax) tstatFemurMinmax.textContent = `Min ${st.femur.min.toFixed(1)}° / Max ${st.femur.max.toFixed(1)}°`;

    if (trialScrubber) {
      trialScrubber.min = '0';
      trialScrubber.max = String(Math.max(0, tr.samples.length - 1));
    }

    // Default scrub index to peak flexion sample so user immediately sees peak readout on graph
    const defaultIdx = st.fe.maxIdx || 0;
    applyTrialSampleTo3DAndUI(tr, defaultIdx, engage3D);
  }

  function applyTrialSampleTo3DAndUI(trial, idx, update3D = true) {
    if (!trial || !trial.samples || !trial.samples.length) return;
    const n = trial.samples.length;
    const clampedIdx = Math.max(0, Math.min(n - 1, Math.round(idx)));
    trialScrubIndex = clampedIdx;
    const s = trial.samples[clampedIdx];
    const t0 = trial.samples[0].t || 0;
    const tEnd = Math.max(0.01, (trial.samples[n - 1].t || 0) - t0);
    const tCur = Math.max(0, (s.t || 0) - t0);
    const femurVal = Number(s.femur ?? 0);

    if (trialScrubber) trialScrubber.value = String(clampedIdx);
    if (trialScrubTime) trialScrubTime.textContent = `t = ${tCur.toFixed(2)}s / ${tEnd.toFixed(2)}s`;
    if (scrubValFe) scrubValFe.textContent = `${s.fe >= 0 ? '+' : ''}${s.fe.toFixed(1)}°`;
    if (scrubValRu) scrubValRu.textContent = `${s.ru >= 0 ? '+' : ''}${s.ru.toFixed(1)}°`;
    if (scrubValPs) scrubValPs.textContent = `${s.ps >= 0 ? '+' : ''}${s.ps.toFixed(1)}°`;
    if (scrubValFemur) scrubValFemur.textContent = `${femurVal >= 0 ? '+' : ''}${femurVal.toFixed(1)}°`;
    if (scrubValVel) scrubValVel.textContent = `${Number(s.jVel || 0).toFixed(1)}°/s`;

    renderTrialReviewChart(trial, clampedIdx);

    if (update3D) {
      isReplayingTrial = true;
      if (hudReplayPill) hudReplayPill.style.display = 'inline-flex';

      // Reconstruct S1 gravity vector from recorded femur sagittal inclination so 3D Femur tilts identically
      const femRad = (femurVal * Math.PI) / 180;
      const s1Replay = {
        ax: -9.81 * Math.sin(femRad),
        ay: 0,
        az: 9.81 * Math.cos(femRad),
        gx: s.gx1 || 0,
        gy: s.gy1 || 0,
        gz: 0
      };
      const shankRad = ((femurVal - s.fe) * Math.PI) / 180;
      const ruRad = ((s.ru || 0) * Math.PI) / 180;
      const s2Replay = {
        ax: -9.81 * Math.sin(shankRad),
        ay: 9.81 * Math.sin(ruRad),
        az: 9.81 * Math.cos(shankRad) * Math.cos(ruRad),
        gx: s.gx2 || 0,
        gy: s.gy2 || 0,
        gz: s.gz2 || 0
      };

      if (joint3D) {
        joint3D.s1LocalTare.set(0, 0, 1);
        joint3D.hasTaredS1 = true;
        joint3D.s1TableYaw = 0;
        const steps = isTrialPlaying ? 1 : 3;
        for (let k = 0; k < steps; k++) {
          joint3D.updateKinematics(
            s1Replay,
            s2Replay,
            s.fe,
            s.ru,
            s.ps,
            95,
            trial.jointName || getPreset(currentJoint).name,
            true
          );
        }
      }

      // Mirror replay frame onto top live cards & horizon gauges
      const totAngle = Number(s.tot ?? Math.hypot(s.fe, s.ru, s.ps));
      if (valTime) valTime.textContent = tCur.toFixed(2);
      if (valFe) valFe.textContent = s.fe.toFixed(1);
      if (valRu) valRu.textContent = s.ru.toFixed(1);
      if (valPs) valPs.textContent = s.ps.toFixed(1);
      if (valFemurPitch) valFemurPitch.textContent = femurVal.toFixed(1);
      if (valTotAngle) valTotAngle.textContent = totAngle.toFixed(1);
      if (valAngVel) valAngVel.textContent = Number(s.jVel || 0).toFixed(1);

      if (hudFeVal) hudFeVal.textContent = `${s.fe >= 0 ? '+' : ''}${s.fe.toFixed(1)}°`;
      if (hudRuVal) hudRuVal.textContent = `${s.ru >= 0 ? '+' : ''}${s.ru.toFixed(1)}°`;
      if (hudPsVal) hudPsVal.textContent = `${s.ps >= 0 ? '+' : ''}${s.ps.toFixed(1)}°`;
      const fePct = currentJoint === 'knee' ? ((s.fe + 15) / 155) * 100 : ((s.fe + 90) / 180) * 100;
      if (barFeFill) barFeFill.style.width = `${Math.max(2, Math.min(100, fePct))}%`;
      if (barRuFill) barRuFill.style.width = `${Math.max(2, Math.min(100, ((s.ru + 45) / 90) * 100))}%`;
      if (barPsFill) barPsFill.style.width = `${Math.max(2, Math.min(100, ((s.ps + 45) / 90) * 100))}%`;
      updateXiaoLcdMirror(tCur, s1Replay, s2Replay, s.fe, s.ru, s.ps, s.vbat || 4.12, 95);
    }
  }

  function startTrialPlayback() {
    const trial = getSelectedTrial();
    if (!trial || !trial.samples || !trial.samples.length) return;
    isReplayingTrial = true;
    isTrialPlaying = true;
    if (hudReplayPill) hudReplayPill.style.display = 'inline-flex';
    if (btnTrialPlay) btnTrialPlay.innerHTML = '&#x23F8; Pause 3D Replay';

    if (trialScrubIndex >= trial.samples.length - 1) {
      trialScrubIndex = 0;
    }
    trialPlayFloatIdx = trialScrubIndex;
    trialPlayLastMs = performance.now();

    const stepLoop = (nowMs) => {
      if (!isTrialPlaying) return;
      const dtSec = Math.min(0.1, Math.max(0.001, (nowMs - trialPlayLastMs) / 1000));
      trialPlayLastMs = nowMs;

      const hz = (trial.stats && trial.stats.avgHz) ? trial.stats.avgHz : 25;
      trialPlayFloatIdx += dtSec * hz * trialPlaybackSpeed;

      if (trialPlayFloatIdx >= trial.samples.length - 1) {
        applyTrialSampleTo3DAndUI(trial, trial.samples.length - 1, true);
        pauseTrialPlayback();
        return;
      }

      applyTrialSampleTo3DAndUI(trial, Math.floor(trialPlayFloatIdx), true);
      trialPlayRafId = requestAnimationFrame(stepLoop);
    };

    if (trialPlayRafId) cancelAnimationFrame(trialPlayRafId);
    trialPlayRafId = requestAnimationFrame(stepLoop);
  }

  function pauseTrialPlayback() {
    isTrialPlaying = false;
    if (trialPlayRafId) {
      cancelAnimationFrame(trialPlayRafId);
      trialPlayRafId = null;
    }
    if (btnTrialPlay) btnTrialPlay.innerHTML = '&#x25B6; Replay Trial in 3D';
  }

  function stopTrialReplay(notifyToast = true) {
    pauseTrialPlayback();
    isReplayingTrial = false;
    if (hudReplayPill) hudReplayPill.style.display = 'none';
    const trial = getSelectedTrial();
    if (trial) renderTrialReviewChart(trial, trialScrubIndex);
    if (notifyToast) {
      showStatusToast('Returned to Live 3D Telemetry Mode', 2800);
    }
  }

  // --- High-DPI Interactive Trial Review Multi-Axis Canvas Renderer ---
  const trialChartPad = { left: 52, right: 24, top: 34, bottom: 34 };

  function renderTrialReviewChart(trial, playheadIdx = 0) {
    if (!trialReviewCanvas || !trial || !trial.samples || !trial.samples.length) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const cssW = trialReviewCanvas.clientWidth || 920;
    const cssH = trialReviewCanvas.clientHeight || 270;
    if (trialReviewCanvas.width !== Math.round(cssW * dpr) || trialReviewCanvas.height !== Math.round(cssH * dpr)) {
      trialReviewCanvas.width = Math.round(cssW * dpr);
      trialReviewCanvas.height = Math.round(cssH * dpr);
    }

    const ctx = trialReviewCanvas.getContext('2d');
    ctx.save();
    ctx.scale(dpr, dpr);

    const W = cssW;
    const H = cssH;
    const pad = trialChartPad;
    const plotW = Math.max(100, W - pad.left - pad.right);
    const plotH = Math.max(80, H - pad.top - pad.bottom);

    // Background
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, W, H);

    const samples = trial.samples;
    const n = samples.length;
    const st = trial.stats || computeTrialStats(samples);
    const t0 = samples[0].t || 0;
    const tMax = Math.max(0.1, (samples[n - 1].t || 0) - t0);

    const rawMin = Math.min(-15, st.fe.min, st.ru.min, st.ps.min, st.femur.min);
    const rawMax = Math.max(45, st.fe.max, st.ru.max, st.ps.max, st.femur.max);
    const yPad = Math.max(8, (rawMax - rawMin) * 0.12);
    const yMin = Math.floor((rawMin - yPad) / 10) * 10;
    const yMax = Math.ceil((rawMax + yPad) / 10) * 10;

    const xOfIdx = (i) => pad.left + (n > 1 ? (i / (n - 1)) * plotW : 0);
    const yOfDeg = (deg) => pad.top + ((yMax - deg) / Math.max(1, yMax - yMin)) * plotH;

    // Horizontal grid lines & Y ticks
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#64748b';
    ctx.font = '600 10.5px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    const ySteps = 5;
    for (let s = 0; s <= ySteps; s++) {
      const val = yMin + ((yMax - yMin) * s) / ySteps;
      const py = yOfDeg(val);
      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(pad.left + plotW, py);
      ctx.stroke();
      ctx.fillText(`${val.toFixed(0)}°`, pad.left - 7, py);
    }

    // Zero-degree baseline
    if (yMin <= 0 && yMax >= 0) {
      const yZero = yOfDeg(0);
      ctx.save();
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 1.2;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(pad.left, yZero);
      ctx.lineTo(pad.left + plotW, yZero);
      ctx.stroke();
      ctx.restore();
    }

    // Vertical time grid lines & X ticks
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const xTicks = 6;
    for (let k = 0; k <= xTicks; k++) {
      const tVal = (tMax * k) / xTicks;
      const px = pad.left + (plotW * k) / xTicks;
      ctx.beginPath();
      ctx.moveTo(px, pad.top);
      ctx.lineTo(px, pad.top + plotH);
      ctx.stroke();
      ctx.fillText(`${tVal.toFixed(1)}s`, px, pad.top + plotH + 7);
    }

    // Shaded area under Flexion/Extension curve
    const yZeroClamped = yOfDeg(Math.max(yMin, Math.min(yMax, 0)));
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(xOfIdx(0), yZeroClamped);
    for (let i = 0; i < n; i++) {
      ctx.lineTo(xOfIdx(i), yOfDeg(samples[i].fe || 0));
    }
    ctx.lineTo(xOfIdx(n - 1), yZeroClamped);
    ctx.closePath();
    const grad = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    grad.addColorStop(0, 'rgba(2, 132, 199, 0.20)');
    grad.addColorStop(1, 'rgba(2, 132, 199, 0.02)');
    ctx.fillStyle = grad;
    ctx.fill();
    ctx.restore();

    // Helper to stroke a channel curve
    const drawSeries = (key, color, width, dash = []) => {
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      if (dash.length) ctx.setLineDash(dash);
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const px = xOfIdx(i);
        const py = yOfDeg(Number(samples[i][key] ?? 0));
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.restore();
    };

    drawSeries('ps', '#f59e0b', 1.6);
    drawSeries('ru', '#10b981', 1.6);
    drawSeries('femur', '#ec4899', 2.0, [5, 3]);
    drawSeries('fe', '#0284c7', 2.5);

    // Peak Callout Markers (Peak Flex/Ext & Peak Femur Tilt)
    const drawPeakCallout = (idx, val, label, color) => {
      const px = xOfIdx(idx);
      const py = yOfDeg(val);
      ctx.save();
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(px, py, 4.2, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      ctx.font = '700 10px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      const txt = `${label}: ${val.toFixed(1)}°`;
      const tw = ctx.measureText(txt).width + 10;
      const bx = Math.max(pad.left + 4, Math.min(pad.left + plotW - tw - 4, px - tw / 2));
      const by = Math.max(pad.top + 3, py - 21);
      ctx.fillStyle = 'rgba(15, 23, 42, 0.86)';
      ctx.beginPath();
      ctx.roundRect(bx, by, tw, 16, 4);
      ctx.fill();
      ctx.fillStyle = '#ffffff';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(txt, bx + 5, by + 8);
      ctx.restore();
    };

    if (n > 2) {
      drawPeakCallout(st.fe.maxIdx, st.fe.max, 'Peak Flex', '#0284c7');
      if (Math.abs(st.femur.max) > 3) {
        drawPeakCallout(st.femur.maxIdx, st.femur.max, 'Peak Femur S1', '#ec4899');
      }
    }

    // Top Compact Legend Strip
    const legendItems = [
      { label: 'Flex/Ext (°)', color: '#0284c7' },
      { label: 'Femur Tilt S1 (°)', color: '#ec4899' },
      { label: 'Var/Val (°)', color: '#10b981' },
      { label: 'Int/Ext Rot (°)', color: '#f59e0b' }
    ];
    let lx = pad.left;
    ctx.font = '700 10.5px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    legendItems.forEach((item) => {
      ctx.fillStyle = item.color;
      ctx.fillRect(lx, 11, 12, 4);
      ctx.fillStyle = '#334155';
      ctx.fillText(item.label, lx + 16, 13);
      lx += ctx.measureText(item.label).width + 32;
    });

    // Vertical Playhead Scrubber Cursor
    const pIdx = Math.max(0, Math.min(n - 1, playheadIdx));
    const curX = xOfIdx(pIdx);
    const curSample = samples[pIdx];

    ctx.save();
    ctx.strokeStyle = isReplayingTrial ? '#e11d48' : '#0f172a';
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.moveTo(curX, pad.top);
    ctx.lineTo(curX, pad.top + plotH);
    ctx.stroke();

    // Intersection dots on playhead
    [
      { key: 'fe', color: '#0284c7' },
      { key: 'femur', color: '#ec4899' },
      { key: 'ru', color: '#10b981' },
      { key: 'ps', color: '#f59e0b' }
    ].forEach((ch) => {
      const cy = yOfDeg(Number(curSample[ch.key] ?? 0));
      ctx.fillStyle = ch.color;
      ctx.beginPath();
      ctx.arc(curX, cy, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.4;
      ctx.stroke();
    });
    ctx.restore();

    ctx.restore();
  }

  // --- Wire Up Trial Review & Playback Event Listeners ---
  if (trialSelect) {
    trialSelect.addEventListener('change', (e) => {
      selectTrialById(e.target.value, true);
    });
  }

  if (btnTrialDelete) {
    btnTrialDelete.addEventListener('click', () => {
      if (!savedTrials.length) return;
      savedTrials = savedTrials.filter((tr) => tr.id !== selectedTrialId);
      if (!savedTrials.length) {
        savedTrials = [generatePreSeededSampleTrial()];
      }
      persistTrialsToStorage();
      selectedTrialId = savedTrials[0].id;
      refreshTrialSelectorUI();
      selectTrialById(selectedTrialId, false);
      showStatusToast('Trial removed from history.', 2500);
    });
  }

  if (btnTrialPlay) {
    btnTrialPlay.addEventListener('click', () => {
      if (isTrialPlaying) pauseTrialPlayback();
      else startTrialPlayback();
    });
  }

  if (btnTrialStop) {
    btnTrialStop.addEventListener('click', () => {
      stopTrialReplay(true);
    });
  }

  document.querySelectorAll('.speed-btn[data-speed]').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.speed-btn[data-speed]').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      trialPlaybackSpeed = parseFloat(btn.getAttribute('data-speed')) || 1.0;
    });
  });

  if (trialScrubber) {
    trialScrubber.addEventListener('input', () => {
      if (isTrialPlaying) pauseTrialPlayback();
      const trial = getSelectedTrial();
      if (trial) {
        applyTrialSampleTo3DAndUI(trial, parseInt(trialScrubber.value, 10) || 0, true);
      }
    });
  }

  // Click & Drag Scrubbing Directly on #trial-review-chart
  if (trialReviewCanvas) {
    let isDraggingChart = false;
    const scrubFromPointerEvent = (ev) => {
      const trial = getSelectedTrial();
      if (!trial || !trial.samples || !trial.samples.length) return;
      const rect = trialReviewCanvas.getBoundingClientRect();
      const clientX = ev.touches && ev.touches.length ? ev.touches[0].clientX : ev.clientX;
      const relX = clientX - rect.left;
      const plotW = Math.max(80, rect.width - trialChartPad.left - trialChartPad.right);
      const frac = Math.max(0, Math.min(1, (relX - trialChartPad.left) / plotW));
      const idx = Math.round(frac * (trial.samples.length - 1));
      if (isTrialPlaying) pauseTrialPlayback();
      applyTrialSampleTo3DAndUI(trial, idx, true);
    };

    trialReviewCanvas.addEventListener('pointerdown', (ev) => {
      isDraggingChart = true;
      if (trialReviewCanvas.setPointerCapture) {
        try { trialReviewCanvas.setPointerCapture(ev.pointerId); } catch (e) {}
      }
      scrubFromPointerEvent(ev);
    });
    trialReviewCanvas.addEventListener('pointermove', (ev) => {
      if (!isDraggingChart) return;
      scrubFromPointerEvent(ev);
    });
    window.addEventListener('pointerup', () => {
      isDraggingChart = false;
    });
  }

  window.addEventListener('resize', () => {
    const trial = getSelectedTrial();
    if (trial) renderTrialReviewChart(trial, trialScrubIndex);
  });

  // ============================================================================
  // GRAPHICAL BIOMECHANICAL REPORT (PDF / HTML) & 1400x1600 HIGH-RES PNG POSTER
  // ============================================================================

  function resolveReportTrial(preferSelectedTrial = false) {
    if (!preferSelectedTrial && recordedData.length > 5) {
      const saved = saveRecordedTrial(recordedData);
      if (saved) return saved;
    }
    return getSelectedTrial() || generatePreSeededSampleTrial();
  }

  /**
   * Figure 1 (1100x360 PNG Canvas): 3D Relative Joint Angles vs. Time (Flex/Ext, Var/Val, Int/Ext)
   */
  function renderReportFigure1Canvas(trial) {
    const canvas = document.createElement('canvas');
    canvas.width = 1100;
    canvas.height = 360;
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    const pad = { left: 68, right: 34, top: 52, bottom: 48 };
    const plotW = W - pad.left - pad.right;
    const plotH = H - pad.top - pad.bottom;

    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(1, 1, W - 2, H - 2);

    const preset = getPreset(trial.joint || currentJoint);
    const samples = trial.samples || [];
    const n = samples.length;
    const st = trial.stats || computeTrialStats(samples);
    const t0 = samples[0]?.t || 0;
    const tMax = Math.max(0.1, (samples[n - 1]?.t || 1) - t0);

    // Figure Title & Legend
    ctx.fillStyle = '#0f172a';
    ctx.font = '800 15px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText(`Figure 1. 3D Relative ${preset.name} Joint Angles vs. Time (${preset.p1Short}, ${preset.p2Short}, ${preset.p3Short})`, pad.left, 24);

    const leg = [
      { text: `${preset.p1Short} (ROM ${st.fe.span.toFixed(1)}°)`, color: '#0284c7' },
      { text: `${preset.p2Short} (ROM ${st.ru.span.toFixed(1)}°)`, color: '#10b981' },
      { text: `${preset.p3Short} (ROM ${st.ps.span.toFixed(1)}°)`, color: '#f59e0b' }
    ];
    let lx = W - pad.right;
    ctx.font = '700 12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    for (let i = leg.length - 1; i >= 0; i--) {
      const tw = ctx.measureText(leg[i].text).width;
      lx -= tw;
      ctx.fillStyle = '#334155';
      ctx.fillText(leg[i].text, lx, 24);
      lx -= 18;
      ctx.fillStyle = leg[i].color;
      ctx.fillRect(lx, 16, 13, 5);
      lx -= 18;
    }

    const rawMin = Math.min(-15, st.fe.min, st.ru.min, st.ps.min);
    const rawMax = Math.max(45, st.fe.max, st.ru.max, st.ps.max);
    const margin = Math.max(10, (rawMax - rawMin) * 0.14);
    const yMin = Math.floor((rawMin - margin) / 10) * 10;
    const yMax = Math.ceil((rawMax + margin) / 10) * 10;

    const xOfIdx = (i) => pad.left + (n > 1 ? (i / (n - 1)) * plotW : 0);
    const yOfVal = (v) => pad.top + ((yMax - v) / Math.max(1, yMax - yMin)) * plotH;

    // Grid
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#475569';
    ctx.font = '600 11.5px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let s = 0; s <= 5; s++) {
      const v = yMin + ((yMax - yMin) * s) / 5;
      const py = yOfVal(v);
      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(pad.left + plotW, py);
      ctx.stroke();
      ctx.fillText(`${v.toFixed(0)}°`, pad.left - 8, py);
    }

    if (yMin <= 0 && yMax >= 0) {
      const y0 = yOfVal(0);
      ctx.save();
      ctx.strokeStyle = '#94a3b8';
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(pad.left, y0);
      ctx.lineTo(pad.left + plotW, y0);
      ctx.stroke();
      ctx.restore();
    }

    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let k = 0; k <= 10; k++) {
      const tVal = (tMax * k) / 10;
      const px = pad.left + (plotW * k) / 10;
      ctx.beginPath();
      ctx.moveTo(px, pad.top);
      ctx.lineTo(px, pad.top + plotH);
      ctx.stroke();
      ctx.fillText(`${tVal.toFixed(1)}s`, px, pad.top + plotH + 8);
    }

    ctx.fillStyle = '#334155';
    ctx.font = '700 12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillText('Elapsed Trial Time (seconds)', pad.left + plotW / 2, H - 18);

    // Shaded FE envelope
    if (n > 1) {
      const y0 = yOfVal(Math.max(yMin, Math.min(yMax, 0)));
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(xOfIdx(0), y0);
      for (let i = 0; i < n; i++) ctx.lineTo(xOfIdx(i), yOfVal(samples[i].fe || 0));
      ctx.lineTo(xOfIdx(n - 1), y0);
      ctx.closePath();
      ctx.fillStyle = 'rgba(2, 132, 199, 0.14)';
      ctx.fill();
      ctx.restore();
    }

    const strokeChannel = (key, color, lw) => {
      if (!n) return;
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = lw;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const px = xOfIdx(i);
        const py = yOfVal(Number(samples[i][key] ?? 0));
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.restore();
    };

    strokeChannel('ps', '#f59e0b', 2.0);
    strokeChannel('ru', '#10b981', 2.0);
    strokeChannel('fe', '#0284c7', 3.0);

    // Peak markers
    const markPeak = (idx, val, label, color) => {
      if (!n) return;
      const px = xOfIdx(idx);
      const py = yOfVal(val);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(px, py, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.8;
      ctx.stroke();

      const badgeText = `${label}: ${val.toFixed(1)}° @ ${st.fe.maxTime.toFixed(2)}s`;
      ctx.font = '700 11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      const bw = ctx.measureText(badgeText).width + 12;
      const bx = Math.max(pad.left + 4, Math.min(pad.left + plotW - bw - 4, px - bw / 2));
      const by = Math.max(pad.top + 4, py - 24);
      ctx.fillStyle = color;
      ctx.fillRect(bx, by, bw, 18);
      ctx.fillStyle = '#ffffff';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(badgeText, bx + 6, by + 9);
    };
    markPeak(st.fe.maxIdx, st.fe.max, `Max ${preset.p1Short}`, '#0284c7');

    return canvas;
  }

  /**
   * Figure 2 (1100x320 PNG Canvas): Proximal Femur Sagittal Inclination (S1 0x69) & Joint Angular Velocity vs. Time
   */
  function renderReportFigure2Canvas(trial) {
    const canvas = document.createElement('canvas');
    canvas.width = 1100;
    canvas.height = 320;
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    const pad = { left: 68, right: 68, top: 50, bottom: 44 };
    const plotW = W - pad.left - pad.right;
    const plotH = H - pad.top - pad.bottom;

    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(1, 1, W - 2, H - 2);

    const samples = trial.samples || [];
    const n = samples.length;
    const st = trial.stats || computeTrialStats(samples);
    const t0 = samples[0]?.t || 0;
    const tMax = Math.max(0.1, (samples[n - 1]?.t || 1) - t0);

    ctx.fillStyle = '#0f172a';
    ctx.font = '800 15px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText('Figure 2. Proximal Femur Sagittal Inclination (Sensor #1 0x69) & Joint Angular Velocity |ω| vs. Time', pad.left, 24);

    const yMinFem = Math.min(-10, Math.floor((st.femur.min - 6) / 10) * 10);
    const yMaxFem = Math.max(45, Math.ceil((st.femur.max + 10) / 10) * 10);
    const yMaxVel = Math.max(60, Math.ceil((st.peakJVel * 1.18) / 20) * 20);

    const xOfIdx = (i) => pad.left + (n > 1 ? (i / (n - 1)) * plotW : 0);
    const yOfFem = (v) => pad.top + ((yMaxFem - v) / Math.max(1, yMaxFem - yMinFem)) * plotH;
    const yOfVel = (v) => pad.top + ((yMaxVel - v) / Math.max(1, yMaxVel)) * plotH;

    // Grid & Left Y-Axis (Femur Tilt deg) + Right Y-Axis (Angular Velocity deg/s)
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    ctx.font = '700 11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textBaseline = 'middle';
    for (let s = 0; s <= 4; s++) {
      const femV = yMinFem + ((yMaxFem - yMinFem) * s) / 4;
      const velV = (yMaxVel * s) / 4;
      const py = yOfFem(femV);
      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(pad.left + plotW, py);
      ctx.stroke();

      ctx.fillStyle = '#db2777';
      ctx.textAlign = 'right';
      ctx.fillText(`${femV.toFixed(0)}°`, pad.left - 8, py);

      const pyVel = yOfVel(velV);
      ctx.fillStyle = '#7c3aed';
      ctx.textAlign = 'left';
      ctx.fillText(`${velV.toFixed(0)}°/s`, pad.left + plotW + 8, pyVel);
    }

    ctx.fillStyle = '#475569';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let k = 0; k <= 10; k++) {
      const tVal = (tMax * k) / 10;
      const px = pad.left + (plotW * k) / 10;
      ctx.beginPath();
      ctx.moveTo(px, pad.top);
      ctx.lineTo(px, pad.top + plotH);
      ctx.stroke();
      ctx.fillText(`${tVal.toFixed(1)}s`, px, pad.top + plotH + 8);
    }

    // Plot Joint Angular Velocity |w| (Right Axis, Purple)
    if (n > 1) {
      ctx.save();
      ctx.strokeStyle = '#8b5cf6';
      ctx.lineWidth = 1.9;
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const px = xOfIdx(i);
        const py = yOfVel(Number(samples[i].jVel || 0));
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.restore();

      // Plot Proximal Femur Sagittal Inclination S1 (Left Axis, Pink/Magenta)
      ctx.save();
      ctx.strokeStyle = '#ec4899';
      ctx.lineWidth = 2.8;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const px = xOfIdx(i);
        const py = yOfFem(Number(samples[i].femur ?? 0));
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.restore();

      // Peak Femur marker
      const pxF = xOfIdx(st.femur.maxIdx);
      const pyF = yOfFem(st.femur.max);
      ctx.fillStyle = '#db2777';
      ctx.beginPath();
      ctx.arc(pxF, pyF, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#db2777';
      ctx.font = '700 11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`Peak Femur S1: ${st.femur.max.toFixed(1)}°`, Math.min(pad.left + plotW - 70, Math.max(pad.left + 70, pxF)), Math.max(pad.top + 10, pyF - 14));
    }

    return canvas;
  }

  /**
   * Figure 3 (1100x340 PNG Canvas): Anatomical Range-of-Motion (ROM) Span & Mean +- SD Bar Chart
   */
  function renderReportFigure3Canvas(trial) {
    const canvas = document.createElement('canvas');
    canvas.width = 1100;
    canvas.height = 340;
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    const pad = { left: 72, right: 36, top: 54, bottom: 52 };
    const plotW = W - pad.left - pad.right;
    const plotH = H - pad.top - pad.bottom;

    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(1, 1, W - 2, H - 2);

    const preset = getPreset(trial.joint || currentJoint);
    const st = trial.stats || computeTrialStats(trial.samples || []);

    ctx.fillStyle = '#0f172a';
    ctx.font = '800 15px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText(`Figure 3. Anatomical Range-of-Motion (ROM) Span, Mean ± SD & Peak Angular Velocity Profile`, pad.left, 25);

    const bars = [
      { label: `${preset.p1Short} (Primary)`, stat: st.fe, color: '#0284c7', bg: 'rgba(2, 132, 199, 0.16)' },
      { label: 'Femur Tilt (Sensor #1 0x69)', stat: st.femur, color: '#ec4899', bg: 'rgba(236, 72, 153, 0.16)' },
      { label: `${preset.p2Short} (Frontal)`, stat: st.ru, color: '#10b981', bg: 'rgba(16, 185, 129, 0.16)' },
      { label: `${preset.p3Short} (Transverse)`, stat: st.ps, color: '#f59e0b', bg: 'rgba(245, 158, 11, 0.16)' }
    ];

    const yMax = Math.max(60, Math.ceil((Math.max(st.fe.span, st.femur.span, st.ru.span, st.ps.span) * 1.25) / 10) * 10);
    const yOfSpan = (v) => pad.top + ((yMax - v) / Math.max(1, yMax)) * plotH;

    // Grid
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#475569';
    ctx.font = '600 11.5px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let s = 0; s <= 5; s++) {
      const v = (yMax * s) / 5;
      const py = yOfSpan(v);
      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(pad.left + plotW, py);
      ctx.stroke();
      ctx.fillText(`${v.toFixed(0)}°`, pad.left - 8, py);
    }

    const colW = plotW / bars.length;
    const barW = Math.min(130, colW * 0.54);

    bars.forEach((b, idx) => {
      const cx = pad.left + colW * idx + colW / 2;
      const bx = cx - barW / 2;
      const by = yOfSpan(b.stat.span);
      const bh = Math.max(4, pad.top + plotH - by);

      ctx.fillStyle = b.bg;
      ctx.fillRect(bx, by, barW, bh);
      ctx.strokeStyle = b.color;
      ctx.lineWidth = 2.4;
      ctx.strokeRect(bx, by, barW, bh);

      // Value labels above bar
      ctx.fillStyle = b.color;
      ctx.font = '800 14px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'bottom';
      ctx.fillText(`ROM: ${b.stat.span.toFixed(1)}°`, cx, by - 18);

      ctx.fillStyle = '#334155';
      ctx.font = '600 11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.fillText(`[${b.stat.min.toFixed(1)}° to ${b.stat.max.toFixed(1)}°] • Mean ${b.stat.mean.toFixed(1)}±${b.stat.sd.toFixed(1)}°`, cx, by - 4);

      // X-axis category label
      ctx.fillStyle = '#0f172a';
      ctx.font = '700 12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.textBaseline = 'top';
      ctx.fillText(b.label, cx, pad.top + plotH + 10);
      ctx.fillStyle = '#64748b';
      ctx.font = '600 11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.fillText(`Peak Vel: ${b.stat.peakVel.toFixed(1)} °/s`, cx, pad.top + plotH + 27);
    });

    return canvas;
  }

  function captureTrial3DSnapshot(trial) {
    if (!joint3D || typeof joint3D.captureSnapshotDataURL !== 'function') return '';
    // Pose 3D model at current scrub index (or peak flexion index) for an informative 3D anatomical posture snapshot
    if (trial && trial.samples && trial.samples.length) {
      const snapIdx = trialScrubIndex > 0 ? trialScrubIndex : (trial.stats?.fe?.maxIdx || 0);
      const s = trial.samples[snapIdx] || trial.samples[0];
      const femRad = (Number(s.femur || 0) * Math.PI) / 180;
      const s1Snap = { ax: -9.81 * Math.sin(femRad), ay: 0, az: 9.81 * Math.cos(femRad), gx: 0, gy: 0, gz: 0 };
      const s2Snap = { ax: 0, ay: 0, az: 9.81, gx: 0, gy: 0, gz: 0 };
      joint3D.s1LocalTare.set(0, 0, 1);
      joint3D.hasTaredS1 = true;
      joint3D.s1TableYaw = 0;
      for (let i = 0; i < 3; i++) {
        joint3D.updateKinematics(s1Snap, s2Snap, s.fe, s.ru, s.ps, 95, trial.jointName || 'KNEE', true);
      }
    }
    return joint3D.captureSnapshotDataURL() || '';
  }

  async function generateAndExportGraphicalReport(preferSelectedTrial = true) {
    const trial = resolveReportTrial(preferSelectedTrial);
    if (!trial || !trial.samples || !trial.samples.length) {
      showStatusToast('No trial samples available for report generation.', 3000);
      return;
    }

    const st = trial.stats || computeTrialStats(trial.samples);
    const preset = getPreset(trial.joint || currentJoint);
    const fig1Url = renderReportFigure1Canvas(trial).toDataURL('image/png');
    const fig2Url = renderReportFigure2Canvas(trial).toDataURL('image/png');
    const fig3Url = renderReportFigure3Canvas(trial).toDataURL('image/png');
    const snap3dUrl = captureTrial3DSnapshot(trial);
    const dateStr = new Date(trial.createdAt || Date.now()).toLocaleString();

    const htmlContent = `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>LTU EBL Biomechanical Kinematic Report — ${trial.title}</title>
  <style>
    * { box-sizing: border-box; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      color: #0f172a;
      background: #f1f5f9;
      margin: 0;
      padding: 0;
    }
    .print-toolbar {
      position: sticky;
      top: 0;
      z-index: 100;
      background: #0f172a;
      color: #ffffff;
      padding: 0.75rem 1.5rem;
      display: flex;
      justify-content: space-between;
      align-items: center;
      box-shadow: 0 4px 12px rgba(0,0,0,0.18);
    }
    .print-btn {
      background: #0284c7;
      color: #ffffff;
      border: none;
      padding: 0.55rem 1.15rem;
      border-radius: 8px;
      font-weight: 800;
      font-size: 0.92rem;
      cursor: pointer;
    }
    .report-sheet {
      max-width: 1140px;
      margin: 1.5rem auto;
      background: #ffffff;
      padding: 2rem 2.2rem;
      border-radius: 14px;
      box-shadow: 0 8px 28px rgba(15,23,42,0.08);
    }
    .report-header {
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
      border-bottom: 3px solid #0284c7;
      padding-bottom: 1rem;
      margin-bottom: 1.25rem;
    }
    .lab-title { font-size: 1.45rem; font-weight: 800; color: #0f172a; margin: 0 0 0.25rem 0; }
    .lab-sub { font-size: 0.9rem; color: #475569; margin: 0; font-weight: 600; }
    .meta-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 0.75rem;
      background: #f8fafc;
      border: 1px solid #e2e8f0;
      border-radius: 10px;
      padding: 0.85rem 1rem;
      margin-bottom: 1.35rem;
      font-size: 0.84rem;
    }
    .meta-item span { display: block; color: #64748b; font-size: 0.74rem; text-transform: uppercase; font-weight: 700; }
    .meta-item strong { color: #0f172a; font-size: 0.95rem; }
    h2.sec-title {
      font-size: 1.05rem;
      font-weight: 800;
      color: #0f172a;
      margin: 1.25rem 0 0.6rem 0;
      border-left: 4px solid #0284c7;
      padding-left: 0.6rem;
    }
    table.kin-table {
      width: 100%;
      border-collapse: collapse;
      margin-bottom: 1.35rem;
      font-size: 0.88rem;
    }
    table.kin-table th, table.kin-table td {
      border: 1px solid #cbd5e1;
      padding: 0.55rem 0.75rem;
      text-align: right;
    }
    table.kin-table th:first-child, table.kin-table td:first-child { text-align: left; font-weight: 700; }
    table.kin-table thead { background: #0f172a; color: #ffffff; }
    table.kin-table tbody tr:nth-child(even) { background: #f8fafc; }
    .fig-card {
      margin-bottom: 1.25rem;
      border: 1px solid #e2e8f0;
      border-radius: 10px;
      padding: 0.6rem;
      background: #ffffff;
      page-break-inside: avoid;
    }
    .fig-card img { width: 100%; height: auto; display: block; border-radius: 6px; }
    .fig-row-split {
      display: grid;
      grid-template-columns: 2.15fr 1fr;
      gap: 1rem;
      align-items: stretch;
      page-break-inside: avoid;
    }
    .snap-box {
      border: 1px solid #cbd5e1;
      border-radius: 10px;
      background: #0b1221;
      color: #f8fafc;
      padding: 0.65rem;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      text-align: center;
    }
    .snap-box img { width: 100%; max-height: 270px; object-fit: contain; border-radius: 6px; }
    .snap-caption { font-size: 0.78rem; color: #cbd5e1; margin-top: 0.4rem; font-weight: 600; }
    @media print {
      body { background: #ffffff; }
      .print-toolbar { display: none !important; }
      .report-sheet { box-shadow: none; margin: 0; padding: 0.5rem; max-width: 100%; }
    }
  </style>
</head>
<body>
  <div class="print-toolbar">
    <div>
      <strong>LTU Experimental Biomechanics Laboratory (EBL)</strong> &mdash; Graphical Kinematic Report
    </div>
    <button class="print-btn" onclick="window.print()">&#x1F5A8;&#xFE0F; Print / Save as PDF</button>
  </div>

  <div class="report-sheet">
    <div class="report-header">
      <div>
        <h1 class="lab-title">LTU Experimental Biomechanics Laboratory (EBL)</h1>
        <p class="lab-sub">Seeed XIAO nRF52840 1.14" IPS Display &bull; Dual Adafruit ICM-20948 9-DOF IMU Kinematic Report</p>
      </div>
      <div style="text-align:right; font-size:0.84rem; color:#475569;">
        <div><strong>Generated:</strong> ${dateStr}</div>
        <div><strong>Sensors:</strong> S1 Thigh (0x69) &rarr; S2 Shank (0x68)</div>
      </div>
    </div>

    <div class="meta-grid">
      <div class="meta-item"><span>Trial Identifier</span><strong>${trial.title}</strong></div>
      <div class="meta-item"><span>Anatomical Joint</span><strong>${preset.name} (${preset.segments})</strong></div>
      <div class="meta-item"><span>Duration &amp; Rate</span><strong>${st.durationSec.toFixed(2)} s @ ${st.avgHz} Hz (${st.sampleCount} pts)</strong></div>
      <div class="meta-item"><span>Detected Cycles &amp; Peak |&omega;|</span><strong>${st.reps} Reps &bull; ${st.peakJVel.toFixed(1)} &deg;/s</strong></div>
    </div>

    <h2 class="sec-title">1. Biomechanical Kinematic Summary Table (3D Relative Joint Angles &amp; Proximal Femur Inclination)</h2>
    <table class="kin-table">
      <thead>
        <tr>
          <th>Anatomical Degree of Freedom / Segment</th>
          <th>Min (&deg;)</th>
          <th>Max (&deg;)</th>
          <th>ROM Span (&deg;)</th>
          <th>Mean &plusmn; SD (&deg;)</th>
          <th>Peak Ang Vel (&deg;/s)</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td style="color:#0284c7;">${preset.p1Full}</td>
          <td>${st.fe.min.toFixed(1)}&deg;</td>
          <td>${st.fe.max.toFixed(1)}&deg;</td>
          <td><strong>${st.fe.span.toFixed(1)}&deg;</strong></td>
          <td>${st.fe.mean.toFixed(1)}&deg; &plusmn; ${st.fe.sd.toFixed(1)}&deg;</td>
          <td>${st.fe.peakVel.toFixed(1)} &deg;/s</td>
        </tr>
        <tr>
          <td style="color:#10b981;">${preset.p2Full}</td>
          <td>${st.ru.min.toFixed(1)}&deg;</td>
          <td>${st.ru.max.toFixed(1)}&deg;</td>
          <td><strong>${st.ru.span.toFixed(1)}&deg;</strong></td>
          <td>${st.ru.mean.toFixed(1)}&deg; &plusmn; ${st.ru.sd.toFixed(1)}&deg;</td>
          <td>${st.ru.peakVel.toFixed(1)} &deg;/s</td>
        </tr>
        <tr>
          <td style="color:#d97706;">${preset.p3Full}</td>
          <td>${st.ps.min.toFixed(1)}&deg;</td>
          <td>${st.ps.max.toFixed(1)}&deg;</td>
          <td><strong>${st.ps.span.toFixed(1)}&deg;</strong></td>
          <td>${st.ps.mean.toFixed(1)}&deg; &plusmn; ${st.ps.sd.toFixed(1)}&deg;</td>
          <td>${st.ps.peakVel.toFixed(1)} &deg;/s</td>
        </tr>
        <tr>
          <td style="color:#db2777;">Proximal Femur Sagittal Inclination (Sensor #1 0x69)</td>
          <td>${st.femur.min.toFixed(1)}&deg;</td>
          <td>${st.femur.max.toFixed(1)}&deg;</td>
          <td><strong>${st.femur.span.toFixed(1)}&deg;</strong></td>
          <td>${st.femur.mean.toFixed(1)}&deg; &plusmn; ${st.femur.sd.toFixed(1)}&deg;</td>
          <td>${st.femur.peakVel.toFixed(1)} &deg;/s</td>
        </tr>
      </tbody>
    </table>

    <h2 class="sec-title">2. Multi-Axis Time-Series Waveforms &amp; Segment Inclination</h2>
    <div class="fig-card">
      <img src="${fig1Url}" alt="Figure 1: 3D Relative Joint Angles vs Time">
    </div>
    <div class="fig-card">
      <img src="${fig2Url}" alt="Figure 2: Proximal Femur Sagittal Inclination and Joint Angular Velocity">
    </div>

    <h2 class="sec-title">3. Anatomical Range-of-Motion (ROM) Profile &amp; 3D Joint Posture Snapshot</h2>
    <div class="fig-row-split">
      <div class="fig-card" style="margin-bottom:0;">
        <img src="${fig3Url}" alt="Figure 3: Range of Motion Bar Chart">
      </div>
      <div class="snap-box">
        <div style="font-size:0.82rem; font-weight:800; color:#38bdf8;">3D Anatomical Joint Posture</div>
        ${snap3dUrl ? `<img src="${snap3dUrl}" alt="3D Anatomical Posture Snapshot">` : `<div style="padding:3rem 1rem;color:#94a3b8;">3D Viewport Snapshot</div>`}
        <div class="snap-caption">Femur S1 (0x69) + Tibia S2 (0x68) &bull; Peak Flexion ${st.fe.max.toFixed(1)}&deg;</div>
      </div>
    </div>
  </div>
</body>
</html>`;

    // 1. Open interactive printable report in new tab
    try {
      const win = window.open('', '_blank');
      if (win && win.document) {
        win.document.open();
        win.document.write(htmlContent);
        win.document.close();
      }
    } catch (e) {
      console.warn('Popup blocked when opening report tab:', e);
    }

    // 2. Also download self-contained HTML report file
    const filename = `LTU_EBL_Kinematic_Report_${preset.name}_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.html`;
    const blob = new Blob([htmlContent], { type: 'text/html;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    showStatusToast('✓ Generated Graphical Kinematic Report (Opened & Downloaded)', 4000);
  }

  /**
   * High-Resolution 1400x1600 Graphical Analysis Poster (.PNG)
   */
  async function generateAndExportGraphicalPngPoster(preferSelectedTrial = true) {
    const trial = resolveReportTrial(preferSelectedTrial);
    if (!trial || !trial.samples || !trial.samples.length) {
      showStatusToast('No trial samples available for PNG poster.', 3000);
      return;
    }

    const st = trial.stats || computeTrialStats(trial.samples);
    const preset = getPreset(trial.joint || currentJoint);
    const fig1Canvas = renderReportFigure1Canvas(trial);
    const fig2Canvas = renderReportFigure2Canvas(trial);
    const fig3Canvas = renderReportFigure3Canvas(trial);
    const snap3dUrl = captureTrial3DSnapshot(trial);

    const poster = document.createElement('canvas');
    poster.width = 1400;
    poster.height = 1600;
    const ctx = poster.getContext('2d');

    // Background
    ctx.fillStyle = '#f8fafc';
    ctx.fillRect(0, 0, 1400, 1600);

    // Top Header Banner
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, 1400, 118);
    ctx.fillStyle = '#0284c7';
    ctx.fillRect(0, 114, 1400, 5);

    ctx.fillStyle = '#ffffff';
    ctx.font = '800 26px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText('LTU Experimental Biomechanics Laboratory — 3D Dual-IMU Kinematic Analysis', 36, 46);

    ctx.fillStyle = '#38bdf8';
    ctx.font = '700 16px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillText(`${trial.title}  •  Joint: ${preset.name} (${preset.segments})`, 36, 78);

    ctx.fillStyle = '#cbd5e1';
    ctx.font = '600 13.5px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillText(`Duration: ${st.durationSec.toFixed(2)}s @ ${st.avgHz} Hz (${st.sampleCount} samples)   |   Detected Cycles: ${st.reps} Reps   |   Peak Joint |ω|: ${st.peakJVel.toFixed(1)} °/s`, 36, 102);

    // Top-Left Kinematic Summary Table Card (x=36, y=136, w=880, h=295)
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(36, 136, 880, 295);
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(36, 136, 880, 295);

    ctx.fillStyle = '#0f172a';
    ctx.font = '800 16px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillText('Biomechanical Kinematic Summary Table (Standing Neutral 0° Tare)', 54, 166);

    const headers = ['Anatomical Axis / Segment', 'Min (°)', 'Max (°)', 'ROM Span (°)', 'Mean ± SD (°)', 'Peak Vel (°/s)'];
    const colX = [54, 415, 505, 615, 755, 895];
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(50, 180, 850, 34);
    ctx.fillStyle = '#ffffff';
    ctx.font = '700 13px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    headers.forEach((h, idx) => {
      ctx.textAlign = idx === 0 ? 'left' : 'right';
      ctx.fillText(h, colX[idx], 202);
    });

    const tableRows = [
      { name: preset.p1Full, st: st.fe, color: '#0284c7' },
      { name: preset.p2Full, st: st.ru, color: '#10b981' },
      { name: preset.p3Full, st: st.ps, color: '#d97706' },
      { name: 'Proximal Femur Sagittal Tilt (S1 0x69)', st: st.femur, color: '#db2777' }
    ];

    tableRows.forEach((r, idx) => {
      const ry = 214 + idx * 50;
      ctx.fillStyle = idx % 2 === 1 ? '#f8fafc' : '#ffffff';
      ctx.fillRect(50, ry, 850, 50);
      ctx.strokeStyle = '#e2e8f0';
      ctx.strokeRect(50, ry, 850, 50);

      ctx.fillStyle = r.color;
      ctx.font = '700 13.5px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.textAlign = 'left';
      ctx.fillText(r.name, colX[0], ry + 30);

      ctx.fillStyle = '#0f172a';
      ctx.font = '600 13.5px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.textAlign = 'right';
      ctx.fillText(`${r.st.min.toFixed(1)}°`, colX[1], ry + 30);
      ctx.fillText(`${r.st.max.toFixed(1)}°`, colX[2], ry + 30);
      ctx.font = '800 14px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.fillText(`${r.st.span.toFixed(1)}°`, colX[3], ry + 30);
      ctx.font = '600 13.5px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
      ctx.fillText(`${r.st.mean.toFixed(1)} ± ${r.st.sd.toFixed(1)}°`, colX[4], ry + 30);
      ctx.fillText(`${r.st.peakVel.toFixed(1)} °/s`, colX[5], ry + 30);
    });

    // Top-Right 3D Anatomical Snapshot Card (x=936, y=136, w=428, h=295)
    ctx.fillStyle = '#0b1221';
    ctx.fillRect(936, 136, 428, 295);
    ctx.strokeStyle = '#1e293b';
    ctx.strokeRect(936, 136, 428, 295);
    ctx.fillStyle = '#38bdf8';
    ctx.font = '800 14px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('3D Anatomical Dual-IMU Posture Snapshot', 936 + 214, 160);

    if (snap3dUrl) {
      await new Promise((resolve) => {
        const img = new Image();
        img.onload = () => {
          const availW = 408;
          const availH = 235;
          const scale = Math.min(availW / img.width, availH / img.height);
          const dw = img.width * scale;
          const dh = img.height * scale;
          ctx.drawImage(img, 936 + (428 - dw) / 2, 172 + (availH - dh) / 2, dw, dh);
          resolve();
        };
        img.onerror = resolve;
        img.src = snap3dUrl;
      });
    }
    ctx.fillStyle = '#cbd5e1';
    ctx.font = '600 12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillText(`Peak Flexion: ${st.fe.max.toFixed(1)}°  •  Peak Femur S1: ${st.femur.max.toFixed(1)}°`, 936 + 214, 420);

    // Draw Figures 1, 2, 3 full-width (x=36, w=1328)
    ctx.drawImage(fig1Canvas, 36, 448, 1328, 380);
    ctx.drawImage(fig2Canvas, 36, 842, 1328, 345);
    ctx.drawImage(fig3Canvas, 36, 1201, 1328, 355);

    // Footer
    ctx.fillStyle = '#64748b';
    ctx.font = '600 12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`LTU Experimental Biomechanics Laboratory (EBL) • Seeed XIAO nRF52840 Plus + Dual ICM-20948 Wearable Telemetry Suite • ${new Date().toLocaleString()}`, 700, 1582);

    const filename = `LTU_EBL_Kinematic_Poster_${preset.name}_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.png`;

    const blob = await new Promise((resolve) => poster.toBlob(resolve, 'image/png'));
    if (!blob) return;

    try {
      if (isIOS && navigator.canShare && navigator.share) {
        const file = new File([blob], filename, { type: 'image/png' });
        if (navigator.canShare({ files: [file] })) {
          await navigator.share({
            files: [file],
            title: 'LTU EBL 3D Joint Kinematic Analysis Poster'
          });
          return;
        }
      }
    } catch (err) {
      if (err.name === 'AbortError') return;
    }

    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    showStatusToast(`✓ Saved High-Res 1400x1600 Graphical Poster (${filename})`, 4000);
  }

  if (btnExportReport) {
    btnExportReport.addEventListener('click', () => generateAndExportGraphicalReport(false));
  }
  if (btnTrialExportReport) {
    btnTrialExportReport.addEventListener('click', () => generateAndExportGraphicalReport(true));
  }
  if (btnExportPng) {
    btnExportPng.addEventListener('click', () => generateAndExportGraphicalPngPoster(false));
  }
  if (btnTrialExportPng) {
    btnTrialExportPng.addEventListener('click', () => generateAndExportGraphicalPngPoster(true));
  }
  if (btnTrialExportCsv) {
    btnTrialExportCsv.addEventListener('click', () => {
      const trial = getSelectedTrial();
      if (trial && trial.samples) {
        exportSamplesToCsv(trial.samples, trial.id);
      }
    });
  }

  // --- Help Modal ---
  if (btnOpenHelp && modalHelp) {
    btnOpenHelp.addEventListener('click', () => modalHelp.classList.add('open'));
  }
  if (btnCloseHelp && modalHelp) {
    btnCloseHelp.addEventListener('click', () => modalHelp.classList.remove('open'));
  }
  if (modalHelp) {
    modalHelp.addEventListener('click', (e) => {
      if (e.target === modalHelp) modalHelp.classList.remove('open');
    });
  }

  // --- Initialize Default Joint Preset & Pre-Seeded Trial History on Page Load ---
  applyJointPreset('knee', false);
  loadSavedTrialsFromStorage();
});
