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

  // Real-Time Biomechanical Range of Motion (ROM) & Cycle Analytics
  const romStats = {
    fe: { min: Infinity, max: -Infinity, sum: 0, count: 0, peakVel: 0 },
    ru: { min: Infinity, max: -Infinity, sum: 0, count: 0, peakVel: 0 },
    ps: { min: Infinity, max: -Infinity, sum: 0, count: 0, peakVel: 0 },
    reps: 0,
    inFlexCycle: false
  };
  let prevAngles = { fe: 0, ru: 0, ps: 0, t: 0 };

  // Real-Time Chart Buffers
  let activeChartTab = 'angles'; // 'angles' | 'gyro' | 'accel'
  let timeWindowSec = 10;
  const maxPoints = 400;
  const buf = {
    t: [],
    fe: [], ru: [], ps: [], tot: [],
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
  const btnToggleStream = document.getElementById('btn-toggle-stream');
  const btnDumpMem = document.getElementById('btn-dump-mem');
  const btnOpenHelp = document.getElementById('btn-open-help');
  const btnCloseHelp = document.getElementById('btn-close-help');
  const modalHelp = document.getElementById('modal-help');

  // 3D HUD & Virtual XIAO Buttons
  const btnHudCal = document.getElementById('btn-hud-cal');
  const btnToggleAxes = document.getElementById('btn-toggle-axes');
  const btnResetCam = document.getElementById('btn-reset-cam');
  const btnHwScreen = document.getElementById('btn-hw-screen');
  const btnHwCal = document.getElementById('btn-hw-cal');
  const btnHwStream = document.getElementById('btn-hw-stream');
  const btnHwRec = document.getElementById('btn-hw-rec');

  // Live Readouts
  const valTime = document.getElementById('val-time');
  const valFe = document.getElementById('val-fe');
  const valRu = document.getElementById('val-ru');
  const valPs = document.getElementById('val-ps');
  const valTotAngle = document.getElementById('val-tot-angle');
  const valAngVel = document.getElementById('val-ang-vel');
  const valRepsTop = document.getElementById('val-reps-top');
  const valS1Amag = document.getElementById('val-s1-amag');
  const valS2Amag = document.getElementById('val-s2-amag');
  const valBatt = document.getElementById('val-batt');
  const valBattPct = document.getElementById('val-batt-pct');
  const valHz = document.getElementById('val-hz');

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
    ['fe', 'ru', 'ps'].forEach((k) => {
      romStats[k].min = Infinity;
      romStats[k].max = -Infinity;
      romStats[k].sum = 0;
      romStats[k].count = 0;
      romStats[k].peakVel = 0;
      document.getElementById(`rom-min-${k}`).textContent = '0.0';
      document.getElementById(`rom-max-${k}`).textContent = '0.0';
      document.getElementById(`rom-span-${k}`).textContent = '0.0';
      document.getElementById(`rom-mean-${k}`).textContent = '0.0';
      document.getElementById(`rom-pkv-${k}`).textContent = '0.0';
    });
    romStats.reps = 0;
    romStats.inFlexCycle = false;
    if (repCounterBadge) repCounterBadge.textContent = 'Reps: 0';
    if (valRepsTop) valRepsTop.textContent = '0';
  }
  if (btnResetRom) btnResetRom.addEventListener('click', resetRomAnalytics);

  // --- Update ROM Table ---
  function updateRomStats(fe, ru, ps, dt) {
    const vals = { fe, ru, ps };
    const vels = {
      fe: dt > 0 ? Math.abs(fe - prevAngles.fe) / dt : 0,
      ru: dt > 0 ? Math.abs(ru - prevAngles.ru) / dt : 0,
      ps: dt > 0 ? Math.abs(ps - prevAngles.ps) / dt : 0
    };

    ['fe', 'ru', 'ps'].forEach((k) => {
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

      document.getElementById(`rom-cur-${k}`).textContent = v.toFixed(1);
      document.getElementById(`rom-min-${k}`).textContent = st.min.toFixed(1);
      document.getElementById(`rom-max-${k}`).textContent = st.max.toFixed(1);
      document.getElementById(`rom-span-${k}`).textContent = span.toFixed(1);
      document.getElementById(`rom-mean-${k}`).textContent = mean.toFixed(1);
      document.getElementById(`rom-pkv-${k}`).textContent = st.peakVel.toFixed(1);
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
    if (!isStreaming) return;
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

    // 1. Update 3D Anatomical Joint Model
    joint3D.updateKinematics(s1, s2, fe, ru, ps, battPct, getPreset(currentJoint).name, isCalibrated);

    // 2. Update Live Angle Cards & Top Metrics
    valTime.textContent = t.toFixed(2);
    valFe.textContent = fe.toFixed(1);
    valRu.textContent = ru.toFixed(1);
    valPs.textContent = ps.toFixed(1);
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
    updateRomStats(fe, ru, ps, dt);
    prevAngles = { fe, ru, ps, t };

    // 6. Push to Real-Time Chart Buffers
    if (!isChartPaused) {
      buf.t.push(t.toFixed(2));
      buf.fe.push(fe);
      buf.ru.push(ru);
      buf.ps.push(ps);
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
        const recRow = {
          iso: new Date().toISOString(),
          t: parseFloat(parts[2]),
          joint: getPreset(currentJoint).name,
          fe: parseFloat(parts[3]),
          ru: parseFloat(parts[4]),
          ps: parseFloat(parts[5]),
          tot: Math.hypot(parseFloat(parts[3]), parseFloat(parts[4]), parseFloat(parts[5])),
          jVel: Math.abs(parseFloat(parts[9]) - parseFloat(parts[8])),
          ax1: 0, ay1: 0, az1: parseFloat(parts[6]),
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
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${r.t.toFixed(3)}</td>
      <td style="color:var(--plane-fe);">${r.fe.toFixed(1)}</td>
      <td style="color:var(--plane-ru);">${r.ru.toFixed(1)}</td>
      <td style="color:var(--plane-ps);">${r.ps.toFixed(1)}</td>
      <td>${r.a1Mag.toFixed(2)}</td>
      <td>${r.a2Mag.toFixed(2)}</td>
      <td>${r.g1Mag.toFixed(1)}</td>
      <td>${r.g2Mag.toFixed(1)}</td>
    `;
    recordTableBody.insertBefore(tr, recordTableBody.firstChild);
    while (recordTableBody.children.length > 60) {
      recordTableBody.removeChild(recordTableBody.lastChild);
    }
  }

  function setRecordingState(active, notifyBoard = true) {
    isRecording = active;
    if (isRecording) {
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
      showStatusToast(`■ Recording Stopped (${recordedData.length} samples ready to Export / Share)`, 4000);
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
      btnExport.disabled = true;
      recordTableBody.innerHTML = `<tr><td colspan="8" style="text-align:center; color:#94a3b8; padding:1.1rem;">Buffer cleared.</td></tr>`;
    });
  }

  // --- Export / Share CSV (Supports Native iPhone iOS Share Sheet + Standard Download) ---
  btnExport.addEventListener('click', async () => {
    if (!recordedData.length) return;
    const headers = [
      'Timestamp_ISO', 'Elapsed_s', 'Joint_Preset',
      'FlexExt_deg', 'VarusValgus_AbAd_deg', 'IntExtRot_ProSup_deg', 'Total3DAngle_deg', 'JointAngVel_dps',
      'S1_Ax_ms2', 'S1_Ay_ms2', 'S1_Az_ms2', 'S1_Gx_dps', 'S1_Gy_dps', 'S1_Gz_dps', 'S1_Mx_uT', 'S1_My_uT', 'S1_Mz_uT',
      'S2_Ax_ms2', 'S2_Ay_ms2', 'S2_Az_ms2', 'S2_Gx_dps', 'S2_Gy_dps', 'S2_Gz_dps', 'S2_Mx_uT', 'S2_My_uT', 'S2_Mz_uT',
      'Battery_V'
    ];
    const rows = recordedData.map((r) => [
      r.iso, r.t, r.joint,
      r.fe, r.ru, r.ps, r.tot, r.jVel,
      r.ax1, r.ay1, r.az1, r.gx1, r.gy1, r.gz1, r.mx1, r.my1, r.mz1,
      r.ax2, r.ay2, r.az2, r.gx2, r.gy2, r.gz2, r.mx2, r.my2, r.mz2,
      r.vbat
    ].join(','));
    const csv = [headers.join(','), ...rows].join('\n');
    const filename = `XIAO_2IMU_${getPreset(currentJoint).name}_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`;

    // Try iPhone / iOS Native Share Sheet first if supported
    try {
      if (navigator.canShare && navigator.share) {
        const csvFile = new File([csv], filename, { type: 'text/csv' });
        if (navigator.canShare({ files: [csvFile] })) {
          await navigator.share({
            files: [csvFile],
            title: 'XIAO Dual-IMU 3D Joint Kinematics CSV',
            text: `Recorded ${recordedData.length} samples (${getPreset(currentJoint).name})`
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
    isDemoMode = true;
    demoStart = performance.now();
    statusPill.className = 'status-pill demo';
    statusText.textContent = '3D Demo (60 Hz)';
    btnDemo.textContent = 'Stop Demo';
    btnDemo.classList.add('btn-danger');
    appendTerminalLine('▶ Started 3D Knee Flexion/Extension Biomechanical Simulation @ 60 Hz', 'info');

    demoTimer = setInterval(() => {
      const t = (performance.now() - demoStart) / 1000;

      // Simulate Proximal Thigh (Sensor #1 @ 0x69) natural movement during squat / gait flexion
      const s1Pitch = 10 * Math.sin(t * 1.4);
      const s1Roll = 3 * Math.cos(t * 0.7);
      const ax1 = -9.81 * Math.sin(s1Pitch * Math.PI / 180);
      const ay1 = 9.81 * Math.sin(s1Roll * Math.PI / 180);
      const az1 = 9.81 * Math.cos(s1Pitch * Math.PI / 180) * Math.cos(s1Roll * Math.PI / 180);
      const gx1 = 14.0 * Math.cos(t * 1.4);
      const gy1 = -2.1 * Math.sin(t * 0.7);
      const gz1 = 1.5 * Math.sin(t * 0.5);

      // Simulate Distal Lower Leg / Shank (Sensor #2 @ 0x68):
      // Anatomical Knee Flexion (0° standing extension -> 95° knee flexion)
      const rawFe = currentJoint === 'knee'
        ? 47.5 * (1 - Math.cos(t * 1.6))       // 0° to 95° realistic knee flexion cycle
        : 42.0 * Math.sin(t * 1.6);
      const rawRu = 6.5 * Math.sin(t * 1.6);   // Frontal plane Varus/Valgus excursion (+-6.5°)
      const rawPs = 11.0 * Math.sin(t * 0.8);  // Transverse Tibial Int/Ext rotation (+-11°)

      const totalPitch2 = s1Pitch - rawFe;
      const totalRoll2 = s1Roll + rawRu;
      const ax2 = -9.81 * Math.sin(totalPitch2 * Math.PI / 180);
      const ay2 = 9.81 * Math.sin(totalRoll2 * Math.PI / 180);
      const az2 = 9.81 * Math.cos(totalPitch2 * Math.PI / 180) * Math.cos(totalRoll2 * Math.PI / 180);
      const gx2 = gx1 + 47.5 * 1.6 * Math.sin(t * 1.6);
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

  // --- Initialize Default Joint Preset to Knee Joint on Page Load ---
  applyJointPreset('knee', false);
});
