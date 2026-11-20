# SPDX-FileCopyrightText: 2026 Eric G. Meyer / LTU Experimental Biomechanics Lab (EBL)
# SPDX-License-Identifier: MIT
"""
Seeed XIAO 1.14" IPS Display (nRF52840) + Dual ICM-20948 9-DOF IMUs (Simple Reference Edition)
- Proximal Sensor #1 (Thigh / Forearm): I2C 0x69 | Distal Sensor #2 (Shank / Hand): I2C 0x68
- Lateral Mounting: Z = Flex/Ext Hinge Axis, Y = Varus/Valgus (AP), X = Bone Shaft (Axial Rot)
- Auto-calibrates neutral standing pose (0, 0, 0 deg) at boot and immediately before recording.
"""
import sys, time, math, gc, board, busio, digitalio, analogio, microcontroller, supervisor
import displayio, terminalio
try: from fourwire import FourWire
except ImportError: from displayio import FourWire
from adafruit_display_text import bitmap_label as label
from adafruit_st7789 import ST7789

# --- 1. RGB LED, Battery Monitor & Physical Buttons (BTN_A=D6, BTN_B=D7, BTN_C=P1_07) ---
leds = [digitalio.DigitalInOut(p) for p in (board.LED_RED, board.LED_GREEN, board.LED_BLUE)]
for d in leds: d.direction, d.value = digitalio.Direction.OUTPUT, True
def set_led(r, g, b): leds[0].value, leds[1].value, leds[2].value = not r, not g, not b
set_led(False, False, True)
vbat_en = digitalio.DigitalInOut(getattr(board, "READ_BATT_ENABLE", microcontroller.pin.P0_14))
vbat_en.direction, vbat_en.value = digitalio.Direction.OUTPUT, False
vbat_adc = analogio.AnalogIn(getattr(board, "VOLTAGE_MONITOR", microcontroller.pin.P0_31))
def read_vbat(): return (vbat_adc.value / 65535.0) * 3.3 * 2.961
def make_button(pin):
    b = digitalio.DigitalInOut(pin); b.direction, b.pull = digitalio.Direction.INPUT, digitalio.Pull.UP
    return b
btn_a, btn_b, btn_c = make_button(board.D6), make_button(board.D7), make_button(microcontroller.pin.P1_07)
prev_a = prev_b = prev_c = False

# --- 2. 1.14" ST7789 IPS Display in Landscape Mode (240x135) ---
display = None
try:
    displayio.release_displays()
    bl = digitalio.DigitalInOut(microcontroller.pin.P1_05); bl.direction, bl.value = digitalio.Direction.OUTPUT, True
    bus = FourWire(busio.SPI(clock=board.SCK, MOSI=board.MOSI), command=board.D3, chip_select=board.D2, reset=microcontroller.pin.P1_03, baudrate=24000000)
    display = ST7789(bus, width=240, height=135, rowstart=40, colstart=53, rotation=90, auto_refresh=False)
    grp = displayio.Group()
    lbl_hdr = label.Label(terminalio.FONT, text="LTU EBL | 3D JOINT [CAL]", color=0xFFFFFF, x=6, y=10, scale=1)
    lbl_fe  = label.Label(terminalio.FONT, text="Flx/Ext:  +0.0 d", color=0x38BDF8, x=8, y=38, scale=2)
    lbl_ru  = label.Label(terminalio.FONT, text="Var/Val:  +0.0 d", color=0x34D399, x=8, y=68, scale=2)
    lbl_ps  = label.Label(terminalio.FONT, text="Int/Ext:  +0.0 d", color=0xFBBF24, x=8, y=98, scale=2)
    lbl_ftr = label.Label(terminalio.FONT, text="IDLE | USB/ADV | 4.10V", color=0x94A3B8, x=6, y=125, scale=1)
    for item in (lbl_hdr, lbl_fe, lbl_ru, lbl_ps, lbl_ftr): grp.append(item)
    display.root_group = grp; display.refresh(minimum_frames_per_second=0)
except Exception as e:
    print("# Display init error:", e)

# --- 3. 400 kHz I2C Bus & Dual ICM-20948 20-Byte Burst Driver (0x69 & 0x68) ---
class FastICM20948:
    ACC_S, GYR_S, MAG_S = 9.80665 / 8192.0, 1.0 / 65.5, 0.15
    def __init__(self, i2c_bus, addr):
        self.i2c, self.addr = i2c_bus, addr
        self.cmd, self.reg, self.buf = bytearray(2), bytearray([0x2D]), bytearray(20)
        self.gx_b = self.gy_b = self.gz_b = 0.0
        self.ax = self.ay = self.gx = self.gy = self.gz = self.mx = self.my = self.mz = 0.0; self.az = 9.81
        for bank, reg, val, wait in (
            (0x00,0x06,0x01,0.02), (0x00,0x07,0x00,0.0), (0x20,0x01,0x03,0.0), (0x20,0x14,0x03,0.0),
            (0x00,0x0F,0x00,0.0),  (0x00,0x03,0x20,0.0), (0x30,0x01,0x07,0.0), (0x30,0x03,0x0C,0.0),
            (0x30,0x04,0x31,0.0),  (0x30,0x06,0x08,0.0), (0x30,0x05,0x81,0.01),(0x30,0x03,0x8C,0.0),
            (0x30,0x04,0x11,0.0),  (0x30,0x05,0x88,0.0), (0x00,0x7F,0x00,0.01)
        ):
            self._w(0x7F, bank); self._w(reg, val)
            if wait: time.sleep(wait)
    def _w(self, reg, val):
        self.cmd[0], self.cmd[1] = reg, val
        while not self.i2c.try_lock(): pass
        try: self.i2c.writeto(self.addr, self.cmd)
        finally: self.i2c.unlock()
    def read_burst(self):
        while not self.i2c.try_lock(): pass
        try: self.i2c.writeto_then_readfrom(self.addr, self.reg, self.buf)
        finally: self.i2c.unlock()
        b = self.buf
        v = [(b[0]<<8)|b[1], (b[2]<<8)|b[3], (b[4]<<8)|b[5], (b[6]<<8)|b[7], (b[8]<<8)|b[9], (b[10]<<8)|b[11], (b[15]<<8)|b[14], (b[17]<<8)|b[16], (b[19]<<8)|b[18]]
        v = [x - 65536 if x & 0x8000 else x for x in v]
        self.ax, self.ay, self.az = v[0]*self.ACC_S, v[1]*self.ACC_S, v[2]*self.ACC_S
        self.gx, self.gy, self.gz = v[3]*self.GYR_S - self.gx_b, v[4]*self.GYR_S - self.gy_b, v[5]*self.GYR_S - self.gz_b
        self.mx, self.my, self.mz = v[6]*self.MAG_S, v[7]*self.MAG_S, v[8]*self.MAG_S
        amag = math.sqrt(self.ax*self.ax + self.ay*self.ay + self.az*self.az)
        if abs(self.gx) < 0.6 and abs(self.gy) < 0.6 and abs(self.gz) < 0.6 and abs(amag - 9.81) < 0.4:
            self.gx_b += 0.002 * self.gx; self.gy_b += 0.002 * self.gy; self.gz_b += 0.002 * self.gz

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
while not i2c.try_lock(): pass
addrs = i2c.scan(); i2c.unlock()
imu1 = FastICM20948(i2c, 0x69) if 0x69 in addrs else None
imu2 = FastICM20948(i2c, 0x68) if 0x68 in addrs else None

# --- 4. Singularity-Free 3D Quaternion Gravity Filter & Z-Y-X Joint Angles ---
def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return (w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2)
def quat_conj(q): return (q[0], -q[1], -q[2], -q[3])
def quat_norm(q):
    n = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n) if n > 1e-9 else (1.0, 0.0, 0.0, 0.0)
def quat_to_euler_deg(q):
    """Intrinsic Z-Y-X: Flex/Ext around lateral Z, Var/Val around Y, Int/Ext around shaft X."""
    w, x, y, z = q
    return (math.degrees(math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))),
            math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w*y - z*x))))),
            math.degrees(math.atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))))

class QuaternionGravityAHRS:
    DEG2RAD = 0.017453292519943295
    def __init__(self, kp=2.5): self.q, self.kp = (1.0, 0.0, 0.0, 0.0), kp
    def align_to_gravity(self, ax, ay, az):
        amag = math.sqrt(ax*ax + ay*ay + az*az)
        if amag < 1e-3: self.q = (1.0, 0.0, 0.0, 0.0); return
        ax_n, ay_n, az_n = ax/amag, ay/amag, az/amag
        s = math.sqrt(2.0 * (1.0 + az_n)) if az_n >= -0.9999 else 0.0
        self.q = quat_norm((0.5 * s, ay_n / s, -ax_n / s, 0.0)) if s > 0 else (0.0, 1.0, 0.0, 0.0)
    def update(self, ax, ay, az, gx, gy, gz, dt):
        amag = math.sqrt(ax*ax + ay*ay + az*az)
        if abs(gx) < 0.8 and abs(gy) < 0.8 and abs(gz) < 0.8 and abs(amag - 9.81) < 0.5:
            gx = gy = gz = 0.0  # Stationary ZUPT deadband
        wx, wy, wz = gx * self.DEG2RAD, gy * self.DEG2RAD, gz * self.DEG2RAD
        w, x, y, z = self.q
        vx, vy, vz = 2.0*(x*z - w*y), 2.0*(w*x + y*z), w*w - x*x - y*y + z*z
        if 5.0 < amag < 15.0:
            ax_n, ay_n, az_n = ax/amag, ay/amag, az/amag
            wx += self.kp * (ay_n*vz - az_n*vy); wy += self.kp * (az_n*vx - ax_n*vz); wz += self.kp * (ax_n*vy - ay_n*vx)
        hdt = 0.5 * dt
        self.q = quat_norm((w + (-x*wx - y*wy - z*wz)*hdt, x + (w*wx + y*wz - z*wy)*hdt,
                            y + (w*wy - x*wz + z*wx)*hdt,  z + (w*wz + x*wy - y*wx)*hdt))
        return self.q

ahrs1, ahrs2, q_cal = QuaternionGravityAHRS(2.5), QuaternionGravityAHRS(2.5), (1.0, 0.0, 0.0, 0.0)

# --- 5. BLE ("XIAO-IMU"), Local Memory Buffer & Stationary Auto-Calibration ---
gc.collect()
MAX_SAMPLES, FIELDS = 500, 8
mem_buffer = memoryview(bytearray(MAX_SAMPLES * FIELDS * 4)).cast("f")
mem_idx = mem_count = 0
ble = uart = adv = None
try:
    from adafruit_ble import BLERadio
    from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
    from adafruit_ble.services.nordic import UARTService
    ble = BLERadio(); ble.name = "XIAO-IMU"; uart = UARTService()
    adv = ProvideServicesAdvertisement(uart); adv.complete_name = "XIAO-IMU"; ble.start_advertising(adv)
except Exception as e:
    print("# BLE unavailable:", e)

def emit(msg):
    line = f"# {msg}\n"; print(line, end="")
    if ble and ble.connected:
        try: uart.write(line.encode("utf-8"))
        except Exception: pass

def calibrate_neutral_pose(cal_gyro=True, samples=25):
    """Samples stationary pose, zeroes gyro bias, aligns gravity, and sets (0, 0, 0) joint angles."""
    global q_cal, last_t
    set_led(True, True, False)
    for imu, ahrs in ((imu1, ahrs1), (imu2, ahrs2)):
        if not imu: continue
        if cal_gyro: imu.gx_b = imu.gy_b = imu.gz_b = 0.0
        axs = ays = azs = gxs = gys = gzs = 0.0
        for _ in range(samples):
            imu.read_burst()
            axs += imu.ax; ays += imu.ay; azs += imu.az; gxs += imu.gx; gys += imu.gy; gzs += imu.gz
            time.sleep(0.004)
        if cal_gyro: imu.gx_b, imu.gy_b, imu.gz_b = gxs/samples, gys/samples, gzs/samples
        ahrs.align_to_gravity(axs/samples, ays/samples, azs/samples)
    q_cal, last_t = quat_norm(quat_mul(quat_conj(ahrs1.q), ahrs2.q)), time.monotonic()
    emit("EVENT:CAL_DONE")

is_streaming, is_recording = True, False
start_t = last_t = last_ble_t = last_disp_t = time.monotonic()

def start_recording_with_auto_cal():
    global start_t, last_t, mem_idx, mem_count, is_recording
    calibrate_neutral_pose(cal_gyro=True)
    start_t = last_t = time.monotonic(); mem_idx, mem_count, is_recording = 0, 0, True
    emit("EVENT:REC_ON")

def handle_command(cmd):
    global is_streaming, is_recording
    c = cmd.strip().upper()
    if "CAL_POSE_ONLY" in c: calibrate_neutral_pose(cal_gyro=False)
    elif "CAL" in c: calibrate_neutral_pose(cal_gyro=True)
    elif "REC_START" in c or "START_REC" in c or "REC_ON" in c: start_recording_with_auto_cal()
    elif "REC_STOP" in c or "STOP_REC" in c or "REC_OFF" in c: is_recording = False; emit("EVENT:REC_OFF")
    elif "START_STREAM" in c or "STREAM_ON" in c: is_streaming = True; emit("EVENT:STREAM_ON")
    elif "STOP_STREAM" in c or "STREAM_OFF" in c: is_streaming = False; emit("EVENT:STREAM_OFF")
    elif "RESET" in c: is_recording = False; start_recording_with_auto_cal(); is_recording = False; emit("EVENT:RESET_DONE")
    elif "DUMP_MEM" in c:
        print(f"# DUMP_START:{mem_count}")
        for i in range(mem_count):
            b = i * FIELDS
            row = f"MEM,{i},{mem_buffer[b]:.3f},{mem_buffer[b+1]:.2f},{mem_buffer[b+2]:.2f},{mem_buffer[b+3]:.2f},{mem_buffer[b+4]:.2f},{mem_buffer[b+5]:.2f},{mem_buffer[b+6]:.2f},{mem_buffer[b+7]:.2f}\n"
            print(row, end="")
            if ble and ble.connected:
                try: uart.write(row.encode("utf-8"))
                except Exception: pass
        print("# DUMP_END")

calibrate_neutral_pose(cal_gyro=True)
start_t = last_t = time.monotonic()
usb_buf, ble_buf, vbat = "", "", read_vbat()

# --- 6. Main Real-Time Kinematics, Display & Telemetry Loop ---
while True:
    now = time.monotonic()
    dt = now - last_t if 0.0 < (now - last_t) < 0.25 else 0.005
    last_t, elapsed = now, now - start_t
    pa, pb, pc = not btn_a.value, not btn_b.value, not btn_c.value
    if pa and not prev_a: calibrate_neutral_pose(cal_gyro=True)
    if pb and not prev_b: is_streaming = not is_streaming
    if pc and not prev_c:
        if not is_recording: start_recording_with_auto_cal()
        else: is_recording = False; emit("EVENT:REC_OFF")
    prev_a, prev_b, prev_c = pa, pb, pc

    if supervisor.runtime.serial_bytes_available:
        ch = sys.stdin.read(1)
        if ch in ("\n", "\r") and usb_buf: handle_command(usb_buf); usb_buf = ""
        elif ch not in ("\n", "\r"): usb_buf += ch
    if ble and ble.connected and uart.in_waiting:
        ble_buf += uart.read(uart.in_waiting).decode("utf-8", "ignore")
        while "\n" in ble_buf: line, ble_buf = ble_buf.split("\n", 1); handle_command(line)

    if imu1: imu1.read_burst()
    if imu2: imu2.read_burst()
    q1 = ahrs1.update(imu1.ax, imu1.ay, imu1.az, imu1.gx, imu1.gy, imu1.gz, dt) if imu1 else (1.0,0.0,0.0,0.0)
    q2 = ahrs2.update(imu2.ax, imu2.ay, imu2.az, imu2.gx, imu2.gy, imu2.gz, dt) if imu2 else (1.0,0.0,0.0,0.0)
    fe, ru, ps = quat_to_euler_deg(quat_norm(quat_mul(quat_conj(quat_mul(q1, q_cal)), q2)))
    ax1, ay1, az1 = (imu1.ax, imu1.ay, imu1.az) if imu1 else (0.0, 0.0, 9.81)
    gx1, gy1, gz1 = (imu1.gx, imu1.gy, imu1.gz) if imu1 else (0.0, 0.0, 0.0)
    mx1, my1, mz1 = (imu1.mx, imu1.my, imu1.mz) if imu1 else (25.0, 0.0, -40.0)
    ax2, ay2, az2 = (imu2.ax, imu2.ay, imu2.az) if imu2 else (0.0, 0.0, 9.81)
    gx2, gy2, gz2 = (imu2.gx, imu2.gy, imu2.gz) if imu2 else (0.0, 0.0, 0.0)
    mx2, my2, mz2 = (imu2.mx, imu2.my, imu2.mz) if imu2 else (25.0, 0.0, -40.0)

    if is_recording:
        b = mem_idx * FIELDS
        mem_buffer[b], mem_buffer[b+1], mem_buffer[b+2], mem_buffer[b+3] = elapsed, fe, ru, ps
        mem_buffer[b+4], mem_buffer[b+5] = math.sqrt(ax1*ax1 + ay1*ay1 + az1*az1), math.sqrt(ax2*ax2 + ay2*ay2 + az2*az2)
        mem_buffer[b+6], mem_buffer[b+7] = math.sqrt(gx1*gx1 + gy1*gy1 + gz1*gz1), math.sqrt(gx2*gx2 + gy2*gy2 + gz2*gz2)
        mem_idx = (mem_idx + 1) % MAX_SAMPLES
        if mem_count < MAX_SAMPLES: mem_count += 1

    if is_streaming:
        rf = 1 if is_recording else 0
        print(f"D,{elapsed:.3f},{ax1:.2f},{ay1:.2f},{az1:.2f},{gx1:.1f},{gy1:.1f},{gz1:.1f},{mx1:.1f},{my1:.1f},{mz1:.1f},{ax2:.2f},{ay2:.2f},{az2:.2f},{gx2:.1f},{gy2:.1f},{gz2:.1f},{mx2:.1f},{my2:.1f},{mz2:.1f},{fe:.1f},{ru:.1f},{ps:.1f},{vbat:.2f},{rf}\n", end="")
        if ble and ble.connected and (now - last_ble_t) >= 0.04:
            last_ble_t = now
            try: uart.write(f"B,{elapsed:.2f},{fe:.1f},{ru:.1f},{ps:.1f},{ax1:.2f},{ay1:.2f},{az1:.2f},{ax2:.2f},{ay2:.2f},{az2:.2f},{vbat:.2f},{rf}\n".encode("utf-8"))
            except Exception: pass

    if (now - last_disp_t) >= 0.40:
        last_disp_t, vbat = now, 0.85 * vbat + 0.15 * read_vbat()
        set_led(is_recording, is_streaming and not is_recording, not is_streaming and not is_recording)
        if ble and not ble.connected and not ble.advertising:
            try: ble.start_advertising(adv)
            except Exception: pass
        if display:
            lbl_fe.text, lbl_ru.text, lbl_ps.text = f"Flx/Ext:{fe:+6.1f}d", f"Var/Val:{ru:+6.1f}d", f"Int/Ext:{ps:+6.1f}d"
            lbl_ftr.text = f"{'REC' if is_recording else 'IDLE'} | {'BLE' if (ble and ble.connected) else 'USB'} | {vbat:.2f}V"
            try: display.refresh(minimum_frames_per_second=0)
            except Exception: pass
