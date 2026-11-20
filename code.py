# SPDX-FileCopyrightText: 2026 Eric G. Meyer / LTU Experimental Biomechanics Lab (EBL)
# SPDX-License-Identifier: MIT
"""
Seeed Studio XIAO 1.14" IPS Display (nRF52840 Plus) - Dual ICM-20948 9-DOF Joint Kinematics System
==================================================================================================
Hardware Configuration:
  - Microcontroller Board: Seeed Studio XIAO 1.14" IPS Display (nRF52840 Plus)
  - Display: Onboard 1.14" IPS TFT (ST7789, 240x135 Landscape Mode) via Hardware SPI
      * SCK  = board.SCK  (D8  / P1.13)
      * MOSI = board.MOSI (D10 / P1.15)
      * CS   = board.D2   (D2  / P0.28)
      * DC   = board.D3   (D3  / P0.29)
      * RST  = P1.03      (D17)
      * BL   = P1.05      (D18)
  - External Sensors (Daisy-Chained via 4-pin JST SH / STEMMA QT on I2C bus D4=SDA, D5=SCL @ 400 kHz):
      * Sensor #1 (Proximal: Thigh / Femur): Adafruit ICM-20948 9-DOF IMU @ I2C 0x69 (Default)
      * Sensor #2 (Distal:   Shank / Tibia): Adafruit ICM-20948 9-DOF IMU @ I2C 0x68 (AD0 Jumper Soldered)
    Lateral Mounting Convention (Femur/Tibia Model & Knee Sleeve Prototype):
      * Sensor X-axis (between STEMMA QT connectors): Proximal-Distal along bone shaft (Axial Rotation)
      * Sensor Y-axis (toward header pins):           Anterior-Posterior in sagittal plane (Varus/Valgus)
      * Sensor Z-axis (normal to flat PCB face):      Laterally outward along Flexion/Extension hinge axis
  - Physical Controls (Active-Low with 1k hardware pull-ups):
      * BTN_A (Left Front,  D6 / P1.11): Short = Next Display Screen | Long (>1s) = Calibrate Neutral Pose
      * BTN_B (Right Front, D7 / P1.12): Short = Start/Stop Stream   | Long (>1s) = Reset System
      * BTN_C (Side Button, D19/ P1.07): Short = Auto-Cal & Record   | Long (>1.5s) = Standby / Low-Power Mode
  - Built-in Feedback & Auxiliary Capabilities:
      * 3-in-1 RGB User LED (LED_RED, LED_GREEN, LED_BLUE): Mode, Calibration, & Recording Indicator
      * Battery Voltage & Charge Status Monitor (P0.31 VBAT, P0.14 EN, P0.17 CHG)
      * Onboard PDM Digital Microphone (D0=CLK, D1=DATA): Acoustic Clap/Voice Event Marker
      * Onboard 6-Axis IMU (LSM6DS3 @ 0x6A): Backup proximal reference & wake-on-motion monitor
"""

import time
import math
import gc
import array
import board
import busio
import digitalio
import microcontroller
import supervisor

# ============================================================================
# 1. BUILT-IN RGB INDICATOR LED & BATTERY MONITOR SETUP
# ============================================================================
class StatusLED:
    """Controls the XIAO nRF52840 built-in active-low RGB LED."""
    def __init__(self):
        self.r = None
        self.g = None
        self.b = None
        try:
            self.r = digitalio.DigitalInOut(board.LED_RED)
            self.g = digitalio.DigitalInOut(board.LED_GREEN)
            self.b = digitalio.DigitalInOut(board.LED_BLUE)
            for led in (self.r, self.g, self.b):
                led.direction = digitalio.Direction.OUTPUT
                led.value = True  # Active-low: True = OFF
        except Exception:
            pass

    def set_color(self, r_on, g_on, b_on):
        if self.r:
            self.r.value = not r_on
            self.g.value = not g_on
            self.b.value = not b_on

    def off(self):
        self.set_color(False, False, False)


class BatteryMonitor:
    """Reads LiPo battery voltage, percentage, and USB-C charging status."""
    def __init__(self):
        self.vbat_adc = None
        self.vbat_en = None
        self.chg_pin = None
        self.voltage = 4.10
        self.percent = 92
        self.charging = False
        try:
            import analogio
            vbat_pin = getattr(board, "VOLTAGE_MONITOR", microcontroller.pin.P0_31)
            self.vbat_adc = analogio.AnalogIn(vbat_pin)
            if hasattr(board, "READ_BATT_ENABLE"):
                self.vbat_en = digitalio.DigitalInOut(board.READ_BATT_ENABLE)
                self.vbat_en.direction = digitalio.Direction.OUTPUT
                self.vbat_en.value = False  # Active-low enable for voltage divider
            chg_p = getattr(board, "CHARGE_STATUS", microcontroller.pin.P0_17)
            self.chg_pin = digitalio.DigitalInOut(chg_p)
            self.chg_pin.direction = digitalio.Direction.INPUT
            self.chg_pin.pull = digitalio.Pull.UP
        except Exception:
            pass

    def update(self):
        if self.vbat_adc:
            try:
                # XIAO nRF52840 voltage divider scale (~1510/510 = 2.96x of 3.3V ADC reference)
                raw = self.vbat_adc.value
                v = (raw / 65535.0) * 3.3 * 2.961
                if 2.5 < v < 4.5:
                    self.voltage = 0.85 * self.voltage + 0.15 * v
                    pct = int((self.voltage - 3.30) / (4.18 - 3.30) * 100.0)
                    self.percent = max(0, min(100, pct))
            except Exception:
                pass
        if self.chg_pin:
            try:
                self.charging = not self.chg_pin.value  # Active-low when charging
            except Exception:
                pass
        return self.voltage, self.percent, self.charging


status_led = StatusLED()
status_led.set_color(False, False, True)  # Booting: Blue
battery_mon = BatteryMonitor()
battery_mon.update()

# ============================================================================
# 2. PHYSICAL BUTTONS (BTN_A = D6, BTN_B = D7, BTN_C = P1.07 / D19)
# ============================================================================
class ButtonControl:
    """Debounced short-press and long-press detector for active-low buttons."""
    def __init__(self, pin, long_press_s=1.1):
        self.io = digitalio.DigitalInOut(pin)
        self.io.direction = digitalio.Direction.INPUT
        self.io.pull = digitalio.Pull.UP
        self.long_press_s = long_press_s
        self.was_pressed = False
        self.press_start = 0.0
        self.long_fired = False

    def poll(self, now):
        """Returns 'SHORT', 'LONG', or None."""
        pressed = not self.io.value  # Active-low
        event = None
        if pressed and not self.was_pressed:
            self.was_pressed = True
            self.press_start = now
            self.long_fired = False
        elif pressed and self.was_pressed:
            if not self.long_fired and (now - self.press_start) >= self.long_press_s:
                self.long_fired = True
                event = "LONG"
        elif not pressed and self.was_pressed:
            self.was_pressed = False
            if not self.long_fired and (now - self.press_start) >= 0.03:
                event = "SHORT"
        return event


btn_a = ButtonControl(board.D6, long_press_s=1.1)              # Left button: Screen / Calibrate
btn_b = ButtonControl(board.D7, long_press_s=1.1)              # Right button: Stream / Reset
btn_c = ButtonControl(microcontroller.pin.P1_07, long_press_s=1.5)  # Side button: Auto-Cal+Record / Standby

# ============================================================================
# 3. ONBOARD PDM MICROPHONE (CLAP / AUDIO EVENT MARKER)
# ============================================================================
class AudioEventDetector:
    """Uses the onboard PDM microphone (D0=CLK, D1=DATA) to detect clap/voice markers."""
    def __init__(self):
        self.pdm = None
        self.buf = None
        self.last_marker_time = 0.0
        self.last_rms = 0
        try:
            import audiobusio
            self.pdm = audiobusio.PDMIn(
                board.D0,
                board.D1,
                sample_rate=16000,
                bit_depth=16
            )
            self.buf = array.array("H", [0] * 64)
        except Exception:
            self.pdm = None

    def check_spike(self, now, threshold=12000):
        if not self.pdm or (now - self.last_marker_time) < 1.0:
            return False
        try:
            self.pdm.record(self.buf, len(self.buf))
            mean_val = sum(self.buf) // len(self.buf)
            peak = max(abs(x - mean_val) for x in self.buf)
            self.last_rms = peak
            if peak > threshold:
                self.last_marker_time = now
                return True
        except Exception:
            pass
        return False


audio_marker = AudioEventDetector()

# ============================================================================
# 4. 1.14" IPS DISPLAY (ST7789, 240x135 LANDSCAPE MODE)
# ============================================================================
display = None
backlight_pin = None
lbl_header = None
lbl_line1 = None
lbl_line2 = None
lbl_line3 = None
lbl_footer = None

try:
    import displayio
    import terminalio
    try:
        from fourwire import FourWire
    except ImportError:
        from displayio import FourWire
    from adafruit_display_text import bitmap_label as label
    from adafruit_st7789 import ST7789

    displayio.release_displays()

    # Backlight control on P1.05 (D18)
    backlight_pin = digitalio.DigitalInOut(microcontroller.pin.P1_05)
    backlight_pin.direction = digitalio.Direction.OUTPUT
    backlight_pin.value = True  # Turn backlight ON

    spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI)
    display_bus = FourWire(
        spi,
        command=board.D3,                   # LCD_DC
        chip_select=board.D2,               # LCD_CS
        reset=microcontroller.pin.P1_03,    # LCD_RST (D17)
        baudrate=24000000                   # 24 MHz fast SPI
    )

    # Landscape mode: 240x135 pixels (rotation=90)
    display = ST7789(
        display_bus,
        width=240,
        height=135,
        rowstart=40,
        colstart=53,
        rotation=90,
        auto_refresh=False
    )

    root_group = displayio.Group()

    # Dark Navy Header Bar (240x24)
    bg_bitmap = displayio.Bitmap(240, 135, 2)
    bg_palette = displayio.Palette(2)
    bg_palette[0] = 0x0B1120  # Deep dark slate background
    bg_palette[1] = 0x0284C7  # LTU EBL Accent Blue header strip
    for x in range(240):
        for y in range(22):
            bg_bitmap[x, y] = 1
    bg_grid = displayio.TileGrid(bg_bitmap, pixel_shader=bg_palette)
    root_group.append(bg_grid)

    lbl_header = label.Label(terminalio.FONT, text="LTU EBL | KNEE 3D [S1+S2]", color=0xFFFFFF, x=6, y=11, scale=1)
    lbl_line1 = label.Label(terminalio.FONT, text="Flx/Ext:   0.0 deg", color=0x38BDF8, x=8, y=38, scale=2)
    lbl_line2 = label.Label(terminalio.FONT, text="Var/Val:   0.0 deg", color=0x34D399, x=8, y=68, scale=2)
    lbl_line3 = label.Label(terminalio.FONT, text="Int/Ext:   0.0 deg", color=0xFBBF24, x=8, y=98, scale=2)
    lbl_footer = label.Label(terminalio.FONT, text="STRM:ON REC:OFF 100Hz 92%", color=0x94A3B8, x=6, y=125, scale=1)

    root_group.append(lbl_header)
    root_group.append(lbl_line1)
    root_group.append(lbl_line2)
    root_group.append(lbl_line3)
    root_group.append(lbl_footer)

    display.root_group = root_group
    display.refresh(minimum_frames_per_second=0)
except Exception as disp_err:
    print(f"# Display init note: {disp_err}")
    display = None

# ============================================================================
# 5. HIGH-SPEED I2C BUS & DUAL ICM-20948 (0x69 & 0x68) BURST DRIVER
# ============================================================================
class FastICM20948:
    """
    High-speed direct I2C burst driver for Adafruit ICM-20948 (ICM26948) 9-DOF IMU.
    Uses each ICM-20948's internal Auxiliary I2C Master (SLV0) to poll its onboard
    AK09916 3-axis magnetometer (0x0C) into EXT_SLV_SENS_DATA_00..07 (0x3B..0x42).
    This prevents 0x0C magnetometer bus collisions when two ICM-20948 sensors (0x69
    and 0x68) are daisy-chained in series, and reads all 9-DOF (Accel + Gyro + Mag)
    in a single 20-byte 400 kHz I2C burst transaction (<0.6 ms per sensor).
    """
    REG_BANK_SEL = 0x7F
    REG_USER_CTRL = 0x03
    REG_PWR_MGMT_1 = 0x06
    REG_PWR_MGMT_2 = 0x07
    REG_INT_PIN_CFG = 0x0F
    REG_ACCEL_XOUT_H = 0x2D

    # Scale factors: +-4g Accel (8192 LSB/g), +-500 dps Gyro (65.5 LSB/dps), Mag (0.15 uT/LSB)
    ACCEL_SCALE = 9.80665 / 8192.0
    GYRO_SCALE = 1.0 / 65.5
    MAG_SCALE = 0.15

    def __init__(self, i2c_bus, address):
        self.i2c = i2c_bus
        self.addr = address
        self.cmd_buf = bytearray(2)
        self.reg_buf = bytearray(1)
        # 20 bytes: 6 Accel + 6 Gyro + 2 Temp + 6 Mag (EXT_SLV_SENS_DATA_00..05)
        self.raw_buf = bytearray(20)

        # Gyro bias offsets (deg/s)
        self.gx_bias = 0.0
        self.gy_bias = 0.0
        self.gz_bias = 0.0

        # Latest 9-DOF readings
        self.ax = 0.0
        self.ay = 0.0
        self.az = 9.81
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.mx = 0.0
        self.my = 0.0
        self.mz = 0.0

        self._init_device()

    def _write_reg(self, reg, val):
        self.cmd_buf[0] = reg
        self.cmd_buf[1] = val
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.addr, self.cmd_buf)
        finally:
            self.i2c.unlock()

    def _init_device(self):
        # 1. Select User Bank 0 & wake chip
        self._write_reg(self.REG_BANK_SEL, 0x00)
        self._write_reg(self.REG_PWR_MGMT_1, 0x01)
        time.sleep(0.02)
        self._write_reg(self.REG_PWR_MGMT_2, 0x00)

        # 2. Select User Bank 2: Configure Accel (+-4g, DLPF) & Gyro (+-500 dps, DLPF)
        self._write_reg(self.REG_BANK_SEL, 0x20)
        self._write_reg(0x01, 0x03)  # GYRO_CONFIG_1: +-500 dps, fast DLPF
        self._write_reg(0x14, 0x03)  # ACCEL_CONFIG:  +-4g, fast DLPF

        # 3. Enable Internal I2C Master in Bank 0 (isolates each sensor's 0x0C AK09916 mag)
        self._write_reg(self.REG_BANK_SEL, 0x00)
        self._write_reg(self.REG_INT_PIN_CFG, 0x00)  # Disable external bypass to avoid 0x0C collision
        self._write_reg(self.REG_USER_CTRL, 0x20)    # Enable I2C_MST_EN

        # 4. Select User Bank 3: Configure I2C Master SLV0 to set AK09916 (0x0C) to 100 Hz Mode 4
        self._write_reg(self.REG_BANK_SEL, 0x30)
        self._write_reg(0x01, 0x07)  # I2C_MST_CTRL: 400 kHz clock
        self._write_reg(0x03, 0x0C)  # I2C_SLV0_ADDR: Write to AK09916 (0x0C)
        self._write_reg(0x04, 0x31)  # I2C_SLV0_REG:  CNTL2 register (0x31)
        self._write_reg(0x06, 0x08)  # I2C_SLV0_DO:   Continuous 100 Hz mode (0x08)
        self._write_reg(0x05, 0x81)  # I2C_SLV0_CTRL: Enable 1-byte write
        time.sleep(0.01)

        # Configure SLV0 to continuously read 8 bytes (HXL..ST2, 0x11..0x18) into 0x3B..0x42
        self._write_reg(0x03, 0x8C)  # I2C_SLV0_ADDR: Read from AK09916 (0x80 | 0x0C)
        self._write_reg(0x04, 0x11)  # I2C_SLV0_REG:  HXL (0x11)
        self._write_reg(0x05, 0x88)  # I2C_SLV0_CTRL: Enable 8-byte continuous read

        # Return to User Bank 0 for fast burst reads
        self._write_reg(self.REG_BANK_SEL, 0x00)
        time.sleep(0.01)

    def read_burst(self):
        """Burst-reads 20 bytes (3-Axis Accel + 3-Axis Gyro + Temp + 3-Axis Mag) in <0.6 ms."""
        self.reg_buf[0] = self.REG_ACCEL_XOUT_H
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto_then_readfrom(self.addr, self.reg_buf, self.raw_buf)
        finally:
            self.i2c.unlock()

        b = self.raw_buf
        # Big-endian Accel & Gyro (0..11)
        raw_ax = (b[0] << 8) | b[1]
        raw_ay = (b[2] << 8) | b[3]
        raw_az = (b[4] << 8) | b[5]
        raw_gx = (b[6] << 8) | b[7]
        raw_gy = (b[8] << 8) | b[9]
        raw_gz = (b[10] << 8) | b[11]

        # Little-endian AK09916 Magnetometer in EXT_SLV_SENS_DATA_00..05 (14..19)
        raw_mx = (b[15] << 8) | b[14]
        raw_my = (b[17] << 8) | b[16]
        raw_mz = (b[19] << 8) | b[18]

        if raw_ax & 0x8000: raw_ax -= 65536
        if raw_ay & 0x8000: raw_ay -= 65536
        if raw_az & 0x8000: raw_az -= 65536
        if raw_gx & 0x8000: raw_gx -= 65536
        if raw_gy & 0x8000: raw_gy -= 65536
        if raw_gz & 0x8000: raw_gz -= 65536
        if raw_mx & 0x8000: raw_mx -= 65536
        if raw_my & 0x8000: raw_my -= 65536
        if raw_mz & 0x8000: raw_mz -= 65536

        self.ax = raw_ax * self.ACCEL_SCALE
        self.ay = raw_ay * self.ACCEL_SCALE
        self.az = raw_az * self.ACCEL_SCALE
        self.gx = (raw_gx * self.GYRO_SCALE) - self.gx_bias
        self.gy = (raw_gy * self.GYRO_SCALE) - self.gy_bias
        self.gz = (raw_gz * self.GYRO_SCALE) - self.gz_bias
        self.mx = raw_mx * self.MAG_SCALE
        self.my = raw_my * self.MAG_SCALE
        self.mz = raw_mz * self.MAG_SCALE

    def update_stationary_bias(self):
        """Adaptively refines gyro bias during runtime whenever the sensor is very still."""
        amag = math.sqrt(self.ax * self.ax + self.ay * self.ay + self.az * self.az)
        if abs(self.gx) < 0.6 and abs(self.gy) < 0.6 and abs(self.gz) < 0.6 and abs(amag - 9.81) < 0.4:
            self.gx_bias += 0.002 * self.gx
            self.gy_bias += 0.002 * self.gy
            self.gz_bias += 0.002 * self.gz

    def sleep(self, enable=True):
        self._write_reg(self.REG_BANK_SEL, 0x00)
        self._write_reg(self.REG_PWR_MGMT_1, 0x41 if enable else 0x01)


# Initialize 400 kHz Fast-Mode I2C on D5 (SCL) and D4 (SDA)
i2c = None
imu1 = None  # Sensor #1: Thigh / Femur Proximal (0x69)
imu2 = None  # Sensor #2: Shank / Tibia Distal   (0x68 - soldered jumper)

try:
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    while not i2c.try_lock():
        pass
    detected_addrs = i2c.scan()
    i2c.unlock()
    print(f"# I2C Bus Scanned @ 400kHz: {[hex(a) for a in detected_addrs]}")

    if 0x69 in detected_addrs:
        imu1 = FastICM20948(i2c, 0x69)
        print("# Sensor #1 (Proximal / Thigh) initialized at 0x69")
    if 0x68 in detected_addrs:
        imu2 = FastICM20948(i2c, 0x68)
        print("# Sensor #2 (Distal / Shank) initialized at 0x68")
except Exception as i2c_err:
    print(f"# I2C init warning: {i2c_err}")

# ============================================================================
# 6. SINGULARITY-FREE 3D QUATERNION FUSION & RELATIVE JOINT ANGLE SOLVER
# ============================================================================
def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])

def quat_norm(q):
    n = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    inv = 1.0 / n
    return (q[0]*inv, q[1]*inv, q[2]*inv, q[3]*inv)

def quat_to_euler_deg(q):
    """
    Decomposes relative joint quaternion q_rel = conj(q1 * q_cal) * q2 = (w, x, y, z)
    in intrinsic Z-Y-X order matching the lateral sensor mounting on femur & tibia:
      - Primary angle Flexion / Extension (flex_ext): rotation around Sensor Z-axis
        (PCB normal pointing laterally outward along the knee hinge axis).
      - Secondary angle Varus / Valgus or Ab/Adduction (rad_uln): rotation around
        Sensor Y-axis (out-of-sagittal-plane tilt).
      - Tertiary angle Internal / External Axial Rotation (pro_sup): rotation around
        Sensor X-axis (longitudinal bone shaft axis between STEMMA QT ports).
    """
    w, x, y, z = q

    # 1. Primary angle: Flexion / Extension around Sensor Z-axis (mediolateral hinge axis)
    flex_ext = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    # 2. Secondary angle: Varus / Valgus or Ab/Adduction around Sensor Y-axis
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    rad_uln = math.degrees(math.asin(sinp))

    # 3. Tertiary angle: Internal / External Axial Rotation around Sensor X-axis (bone shaft)
    pro_sup = math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))

    return flex_ext, rad_uln, pro_sup


class ComplementaryAHRS:
    """
    Singularity-Free 3D Quaternion Gravity Cross-Product Filter (6-DOF Mahony-style).
    Eliminates Euler atan2(ay, az) vertical gimbal lock and uncalibrated magnetometer
    hard-iron yaw divergence while providing zero-drift stationary ZUPT clamping.
    """
    DEG2RAD = 0.017453292519943295

    def __init__(self, kp=2.5):
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.kp = kp

    def align_to_gravity(self, ax, ay, az):
        """
        Sets self.q directly to the shortest-arc quaternion rotating body measured
        gravity (ax, ay, az) to world +Z = (0, 0, 1) with zero singularities.
        """
        amag = math.sqrt(ax * ax + ay * ay + az * az)
        if amag < 1e-3:
            self.q = (1.0, 0.0, 0.0, 0.0)
            return self.q
        inv_a = 1.0 / amag
        ax_n = ax * inv_a
        ay_n = ay * inv_a
        az_n = az * inv_a
        if az_n < -0.9999:
            self.q = (0.0, 1.0, 0.0, 0.0)
        else:
            s = math.sqrt(2.0 * (1.0 + az_n))
            inv_s = 1.0 / s
            self.q = quat_norm((0.5 * s, ay_n * inv_s, -ax_n * inv_s, 0.0))
        return self.q

    def reset_from_accel(self, ax, ay, az):
        return self.align_to_gravity(ax, ay, az)

    def update(self, ax, ay, az, gx, gy, gz, dt, mx=0.0, my=0.0, mz=0.0):
        amag = math.sqrt(ax * ax + ay * ay + az * az)

        # 1. Stationary Gyro Deadband (ZUPT) for strictly zero stationary drift
        if abs(gx) < 0.8 and abs(gy) < 0.8 and abs(gz) < 0.8 and abs(amag - 9.81) < 0.5:
            gx, gy, gz = 0.0, 0.0, 0.0

        wx = gx * self.DEG2RAD
        wy = gy * self.DEG2RAD
        wz = gz * self.DEG2RAD

        w, x, y, z = self.q

        # 2. Predicted unit gravity vector in body frame from self.q = (w, x, y, z)
        vx = 2.0 * (x * z - w * y)
        vy = 2.0 * (w * x + y * z)
        vz = w * w - x * x - y * y + z * z

        # 3. 3D Cross-Product Gravity Feedback (singularity-free at any 3D pose)
        if 5.0 < amag < 15.0:
            inv_a = 1.0 / amag
            ax_n = ax * inv_a
            ay_n = ay * inv_a
            az_n = az * inv_a
            ex = ay_n * vz - az_n * vy
            ey = az_n * vx - ax_n * vz
            ez = ax_n * vy - ay_n * vx
            wx += self.kp * ex
            wy += self.kp * ey
            wz += self.kp * ez

        # 4. Integrate quaternion rate
        half_dt = 0.5 * dt
        qw = w + (-x * wx - y * wy - z * wz) * half_dt
        qx = x + ( w * wx + y * wz - z * wy) * half_dt
        qy = y + ( w * wy - x * wz + z * wx) * half_dt
        qz = z + ( w * wz + x * wy - y * wx) * half_dt
        self.q = quat_norm((qw, qx, qy, qz))
        return self.q


ahrs1 = ComplementaryAHRS(kp=2.5)
ahrs2 = ComplementaryAHRS(kp=2.5)
# Relative neutral calibration quaternion: q_cal = conj(q1_neutral) * q2_neutral
q_cal = (1.0, 0.0, 0.0, 0.0)
is_calibrated = False
joint_name = "KNEE"

# ============================================================================
# 7. LOCAL SRAM RING BUFFER & FLASH CSV STORAGE
# ============================================================================
# Pre-allocate compact float ring buffer in SRAM using zero-copy memoryview
# (8 floats per frame: t, flex_ext, rad_uln, pro_sup, s1_amag, s2_amag, s1_gmag, s2_gmag)
gc.collect()
MAX_MEM_SAMPLES = 500
FIELDS_PER_SAMPLE = 8
_raw_mem_bytes = bytearray(MAX_MEM_SAMPLES * FIELDS_PER_SAMPLE * 4)
mem_buffer = memoryview(_raw_mem_bytes).cast("f")
mem_write_idx = 0
mem_sample_count = 0
flash_log_file = None

def open_flash_log():
    """Attempts to open a local CSV file on the 2MB onboard Flash if mounted writable."""
    global flash_log_file
    if flash_log_file:
        return
    try:
        flash_log_file = open("/local_imu_log.csv", "a")
        flash_log_file.write("Elapsed_s,FlexExt_deg,RadUln_deg,ProSup_deg,S1_Amag,S2_Amag,S1_Gmag,S2_Gmag\n")
    except Exception:
        flash_log_file = None

def close_flash_log():
    global flash_log_file
    if flash_log_file:
        try:
            flash_log_file.flush()
            flash_log_file.close()
        except Exception:
            pass
        flash_log_file = None

# ============================================================================
# 8. BLUETOOTH LOW ENERGY (BLE) NORDIC UART SERVICE (iPhone / iOS Optimized)
# ============================================================================
has_ble = False
ble = None
uart_service = None
advertisement = None

try:
    from adafruit_ble import BLERadio
    from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
    from adafruit_ble.services.nordic import UARTService

    ble = BLERadio()
    # 8-char name so Complete Name + 128-bit NUS UUID fit inside 31-byte iOS BLE adv packet
    ble.name = "XIAO-IMU"
    uart_service = UARTService()
    advertisement = ProvideServicesAdvertisement(uart_service)
    advertisement.complete_name = "XIAO-IMU"
    ble.start_advertising(advertisement)
    has_ble = True
    print("# BLE initialized: Advertising as 'XIAO-IMU' (iPhone/iOS NUS 31-byte packet)")
except Exception as ble_err:
    print(f"# BLE init skipped (USB-only mode): {ble_err}")

# ============================================================================
# 9. CALIBRATION, RECORDING, RESET & COMMAND HANDLERS
# ============================================================================
is_streaming = True
is_recording = False
is_standby = False
active_screen = 0  # 0: 3D Joint Angles, 1: Dual Raw IMU, 2: Memory/Rec, 3: Battery/System
NUM_SCREENS = 4

start_time = time.monotonic()
last_loop_time = start_time
loop_count = 0
measured_hz = 0
last_hz_time = start_time
last_display_time = start_time
last_batt_time = start_time
last_ble_time = start_time


def send_status_event(msg):
    line = f"# {msg}\n"
    print(line, end="")
    if has_ble and ble.connected:
        try:
            uart_service.write(line.encode("utf-8"))
        except Exception:
            pass


def calibrate_neutral_pose(calibrate_gyro=True, samples=25):
    """
    Rapidly samples 25 readings (~0.12s) assuming the subject is standing stationary,
    computes stationary gyro bias and mean gravity vector for both sensors, snaps
    both 3D quaternion filters directly to gravity, and sets q_cal so relative
    3D angles immediately read (0.0, 0.0, 0.0).
    """
    global q_cal, is_calibrated, last_loop_time
    status_led.set_color(True, True, False)  # Yellow during calibration
    n_cal = max(1, int(samples))
    if imu1 or imu2:
        ax1_s = ay1_s = az1_s = 0.0
        gx1_s = gy1_s = gz1_s = 0.0
        ax2_s = ay2_s = az2_s = 0.0
        gx2_s = gy2_s = gz2_s = 0.0
        if calibrate_gyro:
            if imu1:
                imu1.gx_bias = imu1.gy_bias = imu1.gz_bias = 0.0
            if imu2:
                imu2.gx_bias = imu2.gy_bias = imu2.gz_bias = 0.0
        for _ in range(n_cal):
            if imu1:
                imu1.read_burst()
                ax1_s += imu1.ax
                ay1_s += imu1.ay
                az1_s += imu1.az
                gx1_s += imu1.gx
                gy1_s += imu1.gy
                gz1_s += imu1.gz
            if imu2:
                imu2.read_burst()
                ax2_s += imu2.ax
                ay2_s += imu2.ay
                az2_s += imu2.az
                gx2_s += imu2.gx
                gy2_s += imu2.gy
                gz2_s += imu2.gz
            time.sleep(0.004)
        if imu1:
            if calibrate_gyro:
                imu1.gx_bias = gx1_s / n_cal
                imu1.gy_bias = gy1_s / n_cal
                imu1.gz_bias = gz1_s / n_cal
                imu1.gx = imu1.gy = imu1.gz = 0.0
            ahrs1.align_to_gravity(ax1_s / n_cal, ay1_s / n_cal, az1_s / n_cal)
        if imu2:
            if calibrate_gyro:
                imu2.gx_bias = gx2_s / n_cal
                imu2.gy_bias = gy2_s / n_cal
                imu2.gz_bias = gz2_s / n_cal
                imu2.gx = imu2.gy = imu2.gz = 0.0
            ahrs2.align_to_gravity(ax2_s / n_cal, ay2_s / n_cal, az2_s / n_cal)

    # Compute relative neutral offset quaternion q_cal = conj(q1) * q2
    q_cal = quat_norm(quat_mul(quat_conj(ahrs1.q), ahrs2.q))
    is_calibrated = True
    last_loop_time = time.monotonic()
    send_status_event("EVENT:CAL_DONE")


def start_recording_with_auto_cal():
    """
    Immediately executes neutral standing calibration first, resets session timer
    and memory buffer indices, starts recording, and emits CAL_DONE + REC_ON.
    """
    global start_time, last_loop_time, mem_write_idx, mem_sample_count, is_recording
    calibrate_neutral_pose(calibrate_gyro=True)
    start_time = time.monotonic()
    last_loop_time = start_time
    mem_write_idx = 0
    mem_sample_count = 0
    is_recording = True
    open_flash_log()
    send_status_event("EVENT:REC_ON")


def stop_recording_session():
    """Stops active recording session and flushes local log if open."""
    global is_recording
    is_recording = False
    close_flash_log()
    send_status_event("EVENT:REC_OFF")


def reset_system():
    """Resets timestamps, orientation filters, calibration, and memory buffer."""
    global start_time, last_loop_time, mem_write_idx, mem_sample_count, q_cal, is_calibrated, is_recording
    close_flash_log()
    is_recording = False
    calibrate_neutral_pose(calibrate_gyro=True)
    start_time = time.monotonic()
    last_loop_time = start_time
    mem_write_idx = 0
    mem_sample_count = 0
    send_status_event("EVENT:RESET_DONE")


def dump_memory_buffer():
    """Streams locally stored SRAM ring buffer samples to the laptop/iPhone dashboard."""
    print(f"# DUMP_START:{mem_sample_count}")
    for i in range(mem_sample_count):
        base = i * FIELDS_PER_SAMPLE
        t_v = mem_buffer[base]
        fe = mem_buffer[base + 1]
        ru = mem_buffer[base + 2]
        ps = mem_buffer[base + 3]
        a1 = mem_buffer[base + 4]
        a2 = mem_buffer[base + 5]
        g1 = mem_buffer[base + 6]
        g2 = mem_buffer[base + 7]
        line = f"MEM,{i},{t_v:.3f},{fe:.2f},{ru:.2f},{ps:.2f},{a1:.2f},{a2:.2f},{g1:.2f},{g2:.2f}\n"
        print(line, end="")
        if has_ble and ble.connected:
            try:
                uart_service.write(line.encode("utf-8"))
            except Exception:
                pass
        time.sleep(0.002)
    print("# DUMP_END")


def process_incoming_command(cmd_str):
    """Parses remote control commands received from the Web Dashboard via USB or BLE."""
    global is_streaming, active_screen, joint_name
    cmd = cmd_str.strip().upper()
    if not cmd:
        return
    if "CAL_POSE_ONLY" in cmd:
        calibrate_neutral_pose(calibrate_gyro=False)
    elif "CAL" in cmd:
        calibrate_neutral_pose(calibrate_gyro=True)
    elif "START_STREAM" in cmd or "STREAM_START" in cmd or "STREAM_ON" in cmd:
        is_streaming = True
        send_status_event("EVENT:STREAM_ON")
    elif "STOP_STREAM" in cmd or "STREAM_STOP" in cmd or "STREAM_OFF" in cmd:
        is_streaming = False
        send_status_event("EVENT:STREAM_OFF")
    elif "REC_START" in cmd or "START_REC" in cmd or "REC_ON" in cmd:
        start_recording_with_auto_cal()
    elif "REC_STOP" in cmd or "STOP_REC" in cmd or "REC_OFF" in cmd:
        stop_recording_session()
    elif "DUMP_MEM" in cmd:
        dump_memory_buffer()
    elif "RESET" in cmd:
        reset_system()
    elif "SCREEN" in cmd:
        active_screen = (active_screen + 1) % NUM_SCREENS
        send_status_event(f"EVENT:SCREEN_{active_screen}")
    elif "JOINT:" in cmd:
        joint_name = cmd.split("JOINT:")[-1].strip()[:10]
        send_status_event(f"EVENT:JOINT_{joint_name}")


# Perform initial startup neutral calibration so live angles begin at (0.0, 0.0, 0.0)
calibrate_neutral_pose(calibrate_gyro=True)
start_time = time.monotonic()
last_loop_time = start_time

# ============================================================================
# 10. MAIN HIGH-FREQUENCY ACQUISITION, DISPLAY & TELEMETRY LOOP
# ============================================================================
print("# XIAO 1.14 IPS Display (nRF52840) Dual ICM-20948 9-DOF Streamer Ready")
print("# USB Packet: D,t,ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,ax2,ay2,az2,gx2,gy2,gz2,mx2,my2,mz2,fe,ru,ps,vbat,rec")
print("# BLE Packet: B,t,fe,ru,ps,ax1,ay1,az1,ax2,ay2,az2,vbat,rec (25Hz iPhone NUS)")

usb_rx_buf = ""
ble_rx_buf = ""
ble_is_connected = False
force_display_update = True

while True:
    now = time.monotonic()
    dt = now - last_loop_time
    if dt <= 0.0 or dt > 0.25:
        dt = 0.005
    last_loop_time = now
    elapsed = now - start_time

    # 1. Poll physical buttons (BTN_A, BTN_B, BTN_C)
    ev_a = btn_a.poll(now)
    ev_b = btn_b.poll(now)
    ev_c = btn_c.poll(now)

    if ev_a == "SHORT":
        active_screen = (active_screen + 1) % NUM_SCREENS
        force_display_update = True
        send_status_event(f"EVENT:SCREEN_{active_screen}")
    elif ev_a == "LONG":
        calibrate_neutral_pose(calibrate_gyro=True)
        force_display_update = True

    if ev_b == "SHORT":
        is_streaming = not is_streaming
        force_display_update = True
        send_status_event("EVENT:STREAM_ON" if is_streaming else "EVENT:STREAM_OFF")
    elif ev_b == "LONG":
        reset_system()
        force_display_update = True

    if ev_c == "SHORT":
        if not is_recording:
            start_recording_with_auto_cal()
        else:
            stop_recording_session()
        force_display_update = True
    elif ev_c == "LONG":
        is_standby = not is_standby
        if backlight_pin:
            backlight_pin.value = not is_standby
        if imu1:
            imu1.sleep(is_standby)
        if imu2:
            imu2.sleep(is_standby)
        send_status_event("EVENT:STANDBY_ON" if is_standby else "EVENT:STANDBY_OFF")

    if is_standby:
        status_led.set_color(False, False, (int(now * 2) % 2 == 0))
        time.sleep(0.05)
        continue

    # 2. Check incoming remote commands from Laptop/iPhone (USB CDC & BLE UART)
    if supervisor.runtime.serial_bytes_available:
        try:
            ch = sys.stdin.read(1) if "sys" in globals() else __import__("sys").stdin.read(1)
            if ch in ("\n", "\r"):
                if usb_rx_buf:
                    process_incoming_command(usb_rx_buf)
                    usb_rx_buf = ""
                    force_display_update = True
            else:
                usb_rx_buf += ch
        except Exception:
            pass

    if has_ble:
        ble_is_connected = ble.connected

    if ble_is_connected and uart_service.in_waiting:
        try:
            raw_bytes = uart_service.read(uart_service.in_waiting)
            ble_rx_buf += raw_bytes.decode("utf-8", "ignore")
            while "\n" in ble_rx_buf:
                cmd_line, ble_rx_buf = ble_rx_buf.split("\n", 1)
                process_incoming_command(cmd_line)
                force_display_update = True
        except Exception:
            pass

    # 3. High-Speed 9-DOF I2C Burst Read of Sensor #1 (0x69) and Sensor #2 (0x68)
    if imu1:
        imu1.read_burst()
        imu1.update_stationary_bias()
        ax1, ay1, az1 = imu1.ax, imu1.ay, imu1.az
        gx1, gy1, gz1 = imu1.gx, imu1.gy, imu1.gz
        mx1, my1, mz1 = imu1.mx, imu1.my, imu1.mz
    else:
        ax1, ay1, az1 = 0.0, 0.0, 9.81
        gx1, gy1, gz1 = 0.0, 0.0, 0.0
        mx1, my1, mz1 = 25.0, 0.0, -40.0

    if imu2:
        imu2.read_burst()
        imu2.update_stationary_bias()
        ax2, ay2, az2 = imu2.ax, imu2.ay, imu2.az
        gx2, gy2, gz2 = imu2.gx, imu2.gy, imu2.gz
        mx2, my2, mz2 = imu2.mx, imu2.my, imu2.mz
    else:
        ax2, ay2, az2 = 0.0, 0.0, 9.81
        gx2, gy2, gz2 = 0.0, 0.0, 0.0
        mx2, my2, mz2 = 25.0, 0.0, -40.0

    # 4. Update 3D Orientation Quaternions & Relative 3D Joint Angles (Intrinsic Z-Y-X)
    q1 = ahrs1.update(ax1, ay1, az1, gx1, gy1, gz1, dt)
    q2 = ahrs2.update(ax2, ay2, az2, gx2, gy2, gz2, dt)

    # q_rel = conj(q1 * q_cal) * q2
    q1_aligned = quat_mul(q1, q_cal)
    q_rel = quat_norm(quat_mul(quat_conj(q1_aligned), q2))
    flex_ext, rad_uln, pro_sup = quat_to_euler_deg(q_rel)

    # 5. Store sample in local SRAM Ring Buffer (& Flash CSV if mounted writable)
    if is_recording:
        a1_mag = math.sqrt(ax1*ax1 + ay1*ay1 + az1*az1)
        a2_mag = math.sqrt(ax2*ax2 + ay2*ay2 + az2*az2)
        g1_mag = math.sqrt(gx1*gx1 + gy1*gy1 + gz1*gz1)
        g2_mag = math.sqrt(gx2*gx2 + gy2*gy2 + gz2*gz2)
        base = mem_write_idx * FIELDS_PER_SAMPLE
        mem_buffer[base] = elapsed
        mem_buffer[base + 1] = flex_ext
        mem_buffer[base + 2] = rad_uln
        mem_buffer[base + 3] = pro_sup
        mem_buffer[base + 4] = a1_mag
        mem_buffer[base + 5] = a2_mag
        mem_buffer[base + 6] = g1_mag
        mem_buffer[base + 7] = g2_mag
        mem_write_idx = (mem_write_idx + 1) % MAX_MEM_SAMPLES
        if mem_sample_count < MAX_MEM_SAMPLES:
            mem_sample_count += 1
        if flash_log_file and (mem_sample_count % 5 == 0):
            try:
                flash_log_file.write(
                    f"{elapsed:.3f},{flex_ext:.2f},{rad_uln:.2f},{pro_sup:.2f},"
                    f"{a1_mag:.2f},{a2_mag:.2f},{g1_mag:.1f},{g2_mag:.1f}\n"
                )
            except Exception:
                pass

    # 6. Stream full-speed 9-DOF telemetry over USB Serial & 25 Hz compact telemetry over BLE
    if is_streaming:
        rec_flag = 1 if is_recording else 0
        pkt = (
            f"D,{elapsed:.3f},"
            f"{ax1:.2f},{ay1:.2f},{az1:.2f},{gx1:.1f},{gy1:.1f},{gz1:.1f},{mx1:.1f},{my1:.1f},{mz1:.1f},"
            f"{ax2:.2f},{ay2:.2f},{az2:.2f},{gx2:.1f},{gy2:.1f},{gz2:.1f},{mx2:.1f},{my2:.1f},{mz2:.1f},"
            f"{flex_ext:.1f},{rad_uln:.1f},{pro_sup:.1f},"
            f"{battery_mon.voltage:.2f},{rec_flag}\n"
        )
        print(pkt, end="")

        if ble_is_connected and (now - last_ble_time) >= 0.04:
            last_ble_time = now
            ble_pkt = (
                f"B,{elapsed:.2f},{flex_ext:.1f},{rad_uln:.1f},{pro_sup:.1f},"
                f"{ax1:.2f},{ay1:.2f},{az1:.2f},{ax2:.2f},{ay2:.2f},{az2:.2f},"
                f"{battery_mon.voltage:.2f},{rec_flag}\n"
            )
            try:
                uart_service.write(ble_pkt.encode("utf-8"))
            except Exception:
                pass

    # 7. Update Sampling Rate Counter (Hz), BLE Advertising & Status LED (1 Hz)
    loop_count += 1
    if (now - last_hz_time) >= 1.0:
        measured_hz = loop_count
        loop_count = 0
        last_hz_time = now
        if has_ble:
            ble_is_connected = ble.connected
            if not ble_is_connected and not ble.advertising:
                try:
                    ble.start_advertising(advertisement)
                except Exception:
                    pass
        if is_recording:
            status_led.set_color(True, False, False)
        elif is_streaming:
            status_led.set_color(False, True, False)
        else:
            status_led.set_color(False, False, True)

    # 8. Periodic Battery Monitor Update (every 3.0 s)
    if (now - last_batt_time) >= 3.0:
        battery_mon.update()
        last_batt_time = now

    # 9. Non-blocking Landscape Display Refresh (every 0.4s after previous refresh completes)
    if display and (force_display_update or (now - last_display_time) >= 0.40):
        force_display_update = False
        cal_tag = "CAL" if is_calibrated else "RAW"
        if active_screen == 0:
            new_hdr = f"1/4 {joint_name} 3D ANGLES [{cal_tag}]"
            new_l1 = f"Flx/Ext:{flex_ext:+6.1f}d"
            new_l2 = f"Var/Val:{rad_uln:+6.1f}d"
            new_l3 = f"Int/Ext:{pro_sup:+6.1f}d"
        elif active_screen == 1:
            new_hdr = "2/4 S1(0x69) vs S2(0x68) IMU"
            new_l1 = f"S1A:{ax1:+4.1f} {ay1:+4.1f} {az1:+4.1f}"
            new_l2 = f"S2A:{ax2:+4.1f} {ay2:+4.1f} {az2:+4.1f}"
            new_l3 = f"G12:{gz1:+4.0f}/{gz2:+4.0f} dps"
        elif active_screen == 2:
            new_hdr = "3/4 LOCAL MEMORY & RECORDING"
            new_l1 = f"REC: {'ACTIVE' if is_recording else 'READY '}"
            new_l2 = f"MEM: {mem_sample_count}/{MAX_MEM_SAMPLES}"
            new_l3 = f"TIME:{elapsed:7.1f}s"
        else:
            chg_txt = "CHG" if battery_mon.charging else "BAT"
            ble_txt = "BLE:CON" if ble_is_connected else "USB/ADV"
            new_hdr = "4/4 POWER & SYSTEM HEALTH"
            new_l1 = f"{chg_txt}: {battery_mon.voltage:.2f}V ({battery_mon.percent}%)"
            new_l2 = f"LINK: {ble_txt}"
            new_l3 = f"RATE: {measured_hz} Hz"

        strm_tag = "ON" if is_streaming else "PAUSED"
        rec_tag = "REC" if is_recording else "IDLE"
        new_ftr = f"STRM:{strm_tag} | {rec_tag} | {measured_hz}Hz | {battery_mon.percent}%"

        if lbl_header.text != new_hdr: lbl_header.text = new_hdr
        if lbl_line1.text != new_l1: lbl_line1.text = new_l1
        if lbl_line2.text != new_l2: lbl_line2.text = new_l2
        if lbl_line3.text != new_l3: lbl_line3.text = new_l3
        if lbl_footer.text != new_ftr: lbl_footer.text = new_ftr
        try:
            display.refresh(minimum_frames_per_second=0)
        except Exception:
            pass
        last_display_time = time.monotonic()
