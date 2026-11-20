# SPDX-FileCopyrightText: 2026 Eric G. Meyer / LTU Experimental Biomechanics Lab (EBL)
# SPDX-License-Identifier: MIT
"""
boot.py - Seeed Studio XIAO 1.14" IPS Display (nRF52840 Plus)
=============================================================
Configures USB CDC Serial and optional local Flash filesystem write access
for local CSV trial logging on the 2 MB onboard QSPI Flash.

How it works:
  - By default in CircuitPython, if the laptop has CIRCUITPY mounted as Read-Write,
    code.py cannot write files locally to Flash.
  - Hold BTN_C (Side Button, P1.07 / D19) during boot/reset to mount the onboard
    Flash as writable by CircuitPython (`storage.remount("/", readonly=False)`),
    enabling standalone untethered CSV file logging to `/recordings/`.
  - Normal boot (no button held) keeps CIRCUITPY editable over USB from your laptop,
    while `code.py` still stores high-speed recordings in its pre-allocated SRAM
    Ring Buffer for instant wireless/USB download (`CMD:DUMP_MEM`).
"""

import digitalio
import microcontroller
import storage

try:
    # BTN_C (Side Button on XIAO 1.14" IPS Display expansion board = P1.07 / D19)
    btn_c = digitalio.DigitalInOut(microcontroller.pin.P1_07)
    btn_c.direction = digitalio.Direction.INPUT
    btn_c.pull = digitalio.Pull.UP

    # Active-low: if BTN_C is held LOW during boot, enable onboard Flash file writing
    if not btn_c.value:
        storage.remount("/", readonly=False)
    btn_c.deinit()
except Exception:
    pass
