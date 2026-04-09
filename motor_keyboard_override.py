#!/usr/bin/env python3
"""
motor_keyboard_encoder_gpiozero.py

- Drive motor with left/right arrow keys (hold to run).
- If no key is pressed recently, motor is OFF.
- Reads LS7366R encoder counts over SPI and shows count + estimated speed.

Uses gpiozero instead of RPi.GPIO for motor control.
"""

import time
import curses
import spidev
from dataclasses import dataclass
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# ---------------------------
# User settings
# ---------------------------

# Motor GPIO
PWM_PIN = 12      # BCM
INA_PIN = 24      # BCM
INB_PIN = 25      # BCM
PWM_FREQUENCY_HZ = 2000

# Encoder SPI (LS7366R)
SPI_BUS = 0
SPI_DEV = 0       # 0=CE0, 1=CE1
SPI_MAX_HZ = 1_000_000

# Control behavior
DUTY_STEP = 5          # % per key repeat
DUTY_MAX = 80          # %
DEADMAN_TIMEOUT = 0.20 # seconds since last keypress to force motor OFF
UI_HZ = 20             # screen refresh rate

# Encoder interpretation
ENCODER_CPR = 1024
QUAD_MULT = 4          # if LS7366R set to x4 quadrature

# ---------------------------
# LS7366R minimal driver
# ---------------------------

OP_CLR  = 0b00
OP_RD   = 0b01
OP_WR   = 0b10
OP_LOAD = 0b11

REG_MDR0 = 0b001
REG_MDR1 = 0b010
REG_CNTR = 0b100
REG_STR  = 0b110

def _instr(op: int, reg: int) -> int:
    return ((op & 0b11) << 6) | ((reg & 0b111) << 3)

# MDR0 bits
QUAD_X4 = 0b11
CM_FREE = 0b00
IDX_DISABLE = 0b00
IDX_ASYNC = 0
FILT_DIV1 = 0

# MDR1 bits
BW_4 = 0b00
COUNT_ENABLE = 0

@dataclass
class LS7366RConfig:
    quad_mode: int = QUAD_X4
    count_mode: int = CM_FREE
    index_mode: int = IDX_DISABLE
    index_sync: int = IDX_ASYNC
    filter_div2: int = FILT_DIV1
    byte_width: int = BW_4
    disable_counting: bool = False
    enable_flags_mask: int = 0x00

class LS7366R:
    def __init__(self, bus: int, device: int, max_speed_hz: int):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = 0b00
        self.spi.max_speed_hz = max_speed_hz
        self.spi.bits_per_word = 8
        self._nbytes = 4

    def close(self):
        try:
            self.spi.close()
        except Exception:
            pass

    def _xfer(self, tx):
        return self.spi.xfer2(tx)

    def _write_reg(self, reg_sel: int, data_bytes):
        self._xfer([_instr(OP_WR, reg_sel)] + list(data_bytes))

    def _read_reg(self, reg_sel: int, nbytes: int):
        rx = self._xfer([_instr(OP_RD, reg_sel)] + [0x00] * nbytes)
        return rx[1:]

    def _clear_reg(self, reg_sel: int):
        self._xfer([_instr(OP_CLR, reg_sel)])

    def configure(self, cfg: LS7366RConfig):
        mdr0 = 0
        mdr0 |= (cfg.quad_mode & 0b11)
        mdr0 |= (cfg.count_mode & 0b11) << 2
        mdr0 |= (cfg.index_mode & 0b11) << 4
        mdr0 |= (1 if cfg.index_sync else 0) << 6
        mdr0 |= (1 if cfg.filter_div2 else 0) << 7

        mdr1 = 0
        mdr1 |= (cfg.byte_width & 0b11)
        mdr1 |= (1 if cfg.disable_counting else 0) << 2
        mdr1 |= (cfg.enable_flags_mask & 0xF0)

        self._write_reg(REG_MDR0, [mdr0])
        self._write_reg(REG_MDR1, [mdr1])
        self._nbytes = 4

    def clear_cntr(self):
        self._clear_reg(REG_CNTR)

    def read_str(self) -> int:
        return self._read_reg(REG_STR, 1)[0]

    @staticmethod
    def _be_bytes_to_int(data):
        v = 0
        for b in data:
            v = (v << 8) | (b & 0xFF)
        return v

    def read_cntr(self, signed: bool = True) -> int:
        data = self._read_reg(REG_CNTR, self._nbytes)
        raw = self._be_bytes_to_int(data)
        if not signed:
            return raw
        bits = 8 * self._nbytes
        if raw & (1 << (bits - 1)):
            raw -= 1 << bits
        return raw

# ---------------------------
# Motor control with gpiozero
# ---------------------------

def motor_init():
    ina = DigitalOutputDevice(INA_PIN, active_high=True, initial_value=False)
    inb = DigitalOutputDevice(INB_PIN, active_high=True, initial_value=False)

    # initial_value is 0.0..1.0 for PWMOutputDevice
    pwm = PWMOutputDevice(
        PWM_PIN,
        active_high=True,
        initial_value=0.0,
        frequency=PWM_FREQUENCY_HZ
    )
    return ina, inb, pwm

def safe_addstr(win, y, x, s):
    h, w = win.getmaxyx()
    if y < 0 or y >= h:
        return
    if x < 0:
        x = 0
    max_len = max(0, w - x - 1)
    win.addnstr(y, x, s, max_len)

def motor_set(ina, inb, pwm, direction: int, duty: int):
    """
    direction: -1 = left (INB=1), +1 = right (INA=1), 0 = off
    duty: 0..100
    """
    duty = max(0, min(100, int(duty)))
    pwm_value = duty / 100.0

    if direction == 0 or duty == 0:
        ina.off()
        inb.off()
        pwm.value = 0.0
        return

    if direction > 0:
        ina.on()
        inb.off()
    else:
        ina.off()
        inb.on()

    pwm.value = pwm_value

def motor_cleanup(ina, inb, pwm):
    try:
        pwm.value = 0.0
    except Exception:
        pass

    for dev in (pwm, ina, inb):
        try:
            dev.close()
        except Exception:
            pass

# ---------------------------
# UI / main loop
# ---------------------------

def run(stdscr):
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.curs_set(0)

    ina, inb, pwm = motor_init()
    enc = LS7366R(SPI_BUS, SPI_DEV, SPI_MAX_HZ)

    try:
        enc.configure(LS7366RConfig())
        enc.clear_cntr()

        duty = 0
        direction = 0
        last_key_t = 0.0

        last_count = enc.read_cntr()
        last_t = time.time()

        ui_period = 1.0 / max(1, UI_HZ)

        while True:
            now = time.time()

            key = stdscr.getch()
            if key != -1:
                last_key_t = now

                if key in (ord('q'), ord('Q')):
                    break
                elif key == curses.KEY_RIGHT:
                    direction = +1
                    duty = min(DUTY_MAX, duty + DUTY_STEP)
                elif key == curses.KEY_LEFT:
                    direction = -1
                    duty = min(DUTY_MAX, duty + DUTY_STEP)
                elif key == ord(' '):
                    direction = 0
                    duty = 0
                elif key in (ord('c'), ord('C')):
                    enc.clear_cntr()
                    last_count = enc.read_cntr()
                    last_t = now
                elif key == curses.KEY_UP:
                    duty = min(DUTY_MAX, duty + DUTY_STEP)
                elif key == curses.KEY_DOWN:
                    duty = max(0, duty - DUTY_STEP)

            if (now - last_key_t) > DEADMAN_TIMEOUT:
                direction = 0
                duty = 0

            motor_set(ina, inb, pwm, direction, duty)

            count = enc.read_cntr()
            dt = now - last_t
            dc = count - last_count
            cps = (dc / dt) if dt > 1e-6 else 0.0

            counts_per_rev = max(1, ENCODER_CPR * QUAD_MULT)
            rps = cps / counts_per_rev
            rpm = rps * 60.0

            last_count = count
            last_t = now

            stdscr.erase()
            safe_addstr(stdscr, 0, 0, "Motor + Encoder Control (LS7366R + gpiozero PWM)")
            safe_addstr(stdscr, 2, 0, "Hold LEFT/RIGHT arrows to run. Release -> OFF (deadman).")
            safe_addstr(stdscr, 3, 0, "UP/DOWN adjust duty. SPACE stops. C clears count. Q quits.")
            safe_addstr(stdscr, 5, 0, f"Direction: {'RIGHT' if direction > 0 else ('LEFT' if direction < 0 else 'OFF')}")
            safe_addstr(stdscr, 6, 0, f"Duty:      {duty:3d} % (max {DUTY_MAX}%)")
            safe_addstr(stdscr, 8, 0, f"Count:     {count}")
            safe_addstr(stdscr, 9, 0, f"Counts/s:  {cps:0.2f}")
            safe_addstr(stdscr, 10, 0, f"RPM:       {rpm:0.2f}")

            h, w = stdscr.getmaxyx()
            if h >= 13:
                safe_addstr(stdscr, 12, 0, f"CPR={ENCODER_CPR}, QUAD={QUAD_MULT}, counts/rev={counts_per_rev}")

            stdscr.refresh()
            time.sleep(ui_period)

    finally:
        try:
            motor_set(ina, inb, pwm, 0, 0)
        except Exception:
            pass
        enc.close()
        motor_cleanup(ina, inb, pwm)

def main():
    curses.wrapper(run)

if __name__ == "__main__":
    main()