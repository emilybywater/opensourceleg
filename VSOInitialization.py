import json
import select
import sys
import termios
import time
import tty
from collections import deque
from pathlib import Path
from typing import Optional
import numpy as np

from opensourceleg.logging import LOGGER

from opensourceleg.robots.vso import VSO
from opensourceleg.sensors.adc import ChannelConfig
from calibration import VSOCalibration
from sliderPosition import sliderPosition

DEFAULT_CALIB_OFFSET_PATH = Path("vso_calib_offset.json")
DEFAULT_HALL_THRESHOLD_PATH = Path("hall_switch_thresholds.json")


class VSOInitialization:
    """
    Orchestrates VSO startup sequence:
        1. Optional stroke calibration (0→100→0) to compute and save scale_perc.
        2. Encoder homing — drive to soft stop, zero encoder, move to 100%.
        3. Ankle encoder offset calibration — capture unloaded equilibrium angle
           at 100% stiffness and save calib_offset to file.

    Args:
        vso: The VSO instance to initialize.
        calibration_path: Path to the stroke calibration JSON file (scale_perc).
        calib_offset_path: Path to save the ankle encoder offset JSON file.
        homing_pwm: PWM value used for homing and calibration moves.
        sample_rate: Time in seconds between position samples during homing.
        position_threshold: Max position delta (±) to consider motor stopped.

    Example:
        init = VSOInitialization(vso=my_vso)
        init.run(run_calibration=False)  # normal power cycle
        init.run(run_calibration=True)   # after reassembly
    """

    def __init__(
        self,
        vso: VSO,
        calibration_path: Path = Path("vso_calibration.json"),
        calib_offset_path: Path = DEFAULT_CALIB_OFFSET_PATH,
        hall_threshold_path: Path = DEFAULT_HALL_THRESHOLD_PATH,
        homing_pwm: float = 0.25,
        sample_rate: float = 0.05,
        position_threshold: int = 100,
        side: int = 1, # 1 for left leg lateral encoder (-1 if medial), -1 for right leg lateral encoder (1 if medial)
        bat_div_gain: float = 2.0,
    ) -> None:

        self.vso = vso
        self.calibration_path = calibration_path
        self.calib_offset_path = calib_offset_path
        self.hall_threshold_path = Path(hall_threshold_path)
        self.homing_pwm = homing_pwm
        self.sample_rate = sample_rate
        self.position_threshold = position_threshold
        self.side = side
        self.bat_div_gain = bat_div_gain
        self.calib_offset = 0.0

        LOGGER.info("VSO Initialization instance created.")

    def run(self, run_calibration: bool = False, run_hall_calibration: bool = False) -> None:
        """
        Run the full VSO initialization sequence.

        Args:
            run_calibration: If True, runs stroke calibration to compute scale_perc before homing.
                Use after disassembly or first-time setup.
            run_hall_calibration: If True, runs interactive hall switch threshold calibration
                after the ankle encoder offset is captured.  Saves results to hall_threshold_path.
        """
        if self.vso.actuators.get("ankle", None) is not None:
            actuator = self.vso.actuators["ankle"]
        else:
            actuator = None

        if self.vso.sensors.get("motor_encoder", None) is not None:
            encoder_counter = self.vso.sensors["motor_encoder"]
        else:
            encoder_coutner = None
        
        if self.vso.sensors.get("ankle_encoder", None) is not None:
            ankle_sensor = self.vso.sensors["ankle_encoder"]
        else:
            ankle_sensor = None

        if self.vso.sensors.get("adc", None) is not None:
            adc = self.vso.sensors["adc"]
            adc.adc_configure_common(single_shot=True, filter_low_latency=True)
            # readback = adc.read_single_register(adc._REG_ADDR_DATARATE)
            # print(f"DATARATE register: 0x{readback:02X}  (expected 0x3C)")

            
            if self.vso.sensors.get("hallEffect_1", None) is not None:
                hall1 = self.vso.sensors.get("hallEffect_1")
                hall1.configure()
                adc._channels["hall_drv5056_ain3"] = ChannelConfig(
                    name="hall_drv5056_ain3",
                    ain_pos_code=adc._ADS_P_AIN3,
                    postprocess=None,
                    units="V"
                )
                    
            if self.vso.sensors.get("hallEffect_2", None) is not None:
                hall2 = self.vso.sensors.get("hallEffect_2")
                hall2.configure()
                adc._channels["hall_drv5056_ain4"] = ChannelConfig(
                    name="hall_drv5056_ain4",
                    ain_pos_code=adc._ADS_P_AIN4,
                    postprocess=None,
                    units="V"
                )

            if self.vso.sensors.get("battery_monitor", None) is not None:
                battery_monitor = self.vso.sensors.get("battery_monitor")
                adc._channels["battery"] = ChannelConfig(
                    name="battery",
                    ain_pos_code=adc._ADS_P_AIN0,
                    postprocess=lambda v: battery_postprocess(v, self.bat_div_gain),
                    units="V"
                )
                

        else:
            adc = None
        
        time.sleep(0.05)  
 
        if actuator is not None and encoder_counter is not None:
            calibration = VSOCalibration(
                vso=self.vso,
                actuator=actuator,
                encoder=encoder_counter,
                calibration_path=self.calibration_path,
            )

            # Step 1: Optional stroke calibration
            if run_calibration:
                LOGGER.info("Running stroke calibration.")
                scale_perc = calibration.run(
                    homing_pwm=self.homing_pwm,
                    sample_rate=self.sample_rate,
                    position_threshold=self.position_threshold,
                )
                LOGGER.info(f"Stroke calibration complete. scale_perc={scale_perc:.2f}")
            else:
                LOGGER.info("Skipping stroke calibration. Assuming calibration file exists.")

            # Step 2: Encoder homing — zero at soft stop%
            LOGGER.info("Homing to soft stop and zeroing encoder.")
            self.vso.home(
                homing_pwm=self.homing_pwm,
                sample_rate=self.sample_rate,
                position_threshold=self.position_threshold,
                home_zero=True
            )
        
            encoder_counter.clearCounter()
            time.sleep(1.5)  # Ensure encoder clear is seen before moving
            LOGGER.info(f"Encoder zeroed.")

            LOGGER.info("Moving spring-support to stiffest position (100%).")
            scale_perc = calibration.load()

            actuator.position_control_init()
            actuator.position_control_config(scale_perc=scale_perc)

            sliderPosition.slider_position(actuator, desired_position_perc=99.5)

        if ankle_sensor is not None:
            # Step 3: Ankle encoder offset calibration at 100% stiffness
            LOGGER.info("Capturing unloaded equilibrium angle. Waiting for ankle encoder warmup.")
            time.sleep(2)  # Warmup period for ankle encoder to stabilize
            ankle_sensor.update()
            self.calib_offset = self.side*np.rad2deg(ankle_sensor.position)
            self._save_calib_offset()
            LOGGER.info(f"Ankle encoder offset calibrated. calib_offset={self.calib_offset:.4f} deg")
        else:
            LOGGER.info(f"No ankle encoder. Could not capture unloaded equilibrium angle.")

        # Step 4: Optional hall switch threshold calibration
        if run_hall_calibration:
            LOGGER.info("Running hall switch calibration.")
            self.calibrate_hall_switches()
            LOGGER.info("Hall switch calibration complete.")
        else:
            LOGGER.info("Skipping hall switch calibration. Assuming threshold file exists.")

        LOGGER.info("VSO initialization complete.")

    def battery_postprocess(volts_at_adc: float, bat_div_gain: float) -> float:
        """Convert ADC-pin voltage to battery voltage (undo divider)."""
        return volts_at_adc * bat_div_gain

    def _save_calib_offset(self) -> None:
        """
        Save the ankle encoder calibration offset to file.

        Args:
            calib_offset: The unloaded equilibrium angle in radians.
        """
        with open(self.calib_offset_path, "w") as f:
            json.dump({"calib_offset": self.calib_offset}, f, indent=2)
        LOGGER.info(f"calib_offset saved to {self.calib_offset_path}")

    def load_calib_offset(self) -> None:
        """
        Load the ankle encoder calibration offset from file.

        Returns:
            calib_offset: The unloaded equilibrium angle in radians.

        Raises:
            FileNotFoundError: If no calib_offset file exists at the specified path.
        """
        if not self.calib_offset_path.exists():
            raise FileNotFoundError(
                f"No calib_offset file found at {self.calib_offset_path}. "
                "Run VSOInitialization.run() first."
            )
        with open(self.calib_offset_path, "r") as f:
            data = json.load(f)
        self.calib_offset = data["calib_offset"]
        LOGGER.info(f"Loaded calib_offset: {self.calib_offset:.4f} from {self.calib_offset_path}")

    def calibrate_hall_switches(
        self,
        frequency: int = 200,
        n_std_hall: float = 2.0,
        n_std_angle: float = 2.0,
        min_margin_v: float = 0.02,
        min_margin_deg: float = 1.0,
    ) -> dict:
        """
        Interactive calibration of hall-effect switch thresholds.

        Runs a sensor-read loop and records hall1, hall2, ankle angle, and
        their sample-to-sample derivatives at each labelled switch event.
        When finished, thresholds are derived as (min - margin, max + margin)
        across all events of each type and written to self.hall_threshold_path.

        Controls during the loop:
            d      — dorsiflexion switch just fired
            p      — plantarflexion switch just fired
            ENTER  — finish and save thresholds

        Call this after init.run() so that self.calib_offset is already set.

        Args:
            frequency:  Sensor-read rate in Hz (default 200).
            margin_v:   Voltage added to each bound as a detection margin (default 0.05 V).

        Returns:
            Threshold dict (same structure written to JSON).

        Raises:
            RuntimeError: If the ADC sensor is not present.
            ValueError:   If fewer than 1 event of either switch type was captured.
        """
        adc = self.vso.sensors.get("adc")
        ankle_sensor = self.vso.sensors.get("ankle_encoder")

        if adc is None:
            raise RuntimeError("ADC sensor not found in VSO — cannot calibrate hall switches.")

        dorsi_events = []    # list of {hall1, hall2, hall1_dot, hall2_dot, angle_dot}
        plantar_events = []

        dt = 1.0 / frequency
        encoder_alpha = 0.15  # EMA smoothing factor (~5 Hz cutoff at 200 Hz)

        # Window sizing: 0.4 s pre-keypress lookback, 0.2 s post-keypress lookahead
        pre_samples  = int(0.4 * frequency)
        post_samples = int(0.2 * frequency)

        angle_filt = 0.0
        angle_last = 0.0
        hall1_last = 0.0
        hall2_last = 0.0

        rolling_buf = deque(maxlen=pre_samples)  # continuous pre-window
        pending_key  = None   # 'd' or 'p' while collecting post-window
        post_buf     = []
        post_remain  = 0

        print()
        print("=" * 60)
        print("  Hall Switch Calibration")
        print("=" * 60)
        print("  Walk the exoskeleton through dorsi/plantarflexion cycles.")
        print("  Press a key near the time each mechanical switch fires —")
        print("  the peak hall derivative in the surrounding window is used.")
        print()
        print("    d     = dorsiflexion switch")
        print("    p     = plantarflexion switch")
        print("    ENTER = done — compute and save thresholds")
        print("=" * 60)
        print()

        def _read_sensors():
            self.vso.update()
            data = getattr(adc, "_data", [0.0, 0.0])
            h1 = data[0] / 1000.0 if len(data) > 0 else 0.0
            h2 = data[1] / 1000.0 if len(data) > 1 else 0.0
            if ankle_sensor is not None:
                ankle_sensor.update()
                ang = self.side * np.rad2deg(ankle_sensor.position) - self.calib_offset
            else:
                ang = 0.0
            return h1, h2, ang

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        t_loop_start = time.monotonic()

        try:
            tty.setcbreak(fd)

            while True:
                t_iter = time.monotonic()

                hall1, hall2, angle_raw = _read_sensors()
                angle_filt = encoder_alpha * angle_raw + (1 - encoder_alpha) * angle_filt
                hall1_dot = hall1 - hall1_last
                hall2_dot = hall2 - hall2_last
                angle_dot = angle_filt - angle_last
                angle = angle_filt

                sample = {
                    "hall1": hall1,         "hall2": hall2,
                    "hall1_dot": hall1_dot, "hall2_dot": hall2_dot,
                    "angle": angle,         "angle_dot": angle_dot,
                }

                # --- Post-window collection ---
                if post_remain > 0:
                    post_buf.append(sample)
                    post_remain -= 1

                    if post_remain == 0:
                        # Find the sample in the combined window with the largest
                        # total hall derivative — that is when the switch actually fired.
                        window = list(rolling_buf) + post_buf
                        peak = max(
                            window,
                            key=lambda s: abs(s["hall1_dot"]) + abs(s["hall2_dot"]),
                        )
                        if pending_key == 'd':
                            dorsi_events.append(peak)
                            label = f"DORSI  #{len(dorsi_events):02d}"
                        else:
                            plantar_events.append(peak)
                            label = f"PLANTAR #{len(plantar_events):02d}"
                        print(
                            f"\n  [{label}]"
                            f"  hall1={peak['hall1']:+.4f} V  hall2={peak['hall2']:+.4f} V"
                            f"  d(h1)={peak['hall1_dot']:+.5f}  d(h2)={peak['hall2_dot']:+.5f}"
                            f"  d(ang)={peak['angle_dot']:+.4f}"
                        )
                        pending_key = None
                        post_buf = []

                # Always push to rolling buffer (used as pre-window for future keypresses)
                rolling_buf.append(sample)

                # --- Keypress check ---
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)

                    if key in ('\n', '\r'):
                        if post_remain > 0:
                            print("\n  (Discarding incomplete window — press ENTER again to finish.)")
                        else:
                            print("\n\nFinishing calibration...")
                            break

                    elif key.lower() in ('d', 'p') and pending_key is None:
                        pending_key = key.lower()
                        post_remain = post_samples
                        post_buf = []

                else:
                    elapsed = time.monotonic() - t_loop_start
                    status = f"collecting +{post_samples - post_remain}/{post_samples}" if post_remain > 0 else f"dorsi={len(dorsi_events)} plantar={len(plantar_events)}"
                    print(
                        f"\r  t={elapsed:6.1f}s"
                        f"  angle={angle:+7.2f}°"
                        f"  h1={hall1:+.4f} V  h2={hall2:+.4f} V"
                        f"  d(h1)={hall1_dot:+.5f}"
                        f"  [{status}]   ",
                        end="",
                        flush=True,
                    )

                angle_last = angle_filt
                hall1_last = hall1
                hall2_last = hall2

                remaining = dt - (time.monotonic() - t_iter)
                if remaining > 0:
                    time.sleep(remaining)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if not dorsi_events or not plantar_events:
            raise ValueError(
                f"Insufficient calibration data — dorsi={len(dorsi_events)} events, "
                f"plantar={len(plantar_events)} events. Need at least 1 of each."
            )

        def _bounds_v(values):
            """mean ± n_std_hall*σ, with a minimum half-width of min_margin_v."""
            a = np.array(values)
            half = max(n_std_hall * float(a.std()), min_margin_v)
            return float(a.mean()) - half, float(a.mean()) + half

        def _bounds_deg(values):
            """mean ± n_std_angle*σ, with a minimum half-width of min_margin_deg."""
            a = np.array(values)
            half = max(n_std_angle * float(a.std()), min_margin_deg)
            return float(a.mean()) - half, float(a.mean()) + half

        def _dot_upper(values):
            """Upper bound on abs derivative: mean + n_std_hall*σ, floored at min_margin_v."""
            a = np.abs(values)
            return float(a.mean()) + max(n_std_hall * float(a.std()), min_margin_v)

        def _summary_v(values, lo, hi):
            a = np.array(values)
            return f"[{lo:.4f}, {hi:.4f}] V  (μ={a.mean():.4f} σ={a.std():.4f})"

        def _summary_deg(values, lo, hi):
            a = np.array(values)
            return f"[{lo:.2f}, {hi:.2f}]°  (μ={a.mean():.2f} σ={a.std():.2f})"

        d_h1      = [e["hall1"]     for e in dorsi_events]
        d_h2      = [e["hall2"]     for e in dorsi_events]
        d_h1_dot  = [e["hall1_dot"] for e in dorsi_events]
        d_ang     = [e["angle"]     for e in dorsi_events]
        d_ang_dot = [e["angle_dot"] for e in dorsi_events]

        p_h1      = [e["hall1"]     for e in plantar_events]
        p_h2      = [e["hall2"]     for e in plantar_events]
        p_ang     = [e["angle"]     for e in plantar_events]
        p_ang_dot = [e["angle_dot"] for e in plantar_events]

        d_h1_lo,  d_h1_hi  = _bounds_v(d_h1)
        d_h2_lo,  d_h2_hi  = _bounds_v(d_h2)
        d_ang_lo, d_ang_hi = _bounds_deg(d_ang)
        p_h1_lo,  p_h1_hi  = _bounds_v(p_h1)
        p_h2_lo,  p_h2_hi  = _bounds_v(p_h2)
        p_ang_lo, p_ang_hi = _bounds_deg(p_ang)

        thresholds = {
            "dorsiflexion": {
                "hall1_lower": d_h1_lo,
                "hall1_upper": d_h1_hi,
                "hall2_lower": d_h2_lo,
                "hall2_upper": d_h2_hi,
                "hall1_dot_upper": _dot_upper(d_h1_dot),
                "angle_lower": d_ang_lo,
                "angle_upper": d_ang_hi,
                "angle_dot_sign": 1 if float(np.mean(d_ang_dot)) >= 0 else -1,
                "n_events": len(dorsi_events),
            },
            "plantarflexion": {
                "hall1_lower": p_h1_lo,
                "hall1_upper": p_h1_hi,
                "hall2_lower": p_h2_lo,
                "hall2_upper": p_h2_hi,
                "angle_lower": p_ang_lo,
                "angle_upper": p_ang_hi,
                "angle_dot_sign": 1 if float(np.mean(p_ang_dot)) >= 0 else -1,
                "n_events": len(plantar_events),
            },
        }

        with open(self.hall_threshold_path, "w") as f:
            json.dump(thresholds, f, indent=2)

        d = thresholds["dorsiflexion"]
        p = thresholds["plantarflexion"]
        print(f"\nThresholds saved to {self.hall_threshold_path}  (n_std_hall={n_std_hall}  n_std_angle={n_std_angle})")
        print(f"  Dorsiflexion  ({d['n_events']} events):")
        print(f"    hall1  {_summary_v(d_h1,  d_h1_lo,  d_h1_hi)}")
        print(f"    hall2  {_summary_v(d_h2,  d_h2_lo,  d_h2_hi)}")
        print(f"    angle  {_summary_deg(d_ang, d_ang_lo, d_ang_hi)}")
        print(f"    d(h1)_upper = {d['hall1_dot_upper']:.4f}")
        print(f"  Plantarflexion ({p['n_events']} events):")
        print(f"    hall1  {_summary_v(p_h1,  p_h1_lo,  p_h1_hi)}")
        print(f"    hall2  {_summary_v(p_h2,  p_h2_lo,  p_h2_hi)}")
        print(f"    angle  {_summary_deg(p_ang, p_ang_lo, p_ang_hi)}")
        LOGGER.info(f"Hall switch thresholds saved to {self.hall_threshold_path}")

        return thresholds

    def load_hall_thresholds(self) -> dict:
        """
        Load hall-effect switch thresholds from file.

        Returns:
            Threshold dict with 'dorsiflexion' and 'plantarflexion' keys.

        Raises:
            FileNotFoundError: If the threshold file does not exist.
        """
        if not self.hall_threshold_path.exists():
            raise FileNotFoundError(
                f"No hall threshold file found at {self.hall_threshold_path}. "
                "Run calibrate_hall_switches() first."
            )
        with open(self.hall_threshold_path, "r") as f:
            thresholds = json.load(f)
        LOGGER.info(f"Loaded hall switch thresholds from {self.hall_threshold_path}")
        return thresholds


if __name__ == "__main__":
    pass
