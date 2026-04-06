import json
import time
from pathlib import Path
from typing import Optional
import numpy as np

from opensourceleg.logging import LOGGER

from opensourceleg.robots.vso import VSO
from opensourceleg.sensors.adc import ChannelConfig
from calibration import VSOCalibration
from sliderPosition import sliderPosition

DEFAULT_CALIB_OFFSET_PATH = Path("vso_calib_offset.json")


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
        homing_pwm: float = 0.25,
        sample_rate: float = 0.05,
        position_threshold: int = 100,
        side: int = 1, # 1 for left leg lateral encoder (-1 if medial), -1 for right leg lateral encoder (1 if medial)
        bat_div_gain: float = 2.0,
    ) -> None:

        self.vso = vso
        self.calibration_path = calibration_path
        self.calib_offset_path = calib_offset_path
        self.homing_pwm = homing_pwm
        self.sample_rate = sample_rate
        self.position_threshold = position_threshold
        self.side = side
        self.bat_div_gain = bat_div_gain

        LOGGER.info("VSO Initialization instance created.")

    def run(self, run_calibration: bool = False) -> None:
        """
        Run the full VSO initialization sequence.

        Args:
            run_calibration: If True, runs stroke calibration to compute scale_perc before homing. 
            Use after disassembly or first-time setup. 
            If False, assumes calibration file exists.
        """
        if self.vso.sensors.get("ankle", None) is not None:
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
            adc.adc_configure_common(single_shot=True,filter_low_latency=False)
            
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
                calib_offset = self.side*np.rad2deg(ankle_sensor.position)
                self._save_calib_offset(calib_offset)
                LOGGER.info(f"Ankle encoder offset calibrated. calib_offset={calib_offset:.4f} deg")
            else:
                LOGGER.info(f"No ankle encoder. Could not capture unloaded equilibrium angle.")


        LOGGER.info("VSO initialization complete.")

    def battery_postprocess(volts_at_adc: float, bat_div_gain: float) -> float:
        """Convert ADC-pin voltage to battery voltage (undo divider)."""
        return volts_at_adc * bat_div_gain

    def _save_calib_offset(self, calib_offset: float) -> None:
        """
        Save the ankle encoder calibration offset to file.

        Args:
            calib_offset: The unloaded equilibrium angle in radians.
        """
        with open(self.calib_offset_path, "w") as f:
            json.dump({"calib_offset": calib_offset}, f, indent=2)
        LOGGER.info(f"calib_offset saved to {self.calib_offset_path}")

    def load_calib_offset(self) -> float:
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
        calib_offset = data["calib_offset"]
        LOGGER.info(f"Loaded calib_offset: {calib_offset:.4f} from {self.calib_offset_path}")
        return calib_offset
    

if __name__ == "__main__":
    pass
