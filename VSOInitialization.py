import json
import time
from pathlib import Path
from typing import Optional
import numpy as np

from opensourceleg.logging import LOGGER

from opensourceleg.robots.vso import VSO
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
        position_threshold: int = 200,
    ) -> None:

        self.vso = vso
        self.calibration_path = calibration_path
        self.calib_offset_path = calib_offset_path
        self.homing_pwm = homing_pwm
        self.sample_rate = sample_rate
        self.position_threshold = position_threshold

        LOGGER.info("VSO Initialization instance created.")

    def run(self, run_calibration: bool = False) -> None:
        """
        Run the full VSO initialization sequence.

        Args:
            run_calibration: If True, runs stroke calibration to compute scale_perc before homing. 
            Use after disassembly or first-time setup. 
            If False, assumes calibration file exists.
        """

        # actuator = next(iter(self.vso.actuators.values())) # just sees which actuators are connected
        # encoder_counter = self.vso.sensors["motor_encoder"]
        # calibration = VSOCalibration(
        #     vso=self.vso,
        #     actuator=actuator,
        #     encoder = encoder_counter,
        #     calibration_path=self.calibration_path,
        # )

        # # Step 1: Optional stroke calibration
        # if run_calibration:
        #     LOGGER.info("Running stroke calibration.")
        #     scale_perc = calibration.run(
        #         homing_pwm=self.homing_pwm,
        #         sample_rate=self.sample_rate,
        #         position_threshold=self.position_threshold,
        #     )
        #     LOGGER.info(f"Stroke calibration complete. scale_perc={scale_perc:.2f}")
        # else:
        #     LOGGER.info("Skipping stroke calibration. Assuming calibration file exists.")

        # Step 2: Encoder homing — zero at soft stop%
        LOGGER.info("Homing to soft stop and zeroing encoder.")
        self.vso.home(
            homing_pwm=self.homing_pwm,
            sample_rate=self.sample_rate,
            position_threshold=self.position_threshold,
            home_zero=True
        )
        
        # encoder_counter.clearCounter()
        # time.sleep(0.5)  # Ensure encoder clear is seen before moving
        # LOGGER.info(f"Encoder zeroed.")

        # LOGGER.info("Moving spring-support to stiffest position (100%).")
        # scale_perc = calibration.load()

        # actuator.position_control_init()
        # actuator.position_control_config(scale_perc=scale_perc)

        # sliderPosition.slider_position(position_percent=99.5)

        # # Step 3: Ankle encoder offset calibration at 100% stiffness
        # LOGGER.info("Capturing unloaded equilibrium angle. Waiting for ankle encoder warmup.")
        # time.sleep(2)  # Warmup period for ankle encoder to stabilize
        # ankle_sensor.update()
        # calib_offset = self.side*np.rad2deg(ankle_sensor.position)
        # self._save_calib_offset(calib_offset)
        # LOGGER.info(f"Ankle encoder offset calibrated. calib_offset={calib_offset:.4f} rad")


        LOGGER.info("VSO initialization complete.")

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
