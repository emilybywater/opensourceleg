import json
import time
from pathlib import Path
from typing import Optional
from opensourceleg.logging import LOGGER

DEFAULT_CALIBRATION_PATH = Path("vso_calibration.json")


class VSOCalibration:
    """
    Stroke calibration for the VSO lead screw assembly.

    Drives the spring-support from the soft stop (0%) to the hard stop (100%)
    and back to measure full travel, compute scale_perc, and save to file.

    Should be re-run any time the lead screw assembly is disassembled or reassembled.

    Args:
        actuator: The motor actuator instance to drive during calibration.
        calibration_path: Path to save/load the calibration JSON file.
            Defaults to vso_calibration.json in the working directory.
    """

    def __init__(
        self,
        actuator,
        encoder,
        calibration_path: Path = DEFAULT_CALIBRATION_PATH,
    ) -> None:
        self.actuator = actuator
        self.encoder = encoder
        self.calibration_path = calibration_path
        self.scale_perc: Optional[float] = None

    def run(
        self,
        homing_pwm: int = 0.45,
        sample_rate: float = 0.05,
        position_threshold: int = 200,
    ) -> float:
        """
        Run the full stroke calibration sequence: 0% → 100% → 0%.

        Moves to the 0 position, clears the encoder, drives to the 100 position
        to measure full travel, computes scale_perc, then returns to 0.
        Saves the result to the calibration file.

        Args:
            homing_pwm: PWM value used during calibration moves.
            sample_rate: Time in seconds between position samples.
            position_threshold: Max position error (±) to consider motor stopped.

        Returns:
            scale_perc: Encoder counts per 1% of lead screw travel.
        """
        LOGGER.info("Starting VSO stroke calibration: moving to 0%.")
        
        self.actuator.home(homing_pwm=homing_pwm, 
                           sample_rate=sample_rate, 
                           position_threshold=position_threshold,
                           home_zero=True)
        self.encoder.clearCounter()
        LOGGER.info("Zero position reached. Encoder zeroed. Moving to 100%.")

        time.sleep(1)

        self.actuator.home(homing_pwm=homing_pwm,
                           sample_rate=sample_rate,
                           position_threshold=position_threshold,
                           home_zero=False)
        encoder_counts = self.encoder.readCounter()
        self.scale_perc = encoder_counts / 100
        LOGGER.info(
            f"Hard stop reached. Encoder counts: {encoder_counts}. "
            f"scale_perc: {self.scale_perc:.2f}"
        )

        time.sleep(1)

        LOGGER.info("Returning to 0%.")
        self.actuator.home(homing_pwm=homing_pwm,
                           sample_rate=sample_rate,
                           position_threshold=position_threshold,
                           home_zero=True)
        self.encoder.clearCounter()
        LOGGER.info("Calibration complete. Returned to 0%.")

        self._save()
        return self.scale_perc

    def load(self) -> float:
        """
        Load scale_perc from the calibration file.

        Returns:
            scale_perc: Encoder counts per 1% of lead screw travel.

        Raises:
            FileNotFoundError: If no calibration file exists at the specified path.
        """
        if not self.calibration_path.exists():
            raise FileNotFoundError(
                f"No calibration file found at {self.calibration_path}. "
                "Run VSOCalibration.run() first."
            )
        with open(self.calibration_path, "r") as f:
            data = json.load(f)
        self.scale_perc = data["scale_perc"]
        LOGGER.info(f"Loaded scale_perc: {self.scale_perc:.2f} from {self.calibration_path}")
        return self.scale_perc


    def _save(self) -> None:
        """Save scale_perc to the calibration JSON file."""
        with open(self.calibration_path, "w") as f:
            json.dump({"scale_perc": self.scale_perc}, f, indent=2)
        LOGGER.info(f"Calibration saved to {self.calibration_path}")