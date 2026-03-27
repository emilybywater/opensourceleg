import time
import numpy as np
from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging import LOGGER

class sliderPosition:
    """
    Moving the slider on the VSO.

    Drives the spring-support from its current position to a new position,
    which can be either based on a number of mm or a % of the total stroke.
    """

    def slider_position(self, desired_position_perc) -> None:
        """
        PID control of the VSO spring support including pwm saturation, small position deadband,
        and killing the motor when the transmission is too loaded
        """
        desired_position_encoder = MaxonActuator.perc_to_cts(desired_position_perc)
        desired_position_mm = MaxonActuator.cts_to_mm(desired_position_encoder)

        start_time = time.time()
        last_time = start_time
        current_time = start_time

        while True:
            try:
                # Check if the ankle is flexed and temporarily abort moving slider until ankle
                # is within an acceptable range

                MaxonActuator.update()

                ankle_in_range = check_ankle_position()

                if not ankle_in_range:
                    MaxonActuator.stop()
                    return

                MaxonActuator.check_coupler_drift()

                if np.abs(desired_position_mm - MaxonActuator.motor_position_mm) > MaxonActuator.min_error:
                    if desired_position_mm > MaxonActuator.slider_max_mm:
                        desired_position_encoder = MaxonActuator.slider_max_counts
                    elif desired_position_mm < MaxonActuator.slider_min_mm:
                        desired_position_encoder = MaxonActuator.slider_min_counts
                else:  # slider position is already close enough to commanded position (conserves battery)
                    MaxonActuator.stop()
                    return

                current_time = time.time()
                dt = current_time - last_time
                t_elapsed = current_time - start_time

                if t_elapsed > MaxonActuator.time_limit:
                    LOGGER.warning("Slider may be jammed - please check prototype (pwm set to zero for safety)")
                    MaxonActuator.stop()
                    return

                error_encoder = int(desired_position_encoder - MaxonActuator.motor_position_cts)

                pwm = MaxonActuator.pid_ctrl_position(error_encoder, dt)

                if (
                    config.ankle_angle <= config.function_stiffness_to_loaded_angle_dorsiflexion(config.stiffness)
                ) and (config.ankle_angle >= config.function_stiffness_to_loaded_angle_planarflexion(config.stiffness)):
                    if pwm < 0.0:  # TODO: Check directionality is correct
                        MaxonActuator.set_motor_direction_backward()
                    else:
                        MaxonActuator.set_motor_direction_forward()

                    MaxonActuator.set_motor_pwm(np.abs(pwm))

                last_time = current_time

            except KeyboardInterrupt:
                MaxonActuator.stop()
                LOGGER.warning("KeyboardInterrupt")
                break
