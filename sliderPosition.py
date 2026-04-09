import time
import numpy as np
from opensourceleg.logging import LOGGER

class sliderPosition:
    """
    Moving the slider on the VSO.

    Drives the spring-support from its current position to a new position,
    which can be either based on a number of mm or a % of the total stroke.
    """
        
    def slider_position(MaxonActuator, desired_position_perc) -> None:
        """
        PID control of the VSO spring support including pwm saturation, small position deadband,
        and killing the motor when the transmission is too loaded
        """
        desired_position_encoder = MaxonActuator.perc_to_cts(desired_position_perc)
        desired_position_mm = MaxonActuator.cts_to_mm(desired_position_encoder)

        # start_time = time.time()
        start_time = time.time()
        last_time = start_time

        while True:
            try:
                # Check if the ankle is flexed and temporarily abort moving slider until ankle
                # is within an acceptable range

                MaxonActuator.update()

                ankle_in_range = True # TODO: add a check_ankle_position() for when not testing on desktop

                if not ankle_in_range:
                    MaxonActuator.stop()
                    return True

                MaxonActuator.check_coupler_drift()

                if np.abs(desired_position_mm - MaxonActuator.motor_position_mm) > MaxonActuator.min_error:
                    if desired_position_mm > MaxonActuator.slider_max_mm:
                        desired_position_encoder = MaxonActuator.slider_max_counts
                    elif desired_position_mm < MaxonActuator.slider_min_mm:
                        desired_position_encoder = MaxonActuator.slider_min_counts
                else:  # slider position is already close enough to commanded position (conserves battery)
                    MaxonActuator.stop()
                    return True 

                current_time = time.time()
                dt = current_time - last_time
                t_elapsed = current_time - start_time

                if dt <= 0.0:
                    dt = 1e-6  # Prevent division by zero, though this should rarely happen

                if t_elapsed > MaxonActuator.time_limit:
                    LOGGER.warning("Slider may be jammed - please check prototype (pwm set to zero for safety)")
                    MaxonActuator.stop()
                    return True

                error_encoder = int(desired_position_encoder - MaxonActuator.motor_position_cts)
                MaxonActuator.position_control_init()
                pwm = MaxonActuator.pid_ctrl_position(error_encoder, dt)
                
                if pwm < 0.0:
                    MaxonActuator.set_motor_direction_backward()
                elif pwm > 0.0:
                    MaxonActuator.set_motor_direction_forward()
                else:
                    MaxonActuator.stop()
                    return True
            
                MaxonActuator.set_motor_pwm(np.abs(pwm))

                last_time = current_time
                time.sleep(0.01)
            except KeyboardInterrupt:
                MaxonActuator.stop()
                LOGGER.warning("KeyboardInterrupt during slider motion.")
                break
