import signal
import sys
import time
from opensourceleg.robots.vso import VSO
from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.sensors.base import SensorBase
from opensourceleg.sensors.encoderCounter import LS7366R
from opensourceleg.logging import LOGGER
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop
from sliderPosition import sliderPosition
from VSOInitialization import VSOInitialization

# Desired slider position (percent)
desiredPos = 40
FREQUENCY = 200
OFFLINE = False

# Global actuator reference for cleanup
GLOBAL_ACTUATOR = None

# Signal handler to stop motors cleanly
def cleanup(sig, frame):
    global GLOBAL_ACTUATOR
    print("\n[INFO] Ctrl+C detected, stopping actuator and exiting...")
    if GLOBAL_ACTUATOR:
        GLOBAL_ACTUATOR.stop()
    sys.exit(0)

# Attach Ctrl+C handler
signal.signal(signal.SIGINT, cleanup)

def movingSlider():
    global GLOBAL_ACTUATOR

    # define the "VSO" robot 
    vso = VSO[MaxonActuator, SensorBase](
        tag ="variableStiffessOrthosis",
        actuators={"ankle": MaxonActuator(offline = OFFLINE, tag = "ankle_actuator", motor_constants = None, frequency=10000)},
        sensors={
                "motor_encoder": LS7366R(offline = OFFLINE, tag = "encoder_counter_motor"),
                # "ankle_encoder": AS5048B(offline = OFFLINE, tag="joint_encoder_ankle", bus='/dev/i2c-3', A1_adr_pin=False,
                #                           A2_adr_pin=True, zero_position=0, enable_diagnostics=False),
            },
        )
    
    GLOBAL_ACTUATOR = vso.actuators["ankle"]  # For cleanup

    vso.actuators["ankle"].set_motor_encoder(vso.sensors["motor_encoder"])
    vso.actuators["ankle"].position_control_config() # set the scale percentage for the position control (stiffness)

    LOGGER.info("Finished setting up VSO...")

    start = time.monotonic()
    def elapsed_time():
        return time.monotonic() - start

    LOGGER.info("Started clock...")

       
    with vso: 
        vso.actuators["ankle"].set_control_mode(CONTROL_MODES.POSITION)
        vso.actuators["ankle"].position_control_init()
        init = VSOInitialization(vso=vso, side=1, homing_pwm = 0.2)
        init.run(run_calibration=False)
        time.sleep(1)
        vso.update()
        loop = SoftRealtimeLoop(dt = 1/FREQUENCY) # soft real time loop set up! 
        slider_done = False

        for t in loop:
            vso.update()
            # vso.sensors["ankle_encoder"].position - init.load_calib_offset()
            if not slider_done:
                try:
                    slider_done = sliderPosition.slider_position(
                        MaxonActuator=vso.actuators["ankle"],
                        desired_position_perc=desiredPos
                    )
                except Exception as e:
                    LOGGER.error(f"Error in sliderPosition: {e}")
                    vso.actuators["ankle"].stop()
                    break

            if slider_done:
                # Optional: keep loop running for monitoring or break
                break  # exit loop once slider has reached position
            
        vso.actuators["ankle"].stop()
        LOGGER.info("Exiting control loop safely.")

if __name__ == "__main__":
    try:
        movingSlider()
    finally:
        if GLOBAL_ACTUATOR:
            GLOBAL_ACTUATOR.stop()    
