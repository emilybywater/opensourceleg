import numpy as np
from datetime import datetime
import time
from opensourceleg.robots.vso import VSO
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging import LOGGER
from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.base import SensorBase
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.sensors.encoderCounter import LS7366R
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop
import sliderPosition 
from VSOInitialization import VSOInitialization
import csv
import traceback


# add the position you want to change the slider to here (as a percentage of the slider)
desiredPos = 40

FREQUENCY = 6000 # in Hz
OFFLINE = False 

def movingSlider():
    # define the "VSO" robot 
    vso = VSO[MaxonActuator, SensorBase](
        tag ="variableStiffessOrthosis",
        actuators={"ankle": MaxonActuator(offline = OFFLINE, tag = "ankle_actuator", motor_constants = None, frequency=10000)},
        sensors={
                "motor_encoder": LS7366R(offline = OFFLINE, tag = "encoder_counter_motor"),
                "ankle_encoder": AS5048B(offline = OFFLINE, tag="joint_encoder_ankle", bus='/dev/i2c-3', A1_adr_pin=False,
                                          A2_adr_pin=True, zero_position=0, enable_diagnostics=False),
            },
        )
    vso.actuators["ankle"].set_motor_encoder(vso.sensors["motor_encoder"])
    vso.actuators["ankle"].position_control_config() # set the scale percentage for the position control (stiffness)

    LOGGER.info("Finished setting up VSO...")

    start = time.monotonic()
    def elapsed_time():
        return time.monotonic() - start

    LOGGER.info("Started clock...")

       
    with vso: 
        vso.actuators["ankle"].set_control_mode(CONTROL_MODES.POSITION)
        init = VSOInitialization(vso=vso, side=1, homing_pwm = 0.45)
        vso.update()
        loop = SoftRealtimeLoop(dt = 1/FREQUENCY) # soft real time loop set up! 
        
        for t in loop:
            vso.update()
            vso.sensors["ankle_encoder"].position - init.load_calib_offset()
            sliderPosition.slider_position(MaxonActuator= vso.actuators["ankle"], desired_position_perc = desiredPos)
            

if __name__ == "__main__":
    movingSlider()
    
