""""
exampleMAIN.py

An example of the MAIN.py script that may be called and created for projects using the VSO/VSPA.
Note: You must have opensourceleg library and opensourceleg[vso] package installed. 

Anushka Rathi
03/02/2025

Emily Bywater
03/25/2025
"""

import numpy as np
from datetime import datetime
import time
from opensourceleg.robots.vso import VSO
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.brushed import MaxonActuator ## add the additional things that may be needed here !! 
from opensourceleg.logging import LOGGER
from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.base import SensorBase
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.sensors.encoderCounter import LS7366R
from opensourceleg.sensors.imu import LordMicrostrainIMU
from opensourceleg.sensors.hall import DRV5056 
from opensourceleg.sensors.adc import ADS114S0x
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop
from opensourceleg.utilities import Profiler
from VSOInitialization import VSOInitialization
import sliderPosition
import csv
import traceback

## Configurables 
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
trialNumber = '1'
subjIDCode = 'trial'
FREQUENCY = 200 # in Hz
OFFLINE = False 

# set up logging configurables  
logName = subjIDCode + '_' + timestamp + '_' + trialNumber
loggingPath = '/home/ebywater/opensourceleg/Data/Trial'


def controller_main():
    
    # set up logging using the Logger 
    datalog = Logger(log_path= loggingPath,file_name=logName)
    
    LOGGER.info("Finished initializing data logger...")

    # define the "VSO" robot 
    vso = VSO[MaxonActuator, SensorBase](
        tag ="variableStiffessOrthosis",
        actuators={"ankle": MaxonActuator(offline = OFFLINE, tag = "ankle_actuator", motor_constants = None, frequency=10000)},
        sensors={
                "ankle_encoder": AS5048B(offline = OFFLINE, tag="joint_encoder_ankle", bus='/dev/i2c-3', A1_adr_pin=False,
                                       A2_adr_pin=True, zero_position=0, enable_diagnostics=False),
                "motor_encoder": LS7366R(offline = OFFLINE, tag = "encoder_counter_motor", spi_bus=0),
                # "adc": ADS114S0x(offline = OFFLINE, tag = "adc", spi_bus=1, data_rate=1000,drdy=16),
                
                #"hallEffect_1" : DRV5056(offline = OFFLINE, tag="hall_effect_1", sensor_num="A1", t_a = 23, supply_voltage =3.3),
                # "hallEffect_2" : DRV5056(offline = OFFLINE, tag="hall_effect_2", sensor_num="A1", t_a = 23, supply_voltage =3.3),
            },
        )
    vso.actuators["ankle"].set_motor_encoder(vso.sensors["motor_encoder"])
    vso.actuators["ankle"].position_control_config() # set the scale percentage for the position control (stiffness)

    LOGGER.info("Finished setting up VSO...")

    profiler = Profiler("ouput_run")

    LOGGER.info("Finished setting up profiler...")

    start = time.monotonic()
    def elapsed_time():
        return time.monotonic() - start

    LOGGER.info("Started clock...")

    position = 0 
    # track specific information using track function in datalog 
    datalog.track_function(elapsed_time, name="time")
    datalog.track_function(lambda: vso.actuators["ankle"].motor_encoder_position_perc, name="motorEncoderPosPerc")
   # datalog.track_function(lambda: np.rad2deg(position), name="ankleEncoderPos")
    # datalog.track_function(lambda: vso.sensors["hallEffect_1"].voltage, name="hallEffect_1_voltage") # mV
    # datalog.track_function(lambda: vso.sensors["hallEffect_2"].voltage, name="hallEffect_2_voltage") # mV
    
    LOGGER.info("Finished setting up datalogger...")
    
    with vso, datalog:
        vso.actuators["ankle"].set_control_mode(CONTROL_MODES.POSITION)

        LOGGER.info("Starting VSO initialization sequence...")
        init = VSOInitialization(vso=vso, side=1, homing_pwm = 0.45)
        init.run(run_calibration=True)  # if not disassembled !


        input('\nPress any key to begin walking:') 
        vso.update() # call an update of the robot
        loop = SoftRealtimeLoop(dt = 1/FREQUENCY) # soft real time loop set up! 
        
        for t in loop:
        #     profiler.tic() # start the profiler timing 
            
            vso.update()
            # position = vso.sensors["ankle_encoder"].position - init.load_calib_offset()
            # datalog.update() # update values into the datalog  
            # datalog.flush_buffer() # can sometimes speed up the loop, this flushes the buffered log data to the CSV file.
            
        #     profiler.toc() # end the profiler timing 



if __name__ == "__main__":
    controller_main()



    
    
    
