""""
adcTestScript.py

Note: You must have opensourceleg library and opensourceleg[vso] package installed. 

Emily Bywater
04/06/2025
"""

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

side = 1

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
        actuators={},
        sensors={
                "adc": ADS114S0x(offline = OFFLINE, tag = "adc", spi_bus=1, data_rate=2000, drdy=16, voltage_reference=1.65),
                "hallEffect_1" : DRV5056(offline = OFFLINE, tag="hall_effect_1", sensor_num="A1", t_a = 23, supply_voltage =3.3),
                "hallEffect_2" : DRV5056(offline = OFFLINE, tag="hall_effect_2", sensor_num="A1", t_a = 23, supply_voltage =3.3),
                "ankle_encoder": AS5048B(offline = OFFLINE, tag="joint_encoder_ankle", bus='/dev/i2c-3', A1_adr_pin=False,
                                       A2_adr_pin=True, zero_position=0, enable_diagnostics=False),
            },
        )
    
    LOGGER.info("Finished setting up VSO...")

    profiler = Profiler("ouput_run")

    LOGGER.info("Finished setting up profiler...")

    start = time.monotonic()
    def elapsed_time():
        return time.monotonic() - start

    LOGGER.info("Started clock...")

    init = VSOInitialization(vso=vso, side=side, homing_pwm = 0.45)

    # track specific information using track function in datalog 
    datalog.track_function(elapsed_time, name="time")
    datalog.track_function(lambda: side * np.rad2deg(vso.sensors["ankle_encoder"].position) - init.calib_offset, name="ankleEncoderPos")
    datalog.track_function(
        lambda: (getattr(vso.sensors.get("adc", []), "_data", [0, 0])[0] / 1000),
        name="hallEffect_1"
    )
    datalog.track_function(
        lambda: (getattr(vso.sensors.get("adc", []), "_data", [0, 0])[1] / 1000),
        name="hallEffect_2"
    )

    LOGGER.info("Finished setting up datalogger...")
    
    with vso, datalog:
        
        LOGGER.info("Starting VSO initialization sequence...")
        init.run(run_calibration=False)  # if not disassembled !

        input('\nPress any key to begin walking:') 
        vso.update() # call an update of the robot
        loop = SoftRealtimeLoop(dt = 1/FREQUENCY) # soft real time loop set up! 
        
        for t in loop:
            # profiler.tic() # start the profiler timing 
            
            vso.update()
            datalog.update() # update values into the datalog  
            datalog.flush_buffer() # can sometimes speed up the loop, this flushes the buffered log data to the CSV file.
            
            profiler.toc() # end the profiler timing 



if __name__ == "__main__":
    controller_main()



    
    
    
