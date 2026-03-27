""""
exampleMAIN.py

An example of the MAIN.py script that may be called and created for projects using the VSO/VSPA.
Note: You must have opensourceleg library and opensourceleg[vso] package installed. 

As of now there are several dependencies that are missing in this. 

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
from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.base import SensorBase
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.sensors.encoderCounter import LS7366R
from opensourceleg.sensors.imu import LordMicrostrainIMU
from opensourceleg.sensors.hall import DRV5056 
from opensourceleg.sensors.adc import ADS114S0x
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop
from opensourceleg.utilities import Profiler
import VSOInitialization 
import sliderPosition
import csv

## Configurables 
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
trialNumber = '1'
subjIDCode = 'trial'
FREQUENCY = 6000 # in Hz
OFFLINE = False 

# set up logging configurables  
logName = subjIDCode + '_' + timestamp + '_' + trialNumber
loggingPath = '/home/jacbrady/opensourceleg/Data/Trial'


def controller_main():
    
    # set up logging using the Logger 
    datalog = Logger(log_path= loggingPath,file_name=logName)
    
    # define the "VSO" robot 
    vso = VSO[MaxonActuator, SensorBase](
        tag ="variableStiffessOrthosis",
        actuators={"ankle": MaxonActuator(offline = OFFLINE, tag = "ankle_actuator", motor_constants = None)},
        sensors={
                "ADC": ADS114S0x(offline = OFFLINE, tag = "adc"),
                "motorEncoder": LS7366R(offline = OFFLINE, tag = "encoder_counter_motor"),
                "ankleEncoder": AS5048B(offline = OFFLINE, tag="joint_encoder_ankle", bus='/dev/i2c-2', A1_adr_pin=False,
                                          A2_adr_pin=True, zero_position=0, enable_diagnostics=False),
                "hallEffect_1" : DRV5056(offline = OFFLINE, tag="hall_effect_1", sensor_num="A1", t_a = 23, supply_voltage =5),
                "hallEffect_2" : DRV5056(offline = OFFLINE, tag="hall_effect_2", sensor_num="A1", t_a = 23, supply_voltage =5),
            },
        )
    profiler = Profiler("ouput_run")

    start = time.monotonic()
    def elapsed_time():
        return time.monotonic() - start
    
    # track specific information using track function in datalog 
    datalog.track_function(elapsed_time, name="time")
    datalog.track_function(lambda: np.rad2deg(vso.sensors["ankleEncoder"].position), name="ankleEncoderPos")
    datalog.track_function(vso.actuators["ankle"].motor_encoder_position_perc, name="motorEncoderPosPerc")
    datalog.track_function(vso.sensors["hallEffect_1"].voltage, name="hallEffect_1_voltage") # mV
    datalog.track_function(vso.sensors["hallEffect_2"].voltage, name="hallEffect_2_voltage") # mV

    with vso, datalog:

        init = VSOInitialization(vso=vso, homing_pwm=0.45, sample_rate= 0.05,position_threshold = 20)
        init.run(run_calibration=False)  # if not disassembled !

        input('\nPress any key to begin walking:') 
        vso.update() # call an update of the robot
        loop = SoftRealtimeLoop(dt = 1/FREQUENCY) # soft real time loop set up! 
        
        for t in loop:
            profiler.tic() # start the profiler timing 
            
            vso.update()
            vso.sensors["ankleEncoder"].position - self.load_calib_offset()
            datalog.update() # update values into the datalog  
            datalog.flush_buffer() # can sometimes speed up the loop, this flushes the buffered log data to the CSV file.
            
            profiler.toc() # end the profiler timing 



if __name__ == "__main__":
    controller_main()



    
    
    
