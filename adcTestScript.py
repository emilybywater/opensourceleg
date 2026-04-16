""""
adcTestScript.py

Note: You must have opensourceleg library and opensourceleg[vso] package installed. 

Emily Bywater
04/06/2025
"""

import json
import numpy as np
from datetime import datetime
from pathlib import Path
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
from VSOInitialization import VSOInitialization, DEFAULT_HALL_THRESHOLD_PATH
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
                "adc": ADS114S0x(offline = OFFLINE, tag = "adc", spi_bus=0, spi_cs=1,data_rate=2000, drdy=16, voltage_reference=1.65),
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

    init = VSOInitialization(vso=vso, side=side, homing_pwm=0.45)

    ENCODER_ALPHA = 0.15  # EMA smoothing factor (~5 Hz cutoff at 200 Hz)
    angle_filt = 0.0
    dorsi_switch = False
    plantar_switch = False

    # track specific information using track function in datalog
    datalog.track_function(elapsed_time, name="time")
    datalog.track_function(lambda: angle_filt, name="ankleEncoderPos")
    datalog.track_function(
        lambda: (getattr(vso.sensors.get("adc", []), "_data", [0, 0])[0] / 1000),
        name="hallEffect_1"
    )
    datalog.track_function(
        lambda: (getattr(vso.sensors.get("adc", []), "_data", [0, 0])[1] / 1000),
        name="hallEffect_2"
    )
    datalog.track_function(lambda: int(dorsi_switch), name="dorsiflexionSwitch")
    datalog.track_function(lambda: int(plantar_switch), name="plantarflexionSwitch")

    LOGGER.info("Finished setting up datalogger...")
    
    with vso, datalog:
        
        input('\nPress any key to begin initialization. Make sure you are on the blue cam!')
        LOGGER.info("Starting VSO initialization sequence...")
        init.run(run_calibration=False, run_hall_calibration=False)

        # Load hall switch thresholds from calibration file
        thresholds = init.load_hall_thresholds()
        d_thresh = thresholds["dorsiflexion"]
        p_thresh = thresholds["plantarflexion"]

        input('\nPress any key to begin walking:')
        vso.update()  # call an update of the robot
        loop = SoftRealtimeLoop(dt=1 / FREQUENCY)  # soft real time loop set up!

        angle_last = 0.0
        hall1_last = 0.0
        hall2_last = 0.0
        loop_count = 0
        t_start_walk = time.monotonic()
        for t in loop:
            # profiler.tic() # start the profiler timing
            vso.update()

            angle_raw = side * np.rad2deg(vso.sensors["ankle_encoder"].position) - init.calib_offset
            angle_filt = ENCODER_ALPHA * angle_raw + (1 - ENCODER_ALPHA) * angle_filt
            angle = angle_filt
            hall1 = getattr(vso.sensors.get("adc", []), "_data", [0, 0])[0] / 1000
            hall2 = getattr(vso.sensors.get("adc", []), "_data", [0, 0])[1] / 1000
            angle_dot = angle - angle_last
            hall1_dot = hall1 - hall1_last

            dorsi_angle_ok  = d_thresh["angle_dot_sign"] * angle_dot > 0
            plantar_angle_ok = p_thresh["angle_dot_sign"] * angle_dot > 0

            if (dorsi_angle_ok
                    and d_thresh["hall1_lower"] <= hall1 <= d_thresh["hall1_upper"]
                    and d_thresh["hall2_lower"] <= hall2 <= d_thresh["hall2_upper"]
                    and abs(hall1_dot) <= d_thresh["hall1_dot_upper"]
                    and d_thresh["angle_lower"] <= angle <= d_thresh["angle_upper"]
                    and not dorsi_switch):
                print('\n  Dorsiflexion switch detected!')
                dorsi_switch = True
                plantar_switch = False
            elif angle > d_thresh["angle_upper"] and not dorsi_switch:
                print('\n  Dorsiflexion switch forced (past range)!')
                dorsi_switch = True
                plantar_switch = False
            elif (plantar_angle_ok
                    and p_thresh["hall1_lower"] <= hall1 <= p_thresh["hall1_upper"]
                    and p_thresh["hall2_lower"] <= hall2 <= p_thresh["hall2_upper"]
                    and p_thresh["angle_lower"] <= angle <= p_thresh["angle_upper"]
                    and not plantar_switch):
                print('\n  Plantarflexion switch detected!')
                dorsi_switch = False
                plantar_switch = True
            elif angle < p_thresh["angle_lower"] and not plantar_switch:
                print('\n  Plantarflexion switch forced (past range)!')
                dorsi_switch = False
                plantar_switch = True

            # Live sensor readout at 10 Hz
            if loop_count % 20 == 0:
                elapsed = time.monotonic() - t_start_walk
                state = "DORSI " if dorsi_switch else "PLANTAR" if plantar_switch else "-------"
                print(
                    f"\r  t={elapsed:6.1f}s"
                    f"  angle={angle:+7.2f}°"
                    f"  h1={hall1:+.4f} V  h2={hall2:+.4f} V"
                    f"  d(h1)={hall1_dot:+.5f}  d(ang)={angle_dot:+.5f}"
                    f"  [{state}]   ",
                    end="",
                    flush=True,
                )
            loop_count += 1

            datalog.update() # update values into the datalog
            datalog.flush_buffer() # can sometimes speed up the loop, this flushes the buffered log data to the CSV file.

            profiler.toc() # end the profiler timing

            angle_last = angle
            hall1_last = hall1
            hall2_last = hall2


if __name__ == "__main__":
    controller_main()



    
    
    
