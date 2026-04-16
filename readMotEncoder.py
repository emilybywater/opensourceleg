""""
readMotEncoder.py

An example of the MAIN.py script that may be called and created for projects using the VSO/VSPA.
Note: You must have opensourceleg library and opensourceleg[vso] package installed.

Emily Bywater
04/16/2026
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
        sensors={"motor_encoder": LS7366R(offline = OFFLINE, tag = "encoder_counter_motor", spi_bus=0)},)

    vso.actuators["ankle"].set_motor_encoder(vso.sensors["motor_encoder"])

    LOGGER.info("Finished setting up VSO...")

    profiler = Profiler("ouput_run")

    LOGGER.info("Finished setting up profiler...")

    start = time.monotonic()
    def elapsed_time():
        return time.monotonic() - start

    LOGGER.info("Started clock...")

    encoder_counter = vso.sensors["motor_encoder"]

    # track specific information using track function in datalog
    datalog.track_function(elapsed_time, name="time")
    datalog.track_function(lambda: encoder_counter.readCounter(), name="motorEncoderCts")

    LOGGER.info("Finished setting up datalogger...")

    with vso, datalog:
        vso.actuators["ankle"].set_control_mode(CONTROL_MODES.POSITION)

        LOGGER.info("Starting manual motor calibration...")
        init = VSOInitialization(vso=vso, side=1)
        scale_perc = init.run_manual_motor_calibration()

        # After calibration the encoder is zeroed at 0% end stop.
        # scale_perc is encoder counts per 1% of full stroke.

        input('\nPress Enter to begin monitoring motor position: ')
        vso.update()
        loop = SoftRealtimeLoop(dt = 1/FREQUENCY)

        PRINT_EVERY = FREQUENCY // 10  # print at 10 Hz
        loop_count = 0

        for t in loop:
            vso.update()

            raw_counts = encoder_counter.readCounter()
            position_perc = raw_counts / scale_perc  # 0–100 %

            if loop_count % PRINT_EVERY == 0:
                print(
                    f"\r  Motor position: {position_perc:6.1f}%   "
                    f"(counts: {raw_counts:8d})   ",
                    end="",
                    flush=True,
                )
            loop_count += 1


if __name__ == "__main__":
    controller_main()
