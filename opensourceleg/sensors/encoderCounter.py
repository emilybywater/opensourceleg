""" 
Original author, Federico Bolanos. 
Updated by Cameron Cobb for Python 3 (March 17th, 2019).
Updated by David Lam for opensourceleg (March, 2026).
Updated by Emily Bywater, also for opensourceleg (March, 2026)

Usage: import LS7366R then create an object by calling enc = LS7366R(CSX, CLK, BTMD)
CSX is either CE0 or CE1, CLK is the speed, BTMD is the bytemode 1-4 the resolution of your counter.
example: lever.Encoder(0, 1000000, 4)
These are the default values.
"""

from time import sleep
from typing import ClassVar, Final

import spidev

from opensourceleg.sensors.base import (
    EncoderCounterBase,
)

from opensourceleg.logging import LOGGER

class LS7366R(EncoderCounterBase):
    # -------------------------------------------
    # Constants

    #   Commands
    CLEAR_COUNTER = 0x20
    CLEAR_STATUS = 0x30
    READ_COUNTER = 0x60
    READ_STATUS = 0x70
    WRITE_MODE0 = 0x88
    WRITE_MODE1 = 0x90

    #   Modes

    # May need to be change "QUADRATURE_COUNT_MODE" line depending on the quadrature count mode... look at datasheet.
    # These values are in HEX (base 16) whereas the data sheet displays them in binary.
    # Datasheet can be found here: https://www.lsicsi.com/pdfs/Data_Sheets/LS7366R.pdf

    # 0x00: non-quadrature count mode. (A = clock, B = direction).
    # 0x01: x1 quadrature count mode (one count per quadrature cycle).
    # 0x02: x2 quadrature count mode (two counts per quadrature cycle).
    # 0x03: x4 quadrature count mode (four counts per quadrature cycle).

    QUADRATURE_COUNT_MODE = 0x03 # originally was 0x00

    class CounterConfig:
        FOURBYTE_COUNTER: Final = 0x00
        THREEBYTE_COUNTER: Final = 0x01
        TWOBYTE_COUNTER: Final = 0x02
        ONEBYTE_COUNTER: Final = 0x03

        MODES: ClassVar[list[int]] = [ONEBYTE_COUNTER, TWOBYTE_COUNTER, THREEBYTE_COUNTER, FOURBYTE_COUNTER]

    # ----------------------------------------------
    # Constructor

    def __init__(
        self,
        CSX: int = 0,
        CLK: int = 1000000,
        BTMD: int = 4,
        max_val: int = 4294967295, # for four byte mode, only correct for four byte mode
        spi_bus: int= 0,
        offline: bool = False,
        tag: str = "encoder_counter",
    ) -> None:

        super().__init__(tag=tag, offline=offline)

        self.counterSize = BTMD  # Sets the byte mode that will be used
        self.max_val = max_val  # Maximum value for the counter, used for signed count conversion

        self.spi = spidev.SpiDev()  # Initialize object
        self.spi.open(spi_bus, CSX)  # Which CS line will be used
        self.spi.max_speed_hz = CLK  # Speed of clk (modifies speed transaction)

        # Init the Encoder
        LOGGER.info(f"Clearing Encoder CS{CSX!s}'s Count...\t")
        self.clearCounter()
        LOGGER.info(f"Clearing Encoder CS{CSX!s}'s Status..\t")
        self.clearStatus()

        self.spi.xfer2([self.WRITE_MODE0, self.QUADRATURE_COUNT_MODE])

        sleep(0.1)  # Rest

        self.spi.xfer2([self.WRITE_MODE1, self.CounterConfig.MODES[self.counterSize - 1]])

    def close(self):
        LOGGER.info("Closing Encoder...")
        self.clearCounter()
        self.clearStatus()
        self.spi.close()
        self.spi = None

    def clearCounter(self):
        self.spi.xfer2([self.CLEAR_COUNTER])

        return "[DONE]"

    def clearStatus(self):
        self.spi.xfer2([self.CLEAR_STATUS])

        return "[DONE]"

    def readCounter(self):
        readTransaction = [self.READ_COUNTER]

        # Replaces the entire 2-line loop
        readTransaction.extend([0] * self.counterSize)

        data = self.spi.xfer2(readTransaction)

        EncoderCount = 0
        for i in range(self.counterSize):
            EncoderCount = (EncoderCount << 8) + data[i + 1]

        if data[1] != 255:
            self.EncoderCount = EncoderCount
        else:
            self.EncoderCount = EncoderCount - (self.max_val + 1)
        
        return self.EncoderCount

    def readStatus(self):
        data = self.spi.xfer2([self.READ_STATUS, 0xFF])

        return data[1]

    def start(self) -> None:
        """Not yet supported by this library."""
        pass

    def stop(self) -> None:
        """Not yet supported by this library."""
        self.close()
        LOGGER.info('Motor encoder stopped successfully.')

    def update(self) -> None:
        """Not yet supported by this library."""
        raise NotImplementedError("Update not implemented.")

    @property
    def count(self) -> None:
        """
        Encoder position in counts.

        Returns:
            float: Counts reading from the sensor.
        """
        return self.readCounter()

    @property
    def data(self) -> None:
        """Not yet supported by this library."""
        raise NotImplementedError("Data not implemented.")

    @property
    def is_streaming(self) -> None:
        """Not yet supported by this library."""
        raise NotImplementedError("Is streaming not implemented.")
