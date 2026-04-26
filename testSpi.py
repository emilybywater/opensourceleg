import spidev
import time
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 100000
spi.mode = 0
time.sleep(5)
# Write a known value to DTR then load it into CNTR
spi.xfer2([0x09, 0x00, 0x00, 0x01, 0x23])  # WR_DTR, write 0x00000123
spi.xfer2([0xE0])                            # LOAD_CNTR from DTR
result = spi.xfer2([0x60, 0x00, 0x00, 0x00, 0x00])  # RD_CNTR + 4 dummy bytes
print([hex(x) for x in result[1:]])          # Should print 0x00, 0x00, 0x01, 0x23
spi.close()