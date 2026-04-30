import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Initialize I2C bus
i2c = busio.I2C('GP13_I2C2_CLK', 'GP14_I2C2_DAT')

# Initialize ADS1115
ads = ADS.ADS1115(i2c)

# Use Single-ended Mode (AIN0)
channel = AnalogIn(ads, ADS.P3)

# Read and Print Voltage
while True:
    voltage = round(channel.voltage, 3)  # Get voltage reading
    print(voltage, 'V', (-5.98*voltage)+16.1, 'pH')
    time.sleep(1)
