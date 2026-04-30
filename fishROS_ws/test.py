from board import SCL, SDA
import busio
import time
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz.
pca.frequency = 450

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
pca.channels[0].duty_cycle = 0xFFFF

mcs = servo.Servo(pca.channels[8], min_pulse=1100, max_pulse=1900)
mcs.angle = 90
print(mcs._pwm_out.duty_cycle)
print(f"Expected duty cycle {(1500 * pca.frequency) / 1000000 * 0xFFFF}")
print(mcs.fraction)
time.sleep(3)
mcs.angle = 130

time.sleep(5)
pca.channels[0].duty_cycle = 0x0000
mcs.angle = 90
pca.deinit()