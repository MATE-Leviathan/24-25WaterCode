"""
Date Created: 02/19/2024
Author(s): Everett Tucker
Description: Testing the calibration for the pwm signals
"""

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

pca.channels[0].duty_cycle = 0xFFFF

# This is the most important part of this operation
# We cannot write a value outside of [0, 180], so we need another way to unlock the edges of the pwm range
# We make the changes in the motor initialization, so this is just a standard linear map
def map_value_to_angle(value):
    # return 93.909 * value + 103.3
    return value * 90 + 90

"""
# I might mess around with the max and min pulse to get it to output a perfect [1100, 1900]
# This is the data for the PCA Board with the black electical tape on the bottom
# Optimal max pulse is about 1971 pwm
# Optimal min pulse is about 1141 pwm
    
for pulse in range(1100, 1200):
    print(f'Testing on Pulse: {pulse}')
    thruster = servo.Servo(pca.channels[0], min_pulse=pulse, max_pulse=1971)
    thruster.angle = 0
    time.sleep(1)
"""

thruster = servo.Servo(pca.channels[8], min_pulse=1141, max_pulse=1971)

# This works as it should now, plus these values are incredibly easy to calibrate at competition
for i in range(3):
    print(f'Writing: {i * 90}')
    thruster.angle = i * 90
    time.sleep(3)

