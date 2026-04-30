# This simple test outputs a 50% duty cycle PWM single on the 0th channel. Connect an LED and
# resistor in series to the pin to visualize duty cycle changes and its impact on brightness.

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

thrusters = []
for pin in range(5):
    thrusters.append(servo.Servo(pca.channels[pin], min_pulse=1141, max_pulse=1971))
#thrusters.append(servo.Servo(pca.channels[16], min_pulse=1100, max_pulse=1900))

# Initializing Thrusters
print("Initalizing...")
for mcs in thrusters:
    mcs.angle = 90
time.sleep(7)

print("Half Forward")
for mcs in thrusters:
    mcs.angle = 135
time.sleep(1)

"""
# Scatter testing to find optimal initialization pwm
# It Turns out that it is about 97, at least today 2/16/2024
for mcs in thrusters:
    for pwm in range(80, 101):
        print(f'Initializing on {pwm}')
        mcs.angle = pwm
        time.sleep(7)

print(1/pca.channels[8].frequency)
print(pca.channels[8].duty_cycle)
time.sleep(4)

continuous_thrusters = []
continuous_thrusters.append(servo.ContinuousServo(pca.channels[7], min_pulse=1100, max_pulse=1900))

for thruster in continuous_thrusters:
    thruster.throttle = 0

print("Thrusters Initializing...")
time.sleep(7)

for thruster in continuous_thrusters:
    thruster.throttle = 0.5
time.sleep(1)
"""

