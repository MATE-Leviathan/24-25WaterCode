
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


servo_pins = [6, 7]
servos = []

for pin in servo_pins:
    servos.append(servo.Servo(pca.channels[pin], actuation_range=300))

while True:
    command = input("Enter a servo command (open/close/angle): ")

    if command == "open":
        for servo in servos:
            servo.angle = 300
    elif command == "close":
        for servo in servos:
            servo.angle = 0
    else:
        try:
            for servo in servos:
                servo.angle = int(command)
        except:
            pass
        break
