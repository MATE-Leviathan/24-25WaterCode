import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface
i2c = busio.I2C(board.SCL, board.SDA)

# Create a PCA9685 object
pca = PCA9685(i2c)
pca.frequency = 450  # Standard for servos, change if needed

# Set the servo channels to OFF
CLAW_SERVO_CHANNELS = [1, 2, 3, 4, 5, 6, 7]  # Modify if your servos are on different channels

for channel in CLAW_SERVO_CHANNELS:
    pca.channels[channel].duty_cycle = 0  # This disables PWM output

print("Servos have been turned off.")
