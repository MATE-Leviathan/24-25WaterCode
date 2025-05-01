import board 
import busio
import time
import adafruit_pca9685
from adafruit_motor import servo

i2c_bus = busio.I2C(board.SCL, board.SDA) 
pca = adafruit_pca9685.PCA9685(i2c_bus, address = 0x40)
pca.frequency = 450
pca.channels[0].duty_cycles = 0xffff

userAngle = int(input('Angle: '))
minpulse = 1340
maxpulse = 1870
mangle = 90

fr1 = servo.Servo(pca.channels[0], min_pulse = minpulse, max_pulse = maxpulse)
mr2 = servo.Servo(pca.channels[5], min_pulse = minpulse, max_pulse = maxpulse)
br3 = servo.Servo(pca.channels[4], min_pulse = minpulse, max_pulse = maxpulse)
bl4 = servo.Servo(pca.channels[1], min_pulse = minpulse, max_pulse = maxpulse)
ml5 = servo.Servo(pca.channels[3], min_pulse = minpulse, max_pulse = maxpulse)
fl6 = servo.Servo(pca.channels[2], min_pulse = minpulse, max_pulse = maxpulse)
oscilloscope = servo.Servo(pca.channels[12], min_pulse = minpulse, max_pulse = maxpulse)


fr1.angle = mangle
mr2.angle = mangle
br3.angle = mangle
bl4.angle = mangle
ml5.angle = mangle
fl6.angle = mangle
oscilloscope.angle = mangle

print('initializing')
time.sleep(5)
print('running')

#fr1.angle = userAngle
mr2.angle = userAngle
#br3.angle = userAngle
#bl4.angle = userAngle
ml5.angle = userAngle
#fl6.angle = userAngle
oscilloscope.angle = userAngle
