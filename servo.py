from picozero import Servo
import time

servo = Servo(0)

servo.min()
time.sleep(1)
servo.max()

servo.off()
