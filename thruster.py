from picozero import Servo
import time

servo = Servo(1, min_pulse_width=0.0011, max_pulse_width=0.0019)

servo.mid()
time.sleep(3)

servo.value = 0.75
time.sleep(0.5)

servo.off()
