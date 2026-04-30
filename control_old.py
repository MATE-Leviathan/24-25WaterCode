import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

time.sleep(3)
# 0 -> fr-1
# 1 -> ml-5
#,2 -> bl-4
# 3 -> br-3
# 4 -> nothing
# 5 -> nothing
# 6 -> fl-6
# 7 -> mr-2
# accounted for 1, 2, 3,
# MAKE VERY SURE TO HAVE PADDING, OTHERWISE COOKED
all_pins = ["26", "07", "16", "03", "15", "11"]
PIN_NUM = "01"
cmd = ""
for pin in all_pins:
    cmd += f"z{pin}00.50x\n"
    print(cmd)

ser.write(cmd.encode())
time.sleep(1.5)

cmd = ""
for pin in all_pins:
    cmd += f"z{pin}00.75x\n"
    #cmd += f"z{pin}00.50x\n"

ser.write(cmd.encode())
print(cmd.encode())
time.sleep(0.2)

cmd = ""
for pin in all_pins:
    cmd += f"z{pin}00.50x\n"
    print(cmd)

ser.write(cmd.encode())

