
line = "z010.55021.00x"
line = "z010.00x"
if line[0] != "z" or line[-1] != "x":
    exit()

offset = 1
while line[offset] != "x":
    motor_idx_str = line[offset:offset+2]
    
    motor_idx = int(motor_idx_str)
    speed_str = line[offset+2:offset+6]
    speed = float(speed_str)
    offset += 6
    print(f"Got motor idx {motor_idx_str} at {speed_str}")
