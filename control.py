import time
import serial

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)


def command(pin, value):
    if value < 0:
        value_str = f"{value:.1f}"
    else:
        value_str = f"{value:.2f}"

    # Format: zPPVALUEx\n
    # Example: z990.20x
    return f"z{int(pin):02d}{value_str}x\n"


def send_commands(commands, pad=False):
    """
    commands should be a list of (pin, value)
    """
    cmd = ""

    for pin, value in commands:
        cmd += command(pin, value)

    print("Sending:")
    print(cmd)

    data = cmd.encode()

    if pad:
        data = data.ljust(64, b" ")

    ser.reset_output_buffer()
    ser.write(data)
    ser.flush()

def set_thrusters(value):
    send_commands([(pin, value) for pin in THRUSTER_PINS], pad=True)

def stop_thrusters():
    send_commands([(pin, 0.00) for pin in THRUSTER_PINS], pad=True)


# -----------------------------
# CONFIG
# -----------------------------

# Thruster channel mapping:
# 0 -> fr-1
# 1 -> ml-5
# 2 -> bl-4
# 3 -> br-3
# 6 -> fl-6
# 7 -> mr-2
THRUSTER_PINS = [7, 16, 3, 15]

# CHANGE THESE to your actual two servo channels
SERVO_PINS = [1, 20]

ACTUATOR_PIN = 99

THRUSTER_TEST_SPEED = 0.75
THRUSTER_TEST_TIME = 0.40
THRUSTER_INIT_VALUE = 0.50
THRUSTER_INIT_TIME = 3.0


SERVO_HIGH = 0.70
SERVO_LOW = 0.30
SERVO_MOVE_TIME = 0.50

ACTUATOR_SPEED = 0.20
ACTUATOR_MOVE_TIME = 0.50


# -----------------------------
# SYSTEMS CHECK
# -----------------------------

time.sleep(1)

try:
    print("Starting systems check...")
    print(f"Setting all thrusters to {THRUSTER_INIT_VALUE} for {THRUSTER_INIT_TIME} seconds")
    set_thrusters(THRUSTER_INIT_VALUE)
    time.sleep(THRUSTER_INIT_TIME)
# 1. Thruster check
    print("\n=== Thruster check ===")
    send_commands([(pin, THRUSTER_TEST_SPEED) for pin in THRUSTER_PINS], pad=True)
    time.sleep(THRUSTER_TEST_TIME)

    print("Stopping thrusters")
    stop_thrusters()
    time.sleep(0.5)
    exit()
    # 2. Servo check
    print("\n=== Servo check ===")
    print("Moving servos to 0.70")
    send_commands([(pin, SERVO_HIGH) for pin in SERVO_PINS])
    time.sleep(SERVO_MOVE_TIME)

    print("Moving servos to 0.30")
    send_commands([(pin, SERVO_LOW) for pin in SERVO_PINS])
    time.sleep(SERVO_MOVE_TIME)

    print("Returning servos to 0.50")
    send_commands([(pin, 0.50) for pin in SERVO_PINS])
    time.sleep(0.5)

    # 3. Linear actuator check
    print("\n=== Linear actuator check ===")
    print("Actuator forward")
    send_commands([(ACTUATOR_PIN, ACTUATOR_SPEED)])
    time.sleep(ACTUATOR_MOVE_TIME)

    print("Actuator stop")
    send_commands([(ACTUATOR_PIN, 0.00)])
    time.sleep(0.3)

    print("Actuator backward")
    send_commands([(ACTUATOR_PIN, -ACTUATOR_SPEED)])
    time.sleep(ACTUATOR_MOVE_TIME)

    print("Actuator stop")
    send_commands([(ACTUATOR_PIN, 0.00)])
    time.sleep(0.3)

    print("\nSystems check complete.")

finally:
    print("Final safety stop")

    # Stop thrusters and actuator no matter what
    stop_thrusters()
    send_commands([(ACTUATOR_PIN, 0.00)])

    ser.close()
