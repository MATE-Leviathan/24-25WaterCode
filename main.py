import asyncio
import sys

from direction_motor import DirectionMotor
from picozero import Servo, pico_led

MOTOR_LOOKUP = {}
SERVO_PINS = [26, 7, 16, 3, 15, 11]
EXTRA_SERVO_PINS = [1, 20]

ACTUATOR_IDX = 99
ACTUATOR_PWM_PIN = 28
ACTUATOR_DIR_PIN = 21
ACTUATOR_PWM_FREQ = 20000

MOTOR_LOOKUP[ACTUATOR_IDX] = DirectionMotor(
    ACTUATOR_PWM_PIN,
    ACTUATOR_DIR_PIN,
    ACTUATOR_PWM_FREQ,
)

for i in SERVO_PINS:
    MOTOR_LOOKUP[i] = Servo(i, min_pulse_width=0.0011, max_pulse_width=0.0019)
    # MOTOR_LOOKUP[i] = Servo(i)

for i in EXTRA_SERVO_PINS:
    MOTOR_LOOKUP[i] = Servo(i, min_pulse_width=0.0007, max_pulse_width=0.0022)


async def read_stdin():
    reader = asyncio.StreamReader(sys.stdin)
    LED_ON = False
    motor_idx = 0
    speed_str = ""
    speed = 0.5
    offset = 1
    while True:
        try:
            line = await reader.readline()
            if not line:  # EOF
                break

            # 4 characters max for the value
            # spaces just for clarity, send without
            # format z 01 0.55 02 1.00x
            # Remove trailing newline
            line = line.decode().rstrip()
            if len(line) < 2 or line[0] != "z" or line[-1] != "x":
                continue

            offset = 1
            while line[offset] != "x":
                motor_idx_str = line[offset : offset + 2]

                motor_idx = int(motor_idx_str)
                # motor_idx = 0
                speed_str = line[offset + 2 : offset + 6]
                speed = float(speed_str)
                # speed = 0.5
                offset += 6
                # print(f"Got motor idx {motor_idx_str} at {speed_str}")
                if motor_idx in MOTOR_LOOKUP:
                    MOTOR_LOOKUP[motor_idx].value = speed
                # pico_led.blink(on_time=0.2, off_time=0.2)
                if LED_ON:
                    pico_led.off()
                    LED_ON = False
                else:
                    pico_led.on()
                    LED_ON = True

            # Process the line here
            # print(f"Received: {line}")

        except Exception as e:
            # print(f"Error reading stdin: {e}")
            pico_led.on()


async def main():
    await read_stdin()


# Run the event loop
try:
    asyncio.run(main())
except KeyboardInterrupt:
    MOTOR_LOOKUP[ACTUATOR_IDX].off()
    # print("\nProgram terminated by user")
