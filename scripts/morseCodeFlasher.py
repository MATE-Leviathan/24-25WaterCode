# Code to use a bluerobotics light to flash morse code
# Made for the MATE ROV 2025 collab mission that got cancelled due to weather

import sys
import time
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


DOT_TIME = 0.5 # time in seconds for one dot
DASH_TIME = DOT_TIME * 3
TIME_BETWEEN_FLASHES = DOT_TIME # time in seconds between dots and dashes
TIME_BETWEEN_LETTERS = DOT_TIME * 3 # time in seconds between completed letters
FLASH_ON = 90 # angle to set flashlight to when on

# Initializing the PCA Board
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 450

flashlight = servo.Servo(pca.channels[7], min_pulse = 1340, max_pulse = 1870) # light is wired on pin 8
flashlight.angle = 0

# Dictionary representing the morse code chart
MORSE_CODE_DICT = { 'A':'.-', 'B':'-...',
                    'C':'-.-.', 'D':'-..', 'E':'.',
                    'F':'..-.', 'G':'--.', 'H':'....',
                    'I':'..', 'J':'.---', 'K':'-.-',
                    'L':'.-..', 'M':'--', 'N':'-.',
                    'O':'---', 'P':'.--.', 'Q':'--.-',
                    'R':'.-.', 'S':'...', 'T':'-',
                    'U':'..-', 'V':'...-', 'W':'.--',
                    'X':'-..-', 'Y':'-.--', 'Z':'--..',
                    '1':'.----', '2':'..---', '3':'...--',
                    '4':'....-', '5':'.....', '6':'-....',
                    '7':'--...', '8':'---..', '9':'----.',
                    '0':'-----', ', ':'--..--', '.':'.-.-.-',
                    '?':'..--..', '/':'-..-.', '-':'-....-',
                    '(':'-.--.', ')':'-.--.-'}

# Function to encrypt the string
# according to the morse code chart
def encrypt(message):
    cipher = ''
    for letter in message:
        if letter != ' ':
            # Looks up the dictionary and adds the
            # corresponding morse code
            # along with a space to separate
            # morse codes for different characters
            cipher += MORSE_CODE_DICT[letter] + ' '
        else:
            # 1 space indicates different characters
            # and 2 indicates different words
            cipher += ' '

    return cipher

# Flashes a dot/dash/space using flashlight
def flash(morse):
    for symbol in morse:
        if symbol == '.':
            #light on
            flashlight.angle(FLASH_ON)
            print("flashing dot")
            time.sleep(DOT_TIME)
            #light off
            flashlight.angle(0)
            print("off")
            time.sleep(TIME_BETWEEN_FLASHES)
        elif symbol == '-':
            #light on
            flashlight.angle(FLASH_ON)
            print("flashing dash")
            time.sleep(DASH_TIME)
            #light off
            flashlight.angle(0)
            print("off")
            time.sleep(TIME_BETWEEN_FLASHES)
        else: # If symbol is an empty space
            #light off
            flashlight.angle(0)
            print("space between letters")
            time.sleep(TIME_BETWEEN_LETTERS)

def main():
    message = sys.argv[1].upper()
    print("Message: ",message)
    morse = encrypt(message)
    print("Morse Code: ",morse)
    flash(morse)

if __name__ == '__main__':
    main()
