# Tries to find all i2c devices on all i2c buses

import os
from smbus2 import SMBus, i2c_msg

# Known device addresses
KNOWN_DEVICES = {
    0x28: 'BNO055 (IMU)',  # BNO055 default I2C address
    0x29: 'BNO055 (IMU)',  # BNO055 alternate address
    0x40: 'PCA9685 (PWM Breakout Board)',  # PCA9685 default I2C address
    0x76: 'Bar02 (Depth)',
    0x48: "ADS1115 (AD Converter for pH)", # ADS1115 default
    0x49: "ADS1115 (AD Converter for pH)", # ADS1115 if address pin is pulled to VCC
}

def get_i2c_buses():
    return sorted(
        int(dev.replace("i2c-", "")) 
        for dev in os.listdir('/dev') if dev.startswith('i2c-')
    )

def scan_bus(busnum):
    print(f"\nScanning I2C bus {busnum} (/dev/i2c-{busnum})...")
    try:
        with SMBus(busnum) as bus:
            found = False
            for address in range(0x03, 0x78):
                try:
                    bus.write_quick(address)
                    found = True
                    name = KNOWN_DEVICES.get(address, "Unknown device")
                    print(f"  Found {name} at address 0x{address:02X}")
                except OSError:
                    continue
            if not found:
                print("  No devices found.")
    except FileNotFoundError:
        print(f"  /dev/i2c-{busnum} not available.")

def main():
    buses = get_i2c_buses()
    if not buses:
        print("No I2C buses found.")
        return
    for busnum in buses:
        scan_bus(busnum)

if __name__ == "__main__":
    main()