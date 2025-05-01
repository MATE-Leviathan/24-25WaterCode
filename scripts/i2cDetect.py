# Tries to find all i2c devices on all i2c buses

import os
import busio
import board

# Known device addresses
KNOWN_DEVICES = {
    0x28: 'BNO055 (IMU)',  # BNO055 default I2C address
    0x29: 'BNO055 (IMU)',  # BNO055 alternate address
    0x40: 'PCA9685 (PWM Breakout Board)',  # PCA9685 default I2C address
    0x76: 'Bar02 (Depth)',
    0x48: "ADS1115 (AD Converter for pH)", # ADS1115 default
    0x49: "ADS1115 (AD Converter for pH)", # ADS1115 if address pin is pulled to VCC
}

def get_i2c_bus_numbers():
    """Return a sorted list of I2C bus numbers like [0, 1, 2, ...]"""
    return sorted(
        int(dev[-1]) for dev in os.listdir('/dev') if dev.startswith('i2c-') and dev[-1].isdigit()
    )

def scan_bus(busnum):
    """Try to scan one I2C bus and identify known devices"""
    print(f"\nScanning I2C bus {busnum}...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA, busnum=busnum)
        while not i2c.try_lock():
            pass
        devices = i2c.scan()
        i2c.unlock()

        if devices:
            print(f"  Found devices at: {[hex(addr) for addr in devices]}")
            for addr in devices:
                if addr in KNOWN_DEVICES:
                    print(f"    Found {KNOWN_DEVICES[addr]} device at address {hex(addr)}")
                else:
                    print(f"    Unknown device at address {hex(addr)}")
        else:
            print("  No devices found.")

    except Exception as e:
        print(f"  Error scanning bus {busnum}: {e}")

def scan_all_buses():
    busnums = get_i2c_bus_numbers()
    if not busnums:
        print("No I2C buses found.")
        return
    for busnum in busnums:
        scan_bus(busnum)

if __name__ == "__main__":
    scan_all_buses()
