import ms5837
import time

for bus in range(3):  # Try i2c-0, i2c-1, i2c-2
    sensor = ms5837.MS5837(ms5837.MODEL_02BA, bus)
    if sensor.init():
        print(f"Sensor found on I2C bus {bus}")
        break

# We must initialize the sensor before reading it
if not sensor.init():
        print("Sensor could not be initialized on any bus")
        exit(1)

sensor.setFluidDensity(1000) #1000 kg/m^3   pool water might be slightly denser

# Print readings
while True:
    if sensor.read(ms5837.OSR_2048):
        print(("Depth: %.3f meters\t P: %0.1f mbar  %0.3f psi\tT: %0.2f C  %0.2f F") % (
        sensor.depth(), # Meters
        sensor.pressure(), # Default is mbar (no arguments)
        sensor.pressure(ms5837.UNITS_psi), # Request psi
        sensor.temperature(), # Default is degrees C (no arguments)
        sensor.temperature(ms5837.UNITS_Farenheit))) # Request Farenheit
    else:
        print("Sensor read failed!")
    #time.sleep(0.25)