import smbus2 as smbus
from device import Device 


# 0x40 from i2cdetect -y 1 (1 if Raspberry pi 2)
bus = smbus.SMBus(1)
dev = Device(bus, 0x40)

# set the duty cycle for LED05 to 50%
dev.set_pwm(0, 2047)

# set the pwm frequency (Hz)
dev.set_pwm_frequency(1000)