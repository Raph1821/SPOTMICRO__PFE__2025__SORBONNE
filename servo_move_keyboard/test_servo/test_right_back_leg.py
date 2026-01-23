#!/usr/bin/env python3
from smbus2 import SMBus
import time

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

class PCA9685:
    def __init__(self):
        self.bus = SMBus(1)
        self.address = 0x40
        prescale = int(25000000.0 / 4096.0 / 50.0 - 1.0 + 0.5)
        oldmode = self.bus.read_byte_data(self.address, MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, MODE1, newmode)
        self.bus.write_byte_data(self.address, PRESCALE, prescale)
        self.bus.write_byte_data(self.address, MODE1, oldmode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1, oldmode | 0x80)
        
    def set_pwm(self, channel, value):
        self.bus.write_byte_data(self.address, LED0_ON_L + 4*channel, 0)
        self.bus.write_byte_data(self.address, LED0_ON_L + 4*channel + 1, 0)
        self.bus.write_byte_data(self.address, LED0_ON_L + 4*channel + 2, value & 0xFF)
        self.bus.write_byte_data(self.address, LED0_ON_L + 4*channel + 3, value >> 8)

pca = PCA9685()

# Right Back leg servos (channels 3, 4, 5)
RB_3 = 3   # Knee (genou)
RB_2 = 4  # Shoulder (épaule)
RB_1 = 5  # Hip (hanche)

print("Testing RIGHT BACK LEG...")
print("Setting all servos to CENTER (306)...")
pca.set_pwm(RB_1, 306)
pca.set_pwm(RB_2, 306)
pca.set_pwm(RB_3, 306)
time.sleep(2)

print("\nTesting RB_1 (Hip - channel 5)...")
pca.set_pwm(RB_1, 200)
time.sleep(1)
pca.set_pwm(RB_1, 400)
time.sleep(1)
pca.set_pwm(RB_1, 306)
time.sleep(1)

print("\nTesting RB_2 (Shoulder - channel 4)...")
pca.set_pwm(RB_2, 200)
time.sleep(1)
pca.set_pwm(RB_2, 400)
time.sleep(1)
pca.set_pwm(RB_2, 306)
time.sleep(1)

print("\nTesting RB_3 (Knee - channel 3)...")
pca.set_pwm(RB_3, 200)
time.sleep(1)
pca.set_pwm(RB_3, 400)
time.sleep(1)
pca.set_pwm(RB_3, 306)
time.sleep(1)

print("\nStopping all servos...")
pca.set_pwm(RB_1, 0)
pca.set_pwm(RB_2, 0)
pca.set_pwm(RB_3, 0)

print("\nTest complete!")