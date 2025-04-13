import time
import board
import busio
import adafruit_amg88xx

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_amg88xx.AMG88XX(i2c)

while True:
    for row in sensor.pixels:
        print(["{0:.1f}".format(temp) for temp in row])
    print("\n")
    time.sleep(1)