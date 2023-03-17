# -----------------------------------------
#
# Программа для поочередного сброса фигур.
#
# -----------------------------------------

import serial
import time

ser = serial.Serial("/dev/ttyUSB0")

# Поочередно сбрасываем по кубику и циллиндру 4 раза
for _ in range(4):
    time.sleep(1)
    ser.write(b"1\n")
    time.sleep(1)
    ser.write(b"2\n")

ser.close()
