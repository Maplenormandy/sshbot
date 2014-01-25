import serial
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt

ser = serial.Serial(
        port = '/dev/ttyACM0',
        baudrate = 115200
        )

voltages = np.zeros(0)
distances = np.zeros(0)
invdist = np.zeros(0)

# 6, 10, 14, 22
while True:
    print "distance (in), negative to quit:",
    dist = float(input()) * 0.0254
    if dist<0:
        break

    volts = np.zeros(32)

    ser.write('o')
    for i in range(32):
        val = ser.readline().strip()
        print "got", val, '--', float(val)
        volts[i] = float(val)

    voltages = np.append(voltages, volts)
    distances = np.append(distances, [dist]*32)
    invdist = np.append(invdist, [1.0/dist]*32)

plt.subplot(2,1,1)
plt.plot(invdist, voltages, '.')
plt.xlabel('1 / dist')
plt.ylabel('voltage')
plt.subplot(2,1,2)
plt.plot(distances, voltages, ',')
plt.xlabel('dist')
plt.ylabel('voltage')

plt.show()

slope, intercept, r_value, p_value, std_err = stats.linregress(voltages,invdist)

print "slope:", slope
print "intercept:", intercept
print "r-value", r_value
print "std_err", std_err

