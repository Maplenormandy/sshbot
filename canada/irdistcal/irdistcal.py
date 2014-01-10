import serial
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt

ser = serial.Serial(
        port = '/dev/ttyACM0',
        baud = 115200
        )

voltages = np.zeros(0)
distances = np.zeros(0)
invdist = np.zeros(0)

while True:
    print "distance (m), negative to quit",
    dist = float(input())
    if dist<0:
        break

    volts = np.zeros(32)

    ser.write('o')
    for i in range(32):
        val = float(ser.readline(eol='\r\n'))
        print "got", val
        volts[i] = val

    np.append(voltages, volts)
    np.append(distances, [dist]*32)
    np.append(invdist, [1.0/dist]*32)

plt.subplot(2,1,1)
plt.plot(invdist, voltage)
plt.xlabel('1 / dist')
plt.ylabel('voltage')
plt.subplot(2,1,2)
plt.plot(distances, voltage)
plt.xlabel('dist')
plt.ylabel('voltage')

plt.show()

slope, intercept, r_value, p_value, std_err = stats.linregress(voltages,invdist)

print "slope:", slope
print "intercept:", intercept
print "r-value", r_value
print "std_err", srd_err

