import numpy as np
import matplotlib.pyplot as plt



f = open(".\\all.txt")
M = []
x = [i for i in range(0,128)]
line = f.readline()
while line:
    print(line)
    line = float(line)
    M.append(line)
    line = f.readline()

plt.figure(1)
plt.xlim((0,128))
plt.plot(x,M[0:128],alpha = 0.8,color = 'red',label='normal')
plt.plot(x,M[128:256],alpha = 0.8,color = 'orange',label='O')
plt.plot(x,M[256:384],alpha = 0.8,color = 'c',label='puff')
plt.plot(x,M[384:512],alpha = 0.8,color = 'green',label='grin')
plt.plot(x,M[512:640],alpha = 0.8,color = 'blue',label='stare')
plt.legend(loc='upper right')
plt.show()

