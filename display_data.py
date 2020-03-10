import matplotlib.pyplot as plt
import math
import numpy as np

file = open("/home/giulio/Scrivania/intrinsically_stable_mpc/zmp_meas_x.txt", "r") 
zmp_x = []
for line in file: 
    a = line.split("\n")
    to_app = float(a[0])
    print(float(a[0]))
    if(len(zmp_x) > 3):
        alpha = 0.9
        
        uno = float(a[0])
        due = zmp_x[len(zmp_x) -1]
        to_app = uno*alpha + (1-alpha)*due
        #to_app = np.mean([uno,due])
    zmp_x.append(to_app)
    print(zmp_x)

file = open("/home/giulio/Scrivania/intrinsically_stable_mpc/zmp_meas_y.txt", "r") 
zmp_y = []
for line in file: 
    a = line.split("\n")
    to_app = a[0]
    if(len(zmp_y) > 3):
        uno = zmp_y[len(zmp_y) -1]
        due = float(a[0])
        to_app = uno*alpha + (1-alpha)*due
    zmp_y.append(float(to_app))

#print("x")
#print(zmp_x)
#print("y")
#print(zmp_y)



plt.plot(zmp_y)
plt.show()