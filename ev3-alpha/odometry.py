from numpy import *
from matplotlib.pyplot import *

data = loadtxt('log.txt')
d = data[:][1:] - data[:][:-1]
#d = loadtxt('new_data.npy')
R = 0.03
L = 0.12
d2r = pi/180.0
x,y,t = 0,0,0
xx = []
yy = []
for row in d:
	x += cos(t)*d2r*R*(row[1]+row[2])/2
	y += sin(t)*d2r*R*(row[1]+row[2])/2
	t += (R/L)*(row[2]-row[1])*d2r
	xx.append(x)
	yy.append(y)
plot(xx,yy)
show()
