# coding:utf-8

import matplotlib.pyplot as plt
import csv
import numpy

filename = '../output/bezier.txt'

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

x = []
y = []

cx = [0, 0, 1, 3, 2]
cy = [0, 1, 2, 0, -3]

with open(filename, 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        x.append(float(line[0]))
        y.append(float(line[1]))

plt.plot(x, y, label='bezier points', c='r', alpha=0.75, marker='o', ms=4)
plt.scatter(cx, cy, label='control points', c='g', alpha=0.75, marker='X', s=60)

plt.grid(ls='--', alpha=0.5)
plt.legend()

plt.title('Bezier Spline')

plt.show()
