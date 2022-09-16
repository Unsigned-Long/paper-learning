# coding:utf-8

import matplotlib.pyplot as plt
import csv
import numpy

resultFileName = '../output/bezier.txt'
# resultFileName = '../output/uniformBSpline.txt'

ControlPointsFileName = '../output/controlPoints.txt'

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

x = []
y = []

cx = []
cy = []

with open(resultFileName, 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        x.append(float(line[0]))
        y.append(float(line[1]))

with open(ControlPointsFileName, 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        cx.append(float(line[0]))
        cy.append(float(line[1]))

plt.plot(x, y, label='B-spline points', c='r', alpha=0.75, marker='o', ms=4)
plt.scatter(cx, cy, label='control points', c='g', alpha=0.75, marker='X', s=60)

plt.grid(ls='--', alpha=0.5)
plt.legend()

plt.title('Bezier Curve')

plt.show()
