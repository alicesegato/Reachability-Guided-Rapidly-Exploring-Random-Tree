#!/usr/bin/env python
#from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos


# Plot a path in R3 with a unit square obstacle centered at the origin
def plotR2(path):
    fig = plt.figure()
    ax = fig.gca()
    x0,y0,dx,dy = ax.get_position().bounds
    maxd = max(dx, dy)
    width = 6*maxd/dx
    height = 6 * maxd/dy
    fig.set_size_inches((width,height))

    # Plotting the path
    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    ax.plot(X, Y)

    plt.show()


# Read the cspace definition and the path from filename
def readPath(filename):
    lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]

    if len(lines) == 0:
        print "That file's empty!"
        sys.exit(1)

    cspace = lines[0].strip()
    if (cspace != 'R2' and cspace != 'SE2' and cspace!= 'Weird'):
        print "Unknown C-space Identifier: " + cspace
        sys.exit(1)

    data = [[float(x) for x in line.split(' ')] for line in lines[1:]]
    return cspace, data

if __name__ == '__main__':
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'path.txt'

    cspace, path = readPath(filename)

    if cspace == 'R2':
        plotR2(path)
    elif cspace == 'SE2':
        plotSE2(path)
    elif cspace == 'Weird':
        plotWeird(path)
