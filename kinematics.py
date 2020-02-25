import numpy as np
from numpy import linalg as la
import math

### cableLengths() ###
# Function finds cable lengths from entry points to end effector
# given an input point (x, y).

def cableLengths(x, y):
    S = 30
    # x and y define the desired point on the plane.
    # S is the side length of the equilateral triangle defining the entry points

    # a (alpha) is yaw angle wrt global frame
    a = 0
    # Rotation matrix of instrument wrt global frame, assuming no pitch or roll
    # For transformation from base orientation to conventional instrument frame, add further rotation
    RotGI = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])

    # r (radius) is radius of end effector
    r = 0 # millimetre
    # This matrix defines the three instrument attachment points wrt instrument frame
    attP = np.array([[r, -r, 0], [-r, r, 0], [0, r, 0]]) #Not actually how it will look, just for test

    # P is the desired position on plane
    P = np.array([[x], [y], [0]])
    # print(P)

    # Find desired point in global frame
    PGI = np.dot(RotGI, attP) + P
    
    # Entry point array - defines global frame at lhs corner (from behind instrument)
    # LHS, RHS, TOP corner coordinates, corners of equilateral of side S
    E = np.array([[0, 0, 0], [S, 0, 0], [0.5*S, S*math.sin(math.pi/3), 0]])
    E = np.transpose(E)
    # print(E)
    # Result is 3x3 matrix, norms of columns are lengths
    R = E - PGI
    # print(R)

    lhsCable = la.norm(R[:,0])
    rhsCable = la.norm(R[:,1])
    topCable = la.norm(R[:,2])
    return lhsCable, rhsCable, topCable

