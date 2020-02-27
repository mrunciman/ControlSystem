"""
Defines the size of the structure, geometry of the hydraulic muscles.
Set of functions to find cable lengths based on input point, find volume
required for a given actuator to assume desired length.
"""

import numpy as np
from numpy import linalg as la
import math as mt

# Define parameters of system:
# Side length of equilateral triangle in m
S = 50
# 'Flat' muscle length:
L0 = S/(1 - 2/mt.pi)
# Width of hydraulic muscle in m
W = 30
# Number of length subdivisions
numLs = 5

# Lookup array of equally spaced theta values in interval 0 to pi/2
numPoints = 100
theta = np.linspace(0, mt.pi/2, numPoints)
# Lookup array of cable lengths, using normalised sinc function so divide by pi.
# This avoids divide by zero errors wnen computing np.sin(x)/x
cableLookup = L0*(1 - np.sinc(theta/mt.pi))



def cableLengths(x, y):
    """ Function finds cable lengths in mm from entry points to end effector
    given an input point (x, y) in mm.
    e.g. [cableL, cableR, cableT] = cableLengths(15, 8.6603)
    """
    # x and y define the desired point on the plane in mm.
    # x = x/1000
    # y = y/1000
    # S is the side length in m of the equilateral triangle that defines the entry points

    # a (alpha) is yaw angle wrt global frame
    a = 0
    # Rotation matrix of instrument wrt global frame, assuming no pitch or roll
    # For transformation from base orientation to conventional instrument frame, add further rotation
    RotGI = np.array([[mt.cos(a), -mt.sin(a), 0], [mt.sin(a), mt.cos(a), 0], [0, 0, 1]])

    # r (radius) is radius of end effector
    r = 0 # millimetre
    # This matrix defines the three instrument attachment points wrt instrument frame
    attP = np.array([[r, -r, 0], [-r, r, 0], [0, r, 0]]) #Not actually how it will look, just for test

    # P is the desired position on plane
    P = np.array([[x], [y], [0]])

    # Find desired point in global frame
    PGI = np.dot(RotGI, attP) + P
    
    # Entry point array - defines global frame at lhs corner (from behind instrument)
    # LHS, RHS, TOP corner coordinates, corners of equilateral of side S
    E = np.array([[0, 0, 0], [S, 0, 0], [0.5*S, S*mt.sin(mt.pi/3), 0]])
    E = np.transpose(E)
    # Result is 3x3 matrix, norms of columns are lengths
    R = E - PGI
    # print(R)

    lhsCable = la.norm(R[:,0])
    rhsCable = la.norm(R[:,1])
    topCable = la.norm(R[:,2])
    return lhsCable, rhsCable, topCable



def length2Vol (lengthCable):
    """
    Function returns required volume in muscle to reach desired length.
    Uses lookup table/interpolation to approximate inverse of sin(theta)/theta
    for required length, given contractedL = flatL times sin(theta)/theta
    e.g. length2Vol(17.32)
    """
    # Use lookup tables defined above to find theta angle based on input length
    angle = np.interp(lengthCable, cableLookup, theta)
    # Calculate normalised volume of a beam of arbitrary size
    normV = (angle - mt.cos(angle)*mt.sin(angle))/(angle**2)
    # print(normV)
    # For real volume calc, there are numLs beams of length L0/numLs
    factV = numLs*(W*(L0/numLs)**2)/2
    volume = normV*factV
    return volume
