"""
Defines the size of the structure, geometry of the hydraulic muscles.
Set of functions to find cable lengths based on input point, find volume
required for a given actuator to assume desired length.
"""

import numpy as np
from numpy import linalg as la
import math as mt

# Define parameters of system:
# Side length of equilateral triangle in mm
S = 50
# 'Flat' muscle length:
L0 = S/(1 - 2/mt.pi)
# Excess length of cable between entry point and muscle, in mm
Lx = 10
# Total length of cable in flat/resting state
Lt = L0 + Lx + S
# print(Lt)
# Width of hydraulic muscle in mm
W = 30
# Number of length subdivisions
numLs = 10

# Lookup array of equally spaced theta values in interval 0 to pi/2
numPoints = 1000
theta = np.linspace(0, mt.pi/2, numPoints)
# Lookup array of cable contractions/length changes.
# Using normalised sinc function so divide by pi, as this
# avoids divide by zero errors when computing np.sin(x)/x
cableLookup = L0*(1 - np.sinc(theta/mt.pi))
# print(cableLookup)

# Syringe cross sectional area, diameter = 30 mm
As = mt.pi*30**2

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
    Function returns required volume in ml in muscle to reach desired length,
    as well as displacement from 0 position on linear actuator.
    Uses lookup table/interpolation to approximate inverse of sin(theta)/theta
    for required length, given contractedL = flatL times sin(theta)/theta
    e.g. length2Vol(17.32)
    """
    Lc = S - lengthCable
    # Use lookup tables defined above to find theta angle based on input length
    angle = np.interp(Lc, cableLookup, theta)
    # Calculate normalised volume of a beam of arbitrary size
    normV = (angle - mt.cos(angle)*mt.sin(angle))/(angle**2)
    # print(normV)
    # For real volume calc, there are numLs beams of length L0/numLs
    factV = numLs*(W*(L0/numLs)**2)/2
    volume = normV*factV
    lengthSyringe = volume/As
    volume = volume / 1000
    return volume, lengthSyringe

# Add length2step count function?
# Control frequency of interrupts to control speed? 
# 'Gearing' using microstepping

# Function taking target point and time to reach it, can be called recursively or called with a list of targets and times

# Function taking input point from master over set time period and finding target speed.
# Use target speed to find necessary rate of cable length changes (qdot).
# TARGET POINT, CURRENT POINT, TARGET SPEED ARE INPUTS USED TO FIND CABLE LENGTHS AND RATE OF CABLE LENGTH CHANGE
