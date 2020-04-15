"""
Defines the size of the structure, geometry of the hydraulic muscles.
Set of functions to find cable lengths based on input point, find volume
required for a given actuator to assume desired length.
"""

import numpy as np
from numpy import linalg as la
import math as mt

##############################################################
# Structure and actuators
##############################################################
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
# Syringe cross sectional area, diameter = 30 mm
As = mt.pi*(15**2)
# Real volume calc: there are numLs beams of length L0/numLs
factV = (W*(L0)**2)/(2*numLs)
maxV = factV*(2/mt.pi)

# For step count:
# Mapping from step to 1 revolution = 200 steps
# 32 microsteps per step, so 6400 steps per revolution
# Set M0, M1, M2 to set microstep size
# Lead = start*pitch , Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
# Steps/mm = (StepPerRev*Microsteps)/(Lead) 
#          = 200*32/8 = 800 steps/mm
# For speed: pulses/mm * mm/s = pulses/s
StepPerRev = 200
Microsteps = 4
Lead = 8
stepsPMM = (StepPerRev*Microsteps)/(Lead) # steps per mm
stepsPV = stepsPMM/As # Steps per mm^3
maxSteps = stepsPV*maxV # number of steps needed to fill pouch
# print(maxSteps)
timeStep = 0.01 # Inverse of sampling frequency on arduinos

###################################################################
# Lookup table
###################################################################
# Lookup array of equally spaced theta values in interval 0 to pi/2
numPoints = 10000
theta = np.linspace(0, mt.pi/2, numPoints)
# Lookup array of cable contractions/length changes.
# Using normalised sinc function so divide by pi, as this
# avoids divide by zero errors when computing np.sin(x)/x
cableLookup = L0*(1 - np.sinc(theta/mt.pi))
# volLookup = factV*(theta - np.cos(theta)*np.sin(theta))/(theta**2)
# From pouch motors:
# Add correction for when actuators are flat
# P is pressure, Ce = 5.0e-6 Pa-1
    # d = Ce*P
    # cableLength*(1 + d*mt.pi/(mt.pi - 2)) - d
# d/dt(sinc(t)) = (t*cos(t)-sin(t))/t**2
# print(cableLookup)

###################################################################
# Master Controller
###################################################################
speedLimit = 25 # mm/s

###################################################################
# Pulse generation
###################################################################
#Arduino clock frequency
CLOCK_FREQ = 16000000
# Arduino prescalar value
PRESCALER = 8
# numBits = 16
# OCR = np.linspace(0, 2**numBits, (2**numBits)+1)
# f8 = CLOCK_FREQ/(PRESCALER*(OCR+1))
MAX_FREQ = 2000
TWO2_16 = 2**16



def cableLengths(x, y):
    """
    Function finds cable lengths in mm from entry points to end effector
    given an input point (x, y) in mm.
    Also returns pseudoinverse of Jacobian for new point.
    Jacobian is transpose of pose dependent structure matrix
    structure matrix.
    e.g. [cableL, cableR, cableT, Jplus] = cableLengths(15, 8.6603)
    """
    # x and y define the desired point on the plane in mm.
    # x = x/1000
    # y = y/1000

    # a (alpha) is yaw angle wrt global frame
    a = 0
    # Rotation matrix of instrument wrt global frame, assuming no pitch or roll
    # For transformation from base orientation to conventional instrument frame, add further rotation
    # This is an input
    RotGI = np.array([[mt.cos(a), -mt.sin(a), 0], [mt.sin(a), mt.cos(a), 0], [0, 0, 1]])

    # r (radius) is radius of end effector
    r = 0 # millimetre
    # This matrix defines the three instrument attachment points wrt instrument frame
    attP = np.array([[r, -r, 0], [-r, r, 0], [0, r, 0]]) #Not actually how it will look, just for test

    # P is the desired position on plane
    P = np.array([[x], [y], [0]])

    # Find cable attachment points in global frame
    PGI = np.dot(RotGI, attP) + P
    # print(PGI)
    
    # Entry point array - global frame defined at lhs corner (from behind instrument)
    # LHS, RHS, TOP corner coordinates, corners of equilateral of side S
    E = np.array([[0, 0, 0], [S, 0, 0], [0.5*S, S*mt.sin(mt.pi/3), 0]])
    E = np.transpose(E)
    # Result is 3x3 matrix of vectors in global frame pointing from attachment points to
    # entry points, norms of columns are cable lengths
    L = E - PGI
    lhsCable = la.norm(L[:,0])
    rhsCable = la.norm(L[:,1])
    topCable = la.norm(L[:,2])
    # print(L)

    # Compute the structure matrix A from cable unit vectors and cable attachment points 
    # in global frame, PGI
    # Find cable unit vectors
    u1 = L[:,0]/lhsCable    ### FILTER OUT ZERO LENGTH ERRORS
    u2 = L[:,1]/rhsCable
    u3 = L[:,2]/topCable
    u = np.array([u1, u2, u3])
    u = np.transpose(u)
    # print(u)
    # Find cross products of cable unit vectors and attachment points
    pCrossU1 = np.cross(PGI[:,0], u1)
    pCrossU2 = np.cross(PGI[:,1], u2)
    pCrossU3 = np.cross(PGI[:,2], u3)
    pCrossU = np.array([pCrossU1, pCrossU2, pCrossU3])
    pCrossU = np.transpose(pCrossU)
    # print(pCrossU)

    # Construct Jacobian from transpose of structure matrix
    Jacobian = np.concatenate((u, pCrossU), axis = 0)
    # print("Jacobian matrix is \n", Jacobian)
    # Use only top two rows
    Jplus = np.linalg.pinv(Jacobian[0:2,:])
    # print(Jplus)
    # speeds = np.array([0,10*0.5774,3,4,5,6])
    # speeds = np.transpose(speeds)
    # print(np.dot(Jplus, speeds[0:2]))
    
    # rank(A) # Check for singular configuration
    # If rank(A) < no controlled DOFs, A is singular

    return lhsCable, rhsCable, topCable, Jplus



def length2Vol (currentCable, targetCable):
    """
    Function returns required volume in ml in muscle to reach desired length,
    the piston speed required, as well as displacement from 0 position on pump.
    Uses lookup table/interpolation to approximate inverse of sin(theta)/theta
    for required length, given contractedL = flatL times sin(theta)/theta
    e.g. length2Vol(17.32, 18)
    """
    Lc = S - targetCable
    Lc0 = S - currentCable
    cableSpeed = (Lc-Lc0)/timeStep
    # Use lookup tables defined above to find theta angle based on input length
    angle = np.interp(Lc, cableLookup, theta)
    # Calculate normalised volume of a beam of arbitrary size
    normV = (angle - mt.cos(angle)*mt.sin(angle))/(angle**2)    ### FILTER OUT ANGLE = 0 ERRORS
    # print(normV)
    # Real volume calc: multiply theta-dependent part of 
    # volume by constant geometry-dependent factor 
    volume = normV*factV
    # angle = np.interp(volume, volLookup, theta)
    # Find distance syringe pump has to move to reach desired volume
    lengthSyringe = volume/As   # Convert to stepCount?
    stepCount = round(lengthSyringe*stepsPMM)
    # stepCount = stepCount*(lengthSyringe/abs(lengthSyringe))
    # stepCount = int(stepCount)
    # Convert from mm^3 to ml
    # volume = volume / 1000
    return volume, cableSpeed, stepCount



###### MAKE THIS FUNCTION ACCPET CURRENT VOLUME, SO CALC ISN'T REPEATED
def volRate(cV, cCable, tCable):
    """
    Makes linear approximation of volume rate.
    Returns volume rate in mm^3/s, syringe speed in mm/s, pulse frequency 
    in Hz. Current and target volumes and syringe displacements are also returned.
    """
    # USE CABLE SPEED AND INITIAL CABLE LENGTH AS INPUT?
    # Find current and target volume and displacement of syringe
    # [cV, cD] = length2Vol(cCable)
    [tV, tSpeed, stepNo] = length2Vol(cCable, tCable)
    # Calculate linear approximation of volume rate:
    volDiff = tV-cV
    vDot = (volDiff)/timeStep #timeSecs  # mm^3/s
    dDot = (vDot/As) # mm/s

    # For step count:
    # Mapping from step to 1 revolution = 200 steps
    # Set M0, M1, M2 to set microstep size
    # Lead = start*pitch , Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
    # Steps/mm = (StepPerRev*Microsteps)/(Lead) 
    # For speed: pulses/mm * mm/s = pulses/s
    fStep = stepsPMM*dDot

    return tV, vDot, dDot, fStep, stepNo, tSpeed



def freqScale(fL, fR, fT):
    """
    Returns output compare register values for use in interrupts.
    If any frequency exceeds stepper MAX_FREQ then all are scaled
    down to preserve velocity vector direction.
    """
    fList = np.array([fL, fR, fT])
    fAbs = np.absolute(fList)
    OCR = np.array([0, 0, 0])
    fRound = np.array([0, 0, 0])
    # Find OCR and scale if any of the frequency values is non-zero
    if np.any(fList):
        fSign = np.sign(fList)
        fMax = np.amax(fAbs)
        # If largest frequency is above MAX_FREQ then scale
        if fMax > MAX_FREQ:
            fFact = MAX_FREQ/fMax
            fScaled = fFact*fAbs
            # OCR = np.ceil((CLOCK_FREQ/(PRESCALER*fScaled))-1)
            fScaled = fScaled*fSign
            fList = fScaled
            # print("Original: ", fList*timeStep, "   Scaled: ", fScaled*timeStep)
        # Else use unscaled frequencies
        # else:
            # OCR = np.ceil((CLOCK_FREQ/(PRESCALER*fAbs))-1)
        fRound = np.around(timeStep*fList*fSign)
        fRound = fRound*fSign
        # fRound = np.int_(fRound)
        # OCR = fSign*OCR
    # Check for low frequency limit/upper limit of OCR, due to 16 bit timer
    # if OCR[OCR > TWO2_16].size > 0:
    #     OCRMax = np.amax(OCR)
    #     OCRFact = TWO2_16/OCRMax
    # OCR[OCR > TWO2_16] = TWO2_16
    # OCR = np.int_(OCR)

    return OCR[0], OCR[1], OCR[2], fRound[0], fRound[1], fRound[2]



def cableSpeeds (cX, cY, tX, tY, JacoPlus, timeSecs):
    """
    Returns required cable length change rates to reach target
    from current point within master sampling period.
    tX is target X, cX is current X.
    Desired speed in X and Y found by multiplying difference 
    by sampling frequency of master. JacoPlus is pseudoinverse of
    Jacobian at a given point.
    """
    timeSecs = 0.01

    # TARGET POINT, CURRENT POINT, TARGET SPEED ARE INPUTS
    # USED TO FIND CABLE LENGTHS AND RATE OF CABLE LENGTH CHANGE
    diffX = tX - cX
    #############################################
    # Testing to see how it looks with fixed time step, in this case 0.01 s for a 100 Hz loop
    tVx =  diffX/timeSecs
    diffY = tY - cY
    tVy = diffY/timeSecs
    vMag = mt.sqrt(tVx**2 + tVy**2)
    # Check if point can be reached without exceeding speed limit.
    # Scale back velocity to speed limit and calculate actual position.

    if vMag > speedLimit:
        # Calculate unit velocity vector and multiply by 
        # limit to scale down
        tVx = (tVx/vMag)*speedLimit
        tVy = (tVy/vMag)*speedLimit
        # Find actual position after movement
        # actX = cX + (diffX*speedLimit/mt.sqrt(diffX**2 + diffY**2))*timeSecs
        # actY = cY + (diffY*speedLimit/mt.sqrt(diffX**2 + diffY**2))*timeSecs
        # print(actX, actY)

    v = np.array([[tVx],[tVy]])

    # IK premultiplying input end effector velocity vector with pseudoinverse J
    # to yield joint velocities.
    cableRates = np.dot(JacoPlus, v)
    # velocity = np.dot(J, cableRates)
    # print(velocity)
    lhsSpeed = cableRates[0] # These are all array type still
    rhsSpeed = cableRates[1]
    topSpeed = cableRates[2]
    # print(v[0], lhsSpeed)
    return lhsSpeed, rhsSpeed, topSpeed