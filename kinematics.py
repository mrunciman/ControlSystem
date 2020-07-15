"""
Defines the size of the structure, geometry of the hydraulic muscles.
Set of functions to find cable lengths based on input point, find volume
required for a given actuator to assume desired length.
"""

import numpy as np
from numpy import linalg as la
import math as mt

sideLength = 25

class kine:
    sideLength0 = sideLength
    def __init__(self):

        ##############################################################
        # Structure and actuators
        ##############################################################
        # Define parameters of system:
        # Side length of equilateral triangle in mm
        self.sideLength = sideLength
        # 'Flat' muscle length:
        self.L0 = self.sideLength/(1 - 2/mt.pi)
        # Excess length of cable between entry point and muscle, in mm
        # self.Lx = 10
        # Total length of cable in flat/resting state
        # self.Lt = self.L0 + self.Lx + self.sideLength
        # print(Lt)
        # Width of hydraulic muscle in mm
        self.muscleWidth = 30
        # Number of length subdivisions
        self.numLs = 3
        # Syringe cross sectional area, diameter = 30 mm
        self.As = mt.pi*(15**2) # mm^2
        # Real volume calc: there are numLs beams of length L0/numLs
        self.factV = (self.muscleWidth*(self.L0)**2)/(2*self.numLs)
        self.maxV = self.factV*(2/mt.pi)

        # For step count:
        # Mapping from step to 1 revolution = 200 steps
        # 32 microsteps per step, so 6400 steps per revolution
        # Set M0, M1, M2 to set microstep size
        # Lead = start*pitch , Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
        # Steps/mm = (StepPerRev*Microsteps)/(Lead) 
        #          = 200*4/8 = 100 steps/mm
        # For speed: pulses/mm * mm/s = pulses/s
        self.StepPerRev = 200
        self.Microsteps = 4
        self.Lead = 8
        self.stepsPMM = (self.StepPerRev*self.Microsteps)/(self.Lead) # steps per mm
        self.stepsPV = self.stepsPMM/self.As # Steps per mm^3
        self.maxSteps = self.stepsPV*self.maxV # number of steps needed to fill pouch
        # print(maxSteps)
        self.timeStep = 6/125 # Inverse of sampling frequency on arduinos


        ###################################################################
        # Pulse generation
        ###################################################################
        #Arduino clock frequency
        self.CLOCK_FREQ = 16000000
        # Arduino prescalar value
        self.PRESCALER = 8
        # numBits = 16
        # OCR = np.linspace(0, 2**numBits, (2**numBits)+1)
        # f8 = CLOCK_FREQ/(PRESCALER*(OCR+1))
        self.MAX_FREQ = 1500
        self.TWO2_16 = 2**16

        ###################################################################
        # Lookup table
        ###################################################################
        # Lookup array of equally spaced theta values in interval 0 to pi/2
        self.numPoints = 10000
        self.theta = np.linspace(0, mt.pi/2, self.numPoints)
        # theta = np.linspace(((mt.pi/2)/numPoints), mt.pi/2, numPoints-1)
        # Lookup array of cable contractions/length changes.
        # Numpy uses unnormalised sinc function so divide by pi. Using sinc
        # avoids divide by zero errors returned when computing np.sin(x)/x
        self.cableLookup = self.L0*(1 - np.sinc(self.theta/mt.pi))
        # Avoid division by zero by prepending volLookup with zero.
        self.thetaNoZero = self.theta[1:self.numPoints]
        self.volLookup = (self.thetaNoZero - np.cos(self.thetaNoZero)*np.sin(self.thetaNoZero))/(self.thetaNoZero**2)
        self.volLookup = np.insert(self.volLookup, 0, 0, axis=None)
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
        self.speedLimit = 25 # mm/s
        

    def cableLengths(self, x, y):
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
        # Z axis out of screen - conssitent with 'master' controller

        # P is the desired position on plane
        P = np.array([[x], [y], [0]])

        # Find cable attachment points in global frame
        PGI = np.dot(RotGI, attP) + P
        # print(PGI)
        
        # Entry point array - global frame defined at bottom lhs corner (from behind instrument)
        # LHS, RHS, TOP corner coordinates, corners of equilateral of side S
        E = np.array([[0, 0, 0], [self.sideLength, 0, 0], [0.5*self.sideLength, self.sideLength*mt.sin(mt.pi/3), 0]])
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



    def length2Vol (self, currentCable, targetCable):
        """
        Function returns required volume in ml in muscle to reach desired length,
        the piston speed required, as well as displacement from 0 position on pump.
        Uses lookup table/interpolation to approximate inverse of sin(theta)/theta
        for required length, given contractedL = flatL times sin(theta)/theta
        e.g. length2Vol(17.32, 18)
        """
        Lc = self.sideLength - targetCable
        Lc0 = self.sideLength - currentCable
        cableSpeed = (Lc-Lc0)/self.timeStep
        # Use lookup tables defined above to find theta angle based on input length
        angle = np.interp(Lc, self.cableLookup, self.theta)
        # Calculate normalised volume of a beam of arbitrary size
        normV = (angle - mt.cos(angle)*mt.sin(angle))/(angle**2)    ### FILTER OUT ANGLE = 0 ERRORS
        # print(angle)
        # Real volume calc: multiply theta-dependent part of 
        # volume by constant geometry-dependent factor 
        volume = normV*self.factV
        # angle = np.interp(volume, volLookup, theta)
        # Find distance syringe pump has to move to reach desired volume
        lengthSyringe = volume/self.As
        # print(volume/1000)

        stepCount = round(lengthSyringe*self.stepsPMM)

        # Find discretised actuator length actuator actually commanded to go to:
        lengthDisc = stepCount/self.stepsPMM
        volDisc = lengthDisc*self.As
        normVDisc = volDisc/self.factV
        angleDisc = np.interp(normVDisc, self.volLookup, self.theta)
        LDisc = self.L0*mt.sin(angleDisc)/angleDisc
        LcDisc = self.L0 - LDisc
        # print(Lc, LcDisc)
        # Convert from mm^3 to ml
        # volume = volume / 1000
        return volume, cableSpeed, stepCount, LcDisc



    ###### MAKE THIS FUNCTION ACCPET CURRENT VOLUME, SO CALC ISN'T REPEATED
    def volRate(self, cV, cCable, tCable):
        """
        Makes linear approximation of volume rate.
        Returns volume rate in mm^3/s, syringe speed in mm/s, pulse frequency 
        in Hz. Current and target volumes and syringe displacements are also returned.
        """
        # USE CABLE SPEED AND INITIAL CABLE LENGTH AS INPUT?
        # Find current and target volume and displacement of syringe
        # [cV, cD] = length2Vol(cCable)
        [tV, tSpeed, stepNo, LcDisc] = self.length2Vol(cCable, tCable)
        # Calculate linear approximation of volume rate:
        volDiff = tV-cV
        vDot = (volDiff)/self.timeStep #timeSecs  # mm^3/s
        dDot = (vDot/self.As) # mm/s

        # For step count:
        # Mapping from step to 1 revolution = 200 steps
        # Set M0, M1, M2 to set microstep size
        # Lead = start*pitch , Lead screw is 4 start, 2 mm pitch, therefore Lead = 8
        # Steps/mm = (StepPerRev*Microsteps)/(Lead) 
        # For speed: pulses/mm * mm/s = pulses/s
        fStep = self.stepsPMM*dDot

        return tV, vDot, dDot, fStep, stepNo, tSpeed, LcDisc



    def freqScale(self, fL, fR, fT):
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
            if fMax > self.MAX_FREQ:
                fFact = self.MAX_FREQ/fMax
                fScaled = fFact*fAbs
                OCR[fScaled != 0] = np.ceil((self.CLOCK_FREQ/(self.PRESCALER*fScaled[fScaled != 0]))-1)
                fScaled = fScaled*fSign
                fList = fScaled
                # print("Original: ", fList*timeStep, "   Scaled: ", fScaled*timeStep)
            # Else use unscaled frequencies
            else:
                OCR[fAbs != 0] = np.ceil((self.CLOCK_FREQ/(self.PRESCALER*fAbs[fAbs != 0]))-1)
                # for i in range(3):
                #     if fAbs[i] == 0:
                #         OCR[i] = 0
                #     else:
                #         OCR[i] = np.ceil((CLOCK_FREQ/(PRESCALER*fAbs[i]))-1)
            fRound = np.around(self.timeStep*fList*fSign)
            fRound = fRound*fSign
            fRound = np.int_(fRound)
            OCR = fSign*OCR
        # Check for low frequency limit/upper limit of OCR, due to 16 bit timer
        OCR[OCR > self.TWO2_16] = self.TWO2_16
        OCR = np.int_(OCR)

        return OCR[0], OCR[1], OCR[2], fRound[0], fRound[1], fRound[2]



    def cableSpeeds (self, cX, cY, tX, tY, JacoPlus, timeSecs):
        """
        Returns required cable length change rates to reach target
        from current point within master sampling period.
        tX is target X, cX is current X.
        Desired speed in X and Y found by multiplying difference 
        by sampling frequency of master. JacoPlus is pseudoinverse of
        Jacobian at a given point.
        """
        timeSecs = self.timeStep

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

        if vMag > self.speedLimit:
            # Calculate unit velocity vector and multiply by 
            # limit to scale down
            tVx = (tVx/vMag)*self.speedLimit
            tVy = (tVy/vMag)*self.speedLimit
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