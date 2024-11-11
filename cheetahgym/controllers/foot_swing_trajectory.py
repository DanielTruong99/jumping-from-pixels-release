import numpy as np

# cubic bezier interpolation between y0 and yf. x is between 0 and 1
def bez(y0, yf, x):
    yDiff = yf - y0
    bezier = x * x * x + 3 * x * x * (1 - x)
    return y0 + bezier * yDiff

def dbez(y0, yf, x):
    yDiff = yf - y0
    bezier = 6 * x * (1 - x)
    return bezier * yDiff

def d2bez(y0, yf, x):
    yDiff = yf - y0
    bezier = 6 - 12 * x
    return bezier * yDiff


# foot swing trajectory class
class FootSwingTrajectory:
    def __init__(self):
        self._p0 = np.zeros(3)
        self._pf = np.zeros(3)
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._a = np.zeros(3)
        self._height = 0

    def setInitialPosition(self, p0):
        self._p0 = p0

    def setFinalPosition(self, pf):
        self._pf = pf

    def setHeight(self, h):
        self._height = h

    def getPosition(self):
        return self._p

    def getVelocity(self):
        return self._v

    def getAcceleration(self):
        return self._a

    def computeSwingTrajectoryBezier(self, phase, swingTime):
        self._p = bez(self._p0, self._pf, phase)
        self._v = dbez(self._p0, self._pf, phase) / swingTime
        self._a = d2bez(self._p0, self._pf, phase) / swingTime**2

        if phase < 0.5:
            zp = bez(self._p0[2], self._p0[2] + self._height, phase*2)
            zv = dbez(self._p0[2], self._p0[2] + self._height, phase*2) * 2 / swingTime
            za = d2bez(self._p0[2], self._p0[2] + self._height, phase*2) * 4 / swingTime**2
        else:
            zp = bez(self._p0[2] + self._height, self._pf[2], phase*2 - 1)
            zv = dbez(self._p0[2] + self._height, self._pf[2], phase*2 - 1) * 2 / swingTime
            za = d2bez(self._p0[2] + self._height, self._pf[2], phase*2 - 1) * 4 / swingTime**2

        self._p[2] = zp
        self._v[2] = zv
        self._a[2] = za

