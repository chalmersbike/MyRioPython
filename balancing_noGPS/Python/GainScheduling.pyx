import math
import cython
import numpy as np
cimport numpy as np


cdef class GainScheduling:
    cdef double P_inner, I_inner, P_outer
    cdef list PiPoly, IiPoly, PoPoly
    cdef bint P_innerON, I_innerON, P_outerON
    def __init__(self, PiPoly = [ -0.3250, 3.5636, -14.2051, 23.0150],
        IiPoly = [0, 0, 0, 0],
        PoPoly = [-0.0113, 0.1233, -0.4798, 1.6257]):

        # PiPoly = [-0.3255, 3.5690, -14.2265, 23.0496],
        # IiPoly = [-0.0538, 0.5895, -2.3497, 3.8070],
        # PoPoly = [-0.0156, 0.1844, -0.8004, 3.6999]
        self.PiPoly = PiPoly
        self.IiPoly = IiPoly
        self.PoPoly = PoPoly



        if sum(self.PiPoly) == 0:
            self.P_innerON = False
        else:
            self.P_innerON = True
        if sum(self.IiPoly) == 0:
            self.I_innerON = False
        else:
            self.I_innerON = True
        if sum(self.PoPoly) == 0:
            self.P_outerON = False
        else:
            self.P_outerON = True

    cpdef tuple calculatePolynomials(self, double v):
        cdef double Pi, Ii, Po

        if self.P_innerON:
            Pi = (self.PiPoly[0] * v**3  +
                  self.PiPoly[1] *  v**2  +
                  self.PiPoly[2] * v  +
                  self.PiPoly[3])
        else:
            Pi = 0.0
        if self.I_innerON:
            Ii = (self.IiPoly[0] * v ** 3 +
                  self.IiPoly[1] * v ** 2 +
                  self.IiPoly[2] * v +
                  self.IiPoly[3])
        else:
            Ii = 0.0
        if self.P_outerON:
            Po = (self.PoPoly[0] * v ** 3 +
                  self.PoPoly[1] * v ** 2 +
                  self.PoPoly[2] * v +
                  self.PoPoly[3])
        else:
            Po = 0.0
        return Pi, Ii, Po

