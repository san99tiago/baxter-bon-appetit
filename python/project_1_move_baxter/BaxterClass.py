#!/usr/bin/env python
# Importamos modulos importantes
import IPK
import FPK
import numpy as np
import math

# Definimos la clase BAXTER


class baxterClass():
    # Definimos los atributos de baxter
    def __init__(self):
        # Medidas de los brazos de baxter
        self.l0 = 0.27035
        self.l1 = 0.06900
        self.l2 = 0.36435
        self.l3 = 0.06900
        self.l4 = 0.37429
        self.l5 = 0.01000
        self.l6 = 0.36830
        # Medidas adicionales
        self.L = 0.27800
        self.h = 0.06400
        self.H = 1.10400# - 0.0601

    # Definimos los metodos de baxter
    def fpk(self, dof, arm, ndof):
        Two_t = FPK.FPK(dof, [self.l0, self.l1,
                              self.l2, self.l3, self.l4, self.l5, self.l6, self.L, self.h, self.H], arm, ndof)
        return Two_t

    def ipk(self, FPK, arm, type):
        vdof = IPK.IPK(FPK, [self.l0, self.l1, self.l2,
                             self.l3, self.l4, self.l6, self.L, self.h, self.H], arm, type)
        return vdof


if __name__ == '__main__':
    pass
