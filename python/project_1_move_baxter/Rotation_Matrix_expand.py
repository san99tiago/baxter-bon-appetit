#!/usr/bin/env python
# Rotation MAtrix Expand
# Importamos modulos importantes
import numpy as np
import math
import TransformationMatrix as tff


def rotation_matrix_expand(angles):
    # Definimos primero si los angulos son dados en radianes o en grados
    # Definimos parametros importantes
    gamma = angles[0]
    beta = angles[1]
    alpha = angles[2]

    # Definimos la matris de rotacion extida
    R_Matrix_Out = np.dot(np.dot(tff.transformation_matrix('z', alpha, [0, 0, 0])[:3, :3],
                                 tff.transformation_matrix('y', beta, [0, 0, 0])[:3, :3]),
                          tff.transformation_matrix('x', gamma, [0, 0, 0])[:3, :3])

    # Retornamos la matriz de rotacion
    return R_Matrix_Out


if __name__ == '__main__':
    pass
