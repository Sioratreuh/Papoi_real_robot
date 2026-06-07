"""
Constantes y utilidades para detección de ArUco markers.
"""

import numpy as np

# Mapa de marcadores conocidos: ID → (x, y) en mapa mundial
KNOWN_MARKERS = {
    70:  (1.84, -0.30),
    705: (0.90, -1.20),
    706: (2.39, -1.26),
    708: (1.19, -1.21),
    703: (1.23, -2.07),
    702: (0.28, -1.82),
    75:  (2.74, -2.40),
    701: (2.84,  0.0)
}

# Transformación cámara → base_footprint
# Offset desde cámara a base del robot
CAMERA_TO_BASE_TRANSLATION = (0.0, 0.0, 0.08)  # Offset frontal

# Matriz de rotación cámara → base (identidad si la cámara apunta al frente)
CAMERA_TO_BASE_ROTATION_MATRIX = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
])
