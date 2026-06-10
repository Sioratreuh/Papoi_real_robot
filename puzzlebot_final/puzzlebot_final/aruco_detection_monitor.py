"""
Constantes y utilidades para detección de ArUco markers.
"""

import numpy as np

# Mapa de marcadores conocidos: ID → (x, y) en mapa mundial
KNOWN_MARKERS = {
    70:  (1.84, -0.295),
    705: (0.93, -1.23),
    706: (2.42, -1.27),
    708: (1.19, -1.25),
    703: (1.21, -2.09),
    702: (0.0, -1.82),
    75:  (2.72, -2.40),
    701: (2.77,  0.0),
    710: (1.86, -0.28),
    711: (3.02, -2.70),
    712: (0.00, -0.92),
    713: (0.37, -3.25),
    714: (0.00, -0.26),
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
