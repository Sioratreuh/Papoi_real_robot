#!/usr/bin/env python3
import numpy as np

# Instrucciones: Captura aquí tus mediciones del mundo real (minimo 15-20 iteraciones).
# datos_x: Distancia total avanzada (Meta: 1.0m)
# datos_y: Desviación lateral (Meta: 0.0m)
# datos_theta_deg: Ángulo final medido con transportador en GRADOS.

datos_x = [1.02, 0.98, 1.01, 0.99, 1.00] # Ejemplos a reemplazar
datos_y = [0.02, -0.01, 0.01, 0.00, -0.02] 
datos_theta_deg = [1.5, -1.0, 0.5, 0.0, -0.5] 

# Procesamiento estadístico
datos_theta_rad = np.deg2rad(datos_theta_deg)

var_x = np.var(datos_x, ddof=1)
var_y = np.var(datos_y, ddof=1)
var_theta = np.var(datos_theta_rad, ddof=1)

print("-" * 45)
print("RESULTADOS ESTADÍSTICOS DEL EXPERIMENTO FÍSICO")
print("-" * 45)
print(f"Varianza longitudinal (X) -> self.var_x = {var_x:.8f}")
print(f"Varianza lateral (Y)      -> self.var_y = {var_y:.8f}")
print(f"Varianza angular (Theta)  -> self.C     = {var_theta:.8f}")
print("-" * 45)