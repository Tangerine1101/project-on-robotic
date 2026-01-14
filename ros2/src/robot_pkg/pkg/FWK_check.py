import numpy as np
try:
    from .FWK_Degree import IK_fulls_1
except ImportError:
    from FWK_Degree import IK_fulls_1
x_mm = 238.0
y_mm = -107.0
z_mm = 0.0
T = np.array([
            [1.0, 0.0, 0.0, x_mm],
            [0.0, -1.0, 0.0, y_mm], 
            [0.0, 0.0, -1.0, z_mm], 
            [0.0, 0.0, 0.0, 1.0]
        ])
solve = IK_fulls_1(T)
print(f"location: '{solve}'")
