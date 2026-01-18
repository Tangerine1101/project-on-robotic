import numpy as np
try:
    from .FWK_Degree import IK_fulls_1
except ImportError:
    from FWK_Degree import IK_fulls_1
x_mm = 320.0
y_mm = -200.0
z_mm = 85.0
T = np.array([
            [1.0, 0.0, 0.0, x_mm],
            [0.0, -1.0, 0.0, y_mm], 
            [0.0, 0.0, -1.0, z_mm], 
            [0.0, 0.0, 0.0, 1.0]
        ])
solve = IK_fulls_1(T)
if solve.size > 0:
    # Use [0] to extract the scalar value from the first row
    # This turns the 1-element array into a standard float
    a = solve[0, 0]
    b = solve[0, 1]
    c = solve[0, 2]
    
    # Now :.2f will work because a, b, and c are floats, not arrays!
    print(f"echo \"moveto -a {a:.2f} -b {b:.2f} -c {c:.2f}\" > /dev/ttyACM0")
else:
    print("No valid IK solution found.")
