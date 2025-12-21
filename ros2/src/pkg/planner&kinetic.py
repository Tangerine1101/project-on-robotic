import numpy as np
from init.FWK_Degree import IK_fulls_1

T = np.array([
    [1, 0, 0, 300],
    [0, -1, 0, 200],
    [0, 0, -1, 50],
])

Q = IK_fulls_1(T)
print(Q)