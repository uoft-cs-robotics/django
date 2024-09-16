
from scipy.spatial.transform import Rotation as R

import numpy as np

def convert_to_euler(rotmat):
    # rotmat = np.array([[0.03545199412325373, -0.999034838148242, 0.025933535794425314, 0.04983074701882735],
    # [0.9992235434847433,  0.034988476930774004, -0.01811398983818928, -0.0355754228846467],[0.017189131987337677, 0.02655557659288625, 0.9994995423177248, -0.045860552332102456], 
    # [0.0, 0.0,0.0,1.0]])
    r = R.from_matrix(rotmat[:3,:3])

    print("trans", rotmat[:3,3])
    print("rot", r.as_euler('zyx')) 