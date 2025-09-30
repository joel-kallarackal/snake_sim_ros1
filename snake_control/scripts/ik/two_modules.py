import numpy as np

def fk_orientation(theta1, theta2):
    s1, c1 = np.sin(theta1), np.cos(theta1)
    s2, c2 = np.sin(theta2), np.cos(theta2)
    R = np.array([[c1*c2, -s1*c2, s2],
                  [c1*s2, -s1*s2, -c2],
                  [s1, c1, 0]])
    
    return rotation_matrix_to_euler(R)
    

def rotation_matrix_to_euler(R):
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
 
    if not singular:
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

theta1 = np.linspace(-np.pi/2, np.pi/2, 20)
theta2 = np.linspace(-np.pi/2, np.pi/2, 20)

for t1 in theta1:
    for t2 in theta2:
        print(f"Theta 1 (head): {t1}, Theta 2: {t2} ", fk_orientation(t1, t2))
        
        
## IK

def ik(r, p, y):
    theta1, theta2 = 0,0
    if p != np.pi/2 and p != -np.pi/2:
        theta1 = -p
        theta2 = y
    else:
        theta1 = -p
        if theta1 == np.pi/2:
            theta2 = 90 - r
        else:
            theta2 = -(90 - r)
        
    return theta1, theta2