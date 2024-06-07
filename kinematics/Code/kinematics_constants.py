# Definition of the kinematics constants used in the kinematics calculations

import numpy as np

# Coordinate frame x is width of finger, y is depth of finger, z is height of finger

# Radius of the spools (mm)
R_spoon = 8.0

# Constants for the thumb (finger a)

# Joint 1

# Radius of the wheel defining the joint 1 of finger a (mm)
R_a1 = 20.0

# Joint 2

R_a2 = 10.0 # Radius of the contact surface of the joint 2 of finger a (mm)
v_a2_c1_T1 = np.array([0, 3.2, 10]) # Vector from the center of the radius defining the lower part of the contact joint 2 of finger a to the hole of the tendon (mm)
v_a2_c2_T2 = np.array([0, -3.4, 10]) # Vector from the center of the radius defining the upper part of the contact joint 2 of finger a to the hole of the tendon (mm)

# Joint 3

R_a3 = 10.0 # Radius of the contact surface of the joint 3 of finger a (mm)
v_a3_c1_T1 = np.array([0, 3.9, 10]) # Vector from the center of the radius defining the lower part of the contact joint 3 of finger a to the hole of the tendon (mm)
v_a3_c2_T2 = np.array([0, -3.4, 10]) # Vector from the center of the radius defining the upper part of the contact joint 3 of finger a to the hole of the tendon (mm)

# Constants for the index finger (finger b, c and d)

# Joint 2

R_bcd2 = 10.0 
v_bcd2_c1_T1 = np.array([0, 2.7, 10]) 
v_bcd2_c2_T2 = np.array([0, -1.9, 10.4]) 

# Joint 3 a (coupled joint lower part)

R_bcd3a = 10.0 
v_bcd3a_c1_T1 = np.array([0, 4.2, 10]) 
v_bcd3a_c2_T2 = np.array([0, -3.4, 10]) 

# Joint 3 b (coupled joint upper part)

R_bcd3b = 10.0 
v_bcd3b_c1_T1 = np.array([0, 6.7, 7.4])
v_bcd3b_c2_T2 = np.array([0, -6.7, 7.4]) 

# Extra constants for the pinky finger (finger d)

# Joint 1

R_d1 = 7.5 # Radius of the wheel defining the joint 1 of finger d (mm)