import numpy as np
import sympy as sp


# First coordinate frame (F_I) at the center of the palm, with z to the top of the hand
# y in the same direction as the arm (coming from the arm) and x pointing towards us

# h represents high and l represents low, high is the frame after the joint and low the fram before the joint

# q is the vector of the joint angles
q= [0,0,0,0,0,0,0,0,0,0,0]
q[0]= 0  #To replace with the value of the joint angle by the motor


# Transformation from the frame I to the first frame of the thumb
C_I_a0 = np.rotz(q(0))
r_I_a0 = np.array([0, 0, 0])
T_I_a0 = np.eye(4)
T_I_a0[:3, :3] = C_I_a0
T_I_a0[:3, 3] = r_I_a0


# Transformation from the first frame of the thumb (a0) to the second frame of the thumb (a1)
C_a0_a1 = np.eye(3)
r_a0_a1 = np.array([0, 0, 0])
T_a0_a1 = np.eye(4)
T_a0_a1[:3, :3] = C_a0_a1
T_a0_a1[:3, 3] = r_a0_a1

# Transformation from the second frame of the thumb (a1) to the third low frame of the thumb (a2l)
C_a1_a2l = np.roty(q(1))
r_a1_a2l = np.array([0, 0, 0]) # translation z axis
T_a1_a2l = np.eye(4)
T_a1_a2l[:3, :3] = C_a1_a2l
T_a1_a2l[:3, 3] = r_a1_a2l

# We want to express the angle theta in function of the length l 
r_a2l_r1 = np.array([0, 0, 0]) # translation from a2l to the hole
r_a2h_r2 = np.array([0, 0, 0]) # translation from a2h to the hole
R = np.array([0, 0, 2]) # distance between the two frames along z axis
theta = sp.symbols('theta')
C_1_0 = np.array([[1, 0, 0], [0, sp.cos(theta/2), -sp.sin(theta/2)], [0, sp.sin(theta/2), sp.cos(theta/2)]])
C_1_2 = np.array([[1, 0, 0], [0, sp.cos(theta), -sp.sin(theta)], [0, sp.sin(theta), sp.cos(theta)]])
l = 1 # Here we write the desired value of the length
eq = sp.Eq(np.linalg.norm(r_a2l_r1+C_1_0*R + C_1_2*r_a2h_r2), l)
sol = sp.solve(eq, theta)
q[2] =  sol



# Transformation from the third frame of the thumb (a2l) to the third high frame of the thumb (a2h)
C_a2l_a2h = np.roty(q(2))
r_a2l_a2h = np.array([0, 0, 0]) # translation z axis
T_a2l_a2h = np.eye(4)
T_a2l_a2h[:3, :3] = C_a2l_a2h
T_a2l_a2h[:3, 3] = r_a2l_a2h





# Transformation from the frame I to the first frame of the first normal finger
alpha1 = 0 # rotation angle around z axis between the frame I and the frame b2l
C_I_b2l = np.rotz(alpha1)
r_I_b2l = np.array([0, 0, 0]) #Translation to the b2l frame
T_I_b2l = np.eye(4)
T_I_b2l[:3, :3] = C_I_b2l
T_I_b2l[:3, 3] = r_I_b2l


# We want to express the angle theta in function of the length l 
r_b2l_r1 = np.array([0, 0, 0]) # translation from b2l to the hole
r_b2h_r2 = np.array([0, 0, 0]) # translation from b2h to the hole
R = np.array([0, 0, 2]) # distance between the two frames along z axis
theta = sp.symbols('theta')
C_1_0 = np.array([[1, 0, 0], [0, sp.cos(theta/2), -sp.sin(theta/2)], [0, sp.sin(theta/2), sp.cos(theta/2)]])
C_1_2 = np.array([[1, 0, 0], [0, sp.cos(theta), -sp.sin(theta)], [0, sp.sin(theta), sp.cos(theta)]])
l = 1 # Here we write the desired value of the length
eq = sp.Eq(np.linalg.norm(r_b2l_r1+C_1_0*R + C_1_2*r_b2h_r2), l)
sol = sp.solve(eq, theta)
q[4] =  sol



# Transformation from the low frame of the first joint of the second normal finger (b2l) to the high frame of the first joint (b2h)
C_b2l_b2h = np.roty(q(4))
r_b2l_b2h = np.array([0, 0, 0]) # translation z axis
T_b2l_b2h = np.eye(4)
T_b2l_b2h[:3, :3] = C_b2l_b2h
T_b2l_b2h[:3, 3] = r_b2l_b2h






# Transformation from the frame I to the first frame of the second normal finger
alpha2 = 0 # rotation angle around z axis between the frame I and the frame c2l
C_I_c2l = np.rotz(alpha2)
r_I_c2l = np.array([0, 0, 0]) #Translation to the c2l frame
T_I_c2l = np.eye(4)
T_I_c2l[:3, :3] = C_I_c2l
T_I_c2l[:3, 3] = r_I_c2l


# We want to express the angle theta in function of the length l 
r_c2l_r1 = np.array([0, 0, 0]) # translation from c2l to the hole
r_c2h_r2 = np.array([0, 0, 0]) # translation from c2h to the hole
R = np.array([0, 0, 2]) # distance between the two frames along z axis
theta = sp.symbols('theta')
C_1_0 = np.array([[1, 0, 0], [0, sp.cos(theta/2), -sp.sin(theta/2)], [0, sp.sin(theta/2), sp.cos(theta/2)]])
C_1_2 = np.array([[1, 0, 0], [0, sp.cos(theta), -sp.sin(theta)], [0, sp.sin(theta), sp.cos(theta)]])
l = 1 # Here we write the desired value of the length
eq = sp.Eq(np.linalg.norm(r_c2l_r1+C_1_0*R + C_1_2*r_c2h_r2), l)
sol = sp.solve(eq, theta)
q[6] =  sol



# Transformation from the low frame of the first joint of the second normal finger (c2l) to the high frame of the first joint (c2h)
C_c2l_c2h = np.roty(q(8))
r_c2l_c2h = np.array([0, 0, 0]) # translation z axis
T_c2l_c2h = np.eye(4)
T_c2l_c2h[:3, :3] = C_c2l_c2h
T_c2l_c2h[:3, 3] = r_c2l_c2h






# Transformation from the frame I to the first frame of the pinky finger
alpha3 = 0 # rotation angle around z axis between the frame I and the frame d1l
C_I_d1l = np.rotz(alpha3)
r_I_d1l = np.array([0, 0, 0]) #Translation to the c2l frame
T_I_d1l = np.eye(4)
T_I_d1l[:3, :3] = C_I_d1l
T_I_d1l[:3, 3] = r_I_d1l

# Translation from the frame d1l to d1h (after the joint effect)
C_d1l_d1h = np.roty(q(8))
r_d1l_d1h = np.array([0, 0, 0]) # translation
T_d1l_d1h = np.eye(4)
T_d1l_d1h[:3, :3] = C_d1l_d1h
T_d1l_d1h[:3, 3] = r_d1l_d1h

C_d1h_d2l = np.eye(3)
r_d1h_d2l = np.array([0, 0, 0]) #Translation to the c2l frame
T_d1h_d2l = np.eye(4)
T_d1h_d2l[:3, :3] = C_d1h_d2l
T_d1h_d2l[:3, 3] = r_d1h_d2l


# We want to express the angle theta in function of the length l 
r_d2l_r1 = np.array([0, 0, 0]) # translation from c2l to the hole
r_d2h_r2 = np.array([0, 0, 0]) # translation from c2h to the hole
R = np.array([0, 0, 2]) # distance between the two frames along z axis
theta = sp.symbols('theta')
C_1_0 = np.array([[1, 0, 0], [0, sp.cos(theta/2), -sp.sin(theta/2)], [0, sp.sin(theta/2), sp.cos(theta/2)]])
C_1_2 = np.array([[1, 0, 0], [0, sp.cos(theta), -sp.sin(theta)], [0, sp.sin(theta), sp.cos(theta)]])
l = 1 # Here we write the desired value of the length
eq = sp.Eq(np.linalg.norm(r_d2l_r1+C_1_0*R + C_1_2*r_d2h_r2), l)
sol = sp.solve(eq, theta)
q[9] =  sol



# Transformation from the low frame of the first joint of the second normal finger (c2l) to the high frame of the first joint (c2h)
C_d2l_d2h = np.roty(q(9))
r_d2l_d2h = np.array([0, 0, 0]) # translation z axis
T_d2l_d2h = np.eye(4)
T_d2l_d2h[:3, :3] = C_d2l_d2h
T_d2l_d2h[:3, 3] = r_d2l_d2h
