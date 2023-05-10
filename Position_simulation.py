import numpy as np
import matplotlib.pyplot as plt
from ModelPos import RoboticArm
from PID import PID

# create a robotic arm object
arm = RoboticArm(joint_angles=np.array([0, 0, 0, 0, 0, 0]))
# Set up the PID controller
# kp = 1.0
# ki = 0.0
# kd = 0.1
# dt=0.01
# create a PID controller object
pid = PID(kp=1.0, ki=0.0, kd=0.0, target=np.array([1, 1, 1]))

# simulation parameters
dt = 0.01
t = np.arange(0, 5, dt)
num_steps = len(t)

# initialize variables
pos = np.zeros((num_steps, 3))
pos[0, :] = arm.forward_kinematics()

# simulate the system
for i in range(1, num_steps):
    # calculate control output
    current_position = arm.forward_kinematics()
    new_joint_angles = pid.control(current_position, dt=dt) # add kp=0.1 

    # update joint angles
    arm.joint_angles = new_joint_angles

    # calculate new position
    pos[i, :] = arm.forward_kinematics()

# plot the results
fig, ax = plt.subplots()
ax.plot(pos[:, 0], pos[:, 1], 'b.-')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Robotic arm simulation')
plt.show()
