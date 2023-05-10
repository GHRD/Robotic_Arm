import numpy as np
import matplotlib.pyplot as plt
import control as ct

# Define the system parameters
m = 0.9 # Mass of the load (kg)
l = 0.9 # Length of the arm (m)
b = 0.01 # Damping coefficient (N.m.s/rad)
g = 9.81 # Acceleration due to gravity (m/s^2)

# Define the state-space model of the system
A = np.array([[0, 1], [0, -b/(m*l**2)]])
B = np.array([[0], [1/(m*l**2)]])
C = np.array([[1, 0]])
D = np.array([[0]])
sys = ct.StateSpace(A, B, C, D)

# Define the desired response of the system
t = np.linspace(0, 70, 1000)
theta_d = np.ones(1000) * np.pi/2
err_d = np.ones(1000) * 0

# Tune the PID controller using the Ziegler-Nichols method
Kp = 0.6 * (2 * np.pi / 10)
Ki = 1.2 * Kp / (0.5 * 10)
Kd = 0.075 * Kp * 10
G = ct.tf([1], [1, 1])
C = Kp + Ki/ct.tf([1, 0], [1]) + Kd*ct.tf([1], [1])

# Convert transfer function to state-space
sys_cl = ct.feedback(C*G, 1)

# Define the PID controller
controller = ct.TransferFunction([Kd, Kp, Ki], [1, 0])

# Compute the closed-loop system with PID control
sys_cl = ct.feedback(controller * sys, 1)

# Simulate the closed-loop system with PID control
x0 = [0, 0,0]
t, y = ct.forced_response(sys_cl, t, theta_d, X0=x0)

# Plot the response of the system with PID control
plt.plot(t, theta_d, 'k--', label='Desired response')
plt.plot(t, y, 'b-', label='PID control')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('PID Control of a 3-Axis Robotic Arm')
plt.legend()
plt.show()

# Plot the error over time
plt.plot(t, err_d, 'k--', label='target')
plt.plot(t, theta_d - y,'r', label='Desired response')
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.title('PID Control of a 3-Axis Robotic Arm')
plt.show()
