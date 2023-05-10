import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from control.matlab import *

# Define the parameters of the robotic arm
m = 1  # mass of the arm
l = 1  # length of the arm
b = 0.1  # damping coefficient of the arm
g = 9.81  # gravitational acceleration

# Define the PID controller
Kp = 100  # proportional gain
Ki = 50  # integral gain
Kd = 10  # derivative gain
num = [Kd, Kp, Ki]
den = [1, 0.1, 0]
pid = tf(num, den)

# Define the state-space model
A = np.array([[0, 1], [-g/l, -b/(m*l**2)]])
B = np.array([[0], [1/(m*l**2)]])
C = np.array([[1, 0], [0, 1]])
D = np.array([[0], [0]])
sys = ss(A, B, C, D)

# Define the simulation parameters
dt = 0.01  # time step
t = np.arange(0, 10, dt)  # time vector
u = np.zeros(len(t))  # input vector
u[100:] = 1.0  # step input

# Simulate the system
y, t, x = lsim(sys, u, t)
y_sp = np.zeros(len(t))
y_sp[100:] = 1.0
y_pid = np.zeros(len(t))
e = np.zeros(len(t))
x0 = [0, 0]
for i in range(len(t)):
    y_pid[i], _, x0 = lsim(pid, [y_sp[i] - y[i], e[i]], [0, dt], X0=x0)
    e[i] = y_sp[i] - y[i]

# Plot the results
plt.figure()
plt.plot(t, y, 'b', label='Actual Position')
plt.plot(t, y_sp, 'r--', label='Setpoint')
plt.plot(t, y_pid, 'g', label='PID Output')
plt.legend(loc='best')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Setpoint vs. Actual Position')

plt.figure()
plt.plot(t, e, 'r', label='Error')
plt.legend(loc='best')
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.title('Error vs. Time')

plt.show()


