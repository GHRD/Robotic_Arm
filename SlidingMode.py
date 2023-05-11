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
t = np.linspace(0, 1000, 1000)
theta_d = np.ones(1000) * np.pi/2
err_d = np.ones(1000) * 0

# Define sliding mode control gains
k1 = 110
k2 = 50

# Define sliding mode control function
def sliding_mode_control(x, x_d, err, err_d):
    s = x[0] - x_d
    s_dot = x[1] - err
    u = -k1*np.sign(s) - k2*np.sign(s_dot) + m*l*g*np.cos(x[0])
    return u

# Simulate the closed-loop system with sliding mode control
x0 = [0, 0]
x = np.zeros((len(t), 2))
x[0, :] = x0
y = np.zeros(len(t))
for i in range(len(t) - 1):
    err = theta_d[i] - x[i, 0]
    err_d[i+1] = err
    u = sliding_mode_control(x[i, :], theta_d[i], err, err_d[i])
    u = np.clip(u, -10, 10)
    x_dot = A.dot(x[i, :]) + B.dot(u)
    x_dot_flat = x_dot.reshape((-1,))
    x[i+1, :] = x[i, :] + x_dot_flat[3]*np.array([t[i+1] - t[i], t[i+1] - t[i]])
    #x[i+1, :] = x[i, :] + x_dot[1] * np.array([t[i+1] - t[i]]).T
    y[i+1] = C.dot(x[i+1, :])

# Add the last value of x and y
err = theta_d[-1] - x[-1, 0]
err_d[-1] = err
y[-1] = C.dot(x[-1, :])



# Plot the response of the system with sliding mode control
plt.plot(t, theta_d, 'k--', label='Desired response')
plt.plot(t, y, 'b-', label='Sliding mode control')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Sliding Mode Control of a 3-Axis Robotic Arm')
plt.legend()
plt.show()

# Plot the error over time
plt.plot(t, err_d, 'k--', label='target')
plt.plot(t, theta_d - y,'r', label='Desired response')
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.title('Sliding Mode Control of a 3-Axis Robotic Arm')
plt.show()