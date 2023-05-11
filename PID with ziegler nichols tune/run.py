from main import *
import numpy as np
import matplotlib.pyplot as plt
import control as ct

# Define the system parameters
m = 0.9  # Mass of the load (kg)
l = 0.9  # Length of the arm (m)
b = 0.01  # Damping coefficient (N.m.s/rad)
g = 9.81  # Acceleration due to gravity (m/s^2)

# Create the system model
sys_model = Model(m, l, b,g)

# Define the desired response of the system
t = np.linspace(0, 150, 1000)
theta_d = np.ones(1000) * 10*np.pi / 2
x0 = [0, 0]

# Tune the PID controller using the Ziegler-Nichols method
Kp = 0.6 * (2 * np.pi / 10)
Ki = 1.2 * Kp / (0.5 * 10)
Kd = 0.075 * Kp * 10

# Create the PID controller
pid_controller = PID(Kp, Ki, Kd)

# Simulate the system with PID control
#simulation = Simulation(sys_model.sys, pid_controller.controller, t, theta_d, x0)

# Plot the response of the system with PID control


# Plot the error over time
# Create an instance of the Model class
my_model = Model(m=0.9, l=0.9, b=0.01, g=9.81)

# Create an instance of the PID class and tune the gains
my_pid = PID(Kp,Ki,Kd)
my_pid.tune_ziegler_nichols(Ku=0.6*(2*np.pi/10), Tu=10)

# Create an instance of the Simulation class and run the simulation
#my_sim = Simulation(model=my_model, pid=my_pid, t=70, dt=0.01, theta_d=np.pi/2)
# my_sim = Simulation(model=my_model, pid=my_pid, t=70, theta_d=np.pi/2, err_d=0.01, x0=x0)
# my_sim.run()

# Plot the results
# my_sim.plot_response()
# my_sim.plot_error()

# print(my_sim)
pid = PID(Kp=1, Ki=0, Kd=0)
zeg = pid.tune_ziegler_nichols(Ku=0.6*(2*np.pi/10), Tu=10)
model = Model(m=1, l=1, b=0.1, g=9.81)
# pid = PID(Kp=10, Ki=0.1, Kd=1)
pid = PID(Kp=zeg[0], Ki=zeg[1], Kd=zeg[2])
t = np.linspace(0, 250, 1000)



err_d = np.zeros_like(theta_d)
x0 = np.array([0, 0,0])
sim = Simulation(model, pid, t, theta_d, err_d, x0)
t, y = sim.simulate()
# sim.plot_response(t, y,ax=None)
# sim.plot_error(t, y, ax=None)
print(zeg)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

# plot response on top subplot
ax1.plot(t, theta_d, 'k--', label='Desired response')
ax1.plot(t, y, 'b-', label='PID control')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (rad)')
ax1.set_title('PID Control of a 3-Axis Robotic Arm')
ax1.legend()

# plot error on bottom subplot
ax2.plot(t, err_d, 'k--', label='Target')
ax2.plot(t, theta_d - y, 'r', label='Error')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error')
ax2.set_title('PID Control of a 3-Axis Robotic Arm')
ax2.legend()

plt.show()
