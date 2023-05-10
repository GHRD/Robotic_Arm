import numpy as np
import matplotlib.pyplot as plt
from Model import RoboticArm
from PID import PID
from PID import PIDController
# Define the target position
target_pos = np.array([1, 1, 1])

# Initialize the robotic arm and the PID controller
arm = RoboticArm()
pid = PID()
#pidcontroller = PIDController()

# Set the simulation time and time step
t_end = 10
dt = 1
t = np.arange(0, t_end, dt)

# Initialize arrays for plotting data
setpoint = np.zeros((len(t), 3))
actual_pos = np.zeros((len(t), 3))
error = np.zeros((len(t), 3))
print("------------------------------------------------")
# Run the simulation
for i in range(len(t)):
    # Calculate the PID control output
    control_output = pid.Calculate(arm.forward_kinematics(arm.joint_angles)[0], target_pos, dt)
    print('-----------step:', i)
    print(control_output)
    # Update the joint angles using the control output
    upd = arm.update_joint_angles(control_output, dt)
    print(upd)
    # Store data for plotting
    setpoint[i] = target_pos
    actual_pos[i] = arm.forward_kinematics(arm.joint_angles)[0]
    error[i] = pid.prev_error
    

# Plot Setpoint vs. actual position
plt.figure()
plt.plot(t, setpoint[:, 0], 'b--', label='Setpoint')
plt.plot(t, actual_pos[:, 0], 'r-', label='Actual Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('X-axis position')
plt.legend()

plt.figure()
plt.plot(t, setpoint[:, 1], 'b--', label='Setpoint')
plt.plot(t, actual_pos[:, 1], 'r-', label='Actual Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Y-axis position')
plt.legend()

plt.figure()
plt.plot(t, setpoint[:, 2], 'b--', label='Setpoint')
plt.plot(t, actual_pos[:, 2], 'r-', label='Actual Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Z-axis position')
plt.legend()

# Plot Error vs. time
plt.figure()
plt.plot(t, error[:, 0], 'r-', label='X-axis Error')
plt.plot(t, error[:, 1], 'g-', label='Y-axis Error')
plt.plot(t, error[:, 2], 'b-', label='Z-axis Error')
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.title('Error vs. time')
plt.legend()

# Show the plots
plt.show()
