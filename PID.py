from Model import RoboticArm
import numpy as np

class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, target=np.zeros(3), dt=0.01):
        self.kp = kp  # proportional gain
        self.ki = ki  # integral gain
        self.kd = kd  # derivative gain
        self.target = target  # target position
        self.last_error = 0
        self.integral = 0
        self.arm = RoboticArm()  # create a robotic arm object
        self.prev_error = np.zeros(3)  # previous error for derivative term
        self.integral_error = np.zeros(3)  # integral error for integral term
        self.pid = PIDController(kp, ki, kd, dt)  # create a PIDController object
    def control(self, current_position, dt):
    # calculate the error between the desired and current position
        error = self.target - current_position

    # calculate the derivative of the error
        self.previous_error = error
        derivative_error = (error - self.previous_error) / dt


    # calculate the integral of the error
        self.integral_error += error * dt

    # calculate the control output
        control_output = self.kp * error + self.kd * derivative_error + self.ki * self.integral_error

    # return the control output
        return control_output
    def Calculate(self, current_value, target_value, dt):
        error = target_value - current_value
        self.integral_error += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        self.prev_error = error
        return output
    # def control(self, current_position, dt=0.01):
    #     # Compute the error between the current position and the target position
    #     error = self.target - current_position

    #     # Update the PID controller
    #     self.pid.update(error = error, dt = dt)

    #     # Compute the control output
    #     control_output = self.pid.output()

    #     # Compute the desired joint angles
    #     desired_joint_angles = self.arm.inverse_kinematics(self.target)

    #     # Apply the control output to the joint angles
    #     new_joint_angles = desired_joint_angles + control_output

    #     # Return the new joint angles
    #     return new_joint_angles

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = 0
        self.integral_error = 0
        #self.derivative_error = 0

    def update(self, error, dt=0.01):
        #self.error = 0.0
        derivative_error = (error - self.prev_error) / self.dt
        self.integral_error += error * self.dt
        self.prev_error = error
        return self.kp * self.prev_error + self.ki * self.integral_error + self.kd * derivative_error

    def reset(self):
        self.prev_error = 0
        self.integral_error = 0

    def output(self):
        return self.update(error=0)