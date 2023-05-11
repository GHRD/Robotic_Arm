import numpy as np
import matplotlib.pyplot as plt
import control as ct

class Model:
    def __init__(self, m, l, b, g):
        self.m = m
        self.l = l
        self.b = b
        self.g = g
        self.A = np.array([[0, 1], [0, -b/(m*l**2)]])
        self.B = np.array([[0], [1/(m*l**2)]])
        self.C = np.array([[1, 0]])
        self.D = np.array([[0]])
        self.sys = ct.StateSpace(self.A, self.B, self.C, self.D)

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.controller = ct.TransferFunction([Kd, Kp, Ki], [1, 0])
    def tune_ziegler_nichols(self, Ku, Tu):
        Kc = 0.6*Ku
        tauI = 0.5*Tu
        tauD = 0.125*Tu
        self.Kp = 0.6*Kc
        self.Ki = Kc/tauI
        self.Kd = Kc*tauD
        self.controller = ct.TransferFunction([self.Kd, self.Kp, self.Ki], [1, 0])
        return self.Kp, self.Ki, self.Kd


class Simulation:
    def __init__(self, model, pid, t, theta_d, err_d, x0):
        self.model = model
        self.pid = pid
        self.t = t
        self.theta_d = theta_d
        self.err_d = err_d
        self.x0 = x0
    
    def simulate(self):
        sys_cl = ct.feedback(self.pid.controller * self.model.sys, 1)
        t, y = ct.forced_response(sys_cl, self.t, self.theta_d, X0=self.x0)
        return t, y
    
    def plot_response(self, t, y, ax=None):
        if ax is None:
            ax = plt.gca()
        ax.plot(t, self.theta_d, 'k--', label='Desired response')
        ax.plot(t, y, 'b-', label='PID control')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (rad)')
        ax.set_title('PID Control of a 3-Axis Robotic Arm')
        ax.legend()

    def plot_error(self, t, y, ax=None):
        if ax is None:
            ax = plt.gca()
        ax.plot(t, self.err_d, 'k--', label='Target')
        ax.plot(t, self.theta_d - y,'r', label='Error')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error')
        ax.set_title('PID Control of a 3-Axis Robotic Arm')
        ax.legend()
