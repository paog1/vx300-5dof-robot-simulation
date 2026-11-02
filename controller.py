import numpy as np 
from numpy import ndarray
class PID_Controller:
    """Is a PID Controller"""

    def __init__(self,kp,ki,kd):
        """Initializing the gains of the PID Controller"""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def pid(self,err,sum_err,pre_err,step)->list[ndarray]:
        """It Calculating the signal that PID created"""
        control_signal = self.kp*err + self.ki* sum_err + self.kd * (err - pre_err)/step
        sum_err += err*step
        pre_err = np.copy(err)
        return [control_signal,err,sum_err,pre_err] # [The signal that PID created,The current error, The sum error,The previous error]

    def __str__(self):
        return f"PID Controller kp = {self.kp} ki = {self.ki} kd = {self.kd}"
    

class Joint_Space_Controller(PID_Controller):
    """ A child Class that it used to create a joint space controller for VX300-5dof robot
        It actually has the end condition for a task """

    def __init__(self,kp,ki,kd,err,sum_err,pre_err,step):
        PID_Controller.__init__(self,kp,ki,kd)
        self.err = err
        self.sum_err = sum_err
        self.pre_err = pre_err
        self.step = step

    def controller(self):
        return self.pid(self.err,self.sum_err,self.pre_err,self.step)
    
    def good_enough(self,qdesired,qcurrent)->bool:
        """Calculating the acceptable error --> return True If acceptable"""
        # print(qdesired-qcurrent)
        if (np.abs(qdesired-qcurrent) <= 5e-2).all():
            return True
        return False
    
    def __str__(self):
        return f"Joint_Space_Controller kp = {self.kp},ki = {self.ki},kd = {self.kd}\nerr = {self.err},sum_err = {self.sum_err},pre_err = {self.pre_err},step = {self.step}"

    
