import numpy as np
from numpy import ndarray

class Task_Level_Trajectory:
    """It is dicing the total move to 6 easiest ones"""
    
    def __init__(self,start_point:ndarray,end_point:ndarray):
        self.start_point = start_point # The starting point in task space, we get it from box positio, The real cube that we grubing
        self.end_point = end_point # The ending point in task space, we get it from vbox (=virtual box), The virtual one that we are leaving the cube

    def calculate_rectagle_trajectory(self,dz:ndarray):
        """This function it is calculating the Zaxis offset"""
        self.offset = self.end_point - self.start_point

        A_dz = self.start_point + dz.T

        B_dz = self.end_point + dz.T 

        # A: The point that cube is starting and we are trying to grub it
        # B: The point that cube need to transfer in task space and we are trying to let it
        trajectory = [A_dz,self.start_point,A_dz,B_dz,self.end_point,B_dz] #UpA_open -> A_closed -> UpA_closed -> UpB_closed -> B_open -> UpB_open
        return trajectory
