import RobotDART as rd
import numpy as np
from numpy import ndarray

class Simulation:
    """ 1) It is creating all the visual for robotdart simulation Except the visual robot. 
        2) It contains the infos for the cube and virtual cube.
        3) It is setting up the limits for VX300-5DOF robot, joint position and velocities 
    """
    def __init__(self,start_q:ndarray,box_pos:ndarray,vbox_pos:ndarray):

        # The boxes positions and the Homogenous Transformations
        self.box_pos = box_pos
        self.vbox_pos = vbox_pos
        self.Tbox = None
        self.Tvbox = None

        # Creating the model and setting the starting position
        self.robot = rd.Vx300()

        # Setting the limitation of the robot
        dof_names = ["waist","shoulder","elbow","wrist_angle","wrist_rotate"]
        # Joint Limits
        self.pos_upper_limits = np.array([180,101,92,130,180])*(np.pi/180)
        self.pos_lower_limits = np.array([-180,-101,-101,-107,-180])*(np.pi/180)
        self.robot.set_position_upper_limits(self.pos_upper_limits, dof_names)
        self.robot.set_position_lower_limits(self.pos_lower_limits, dof_names)
        
        # Joint Velocities Limits
        vel_upper_lims = np.array([0.3, 0.3, 0.3, 0.3, 0.3, 2, 2])
        self.robot.set_velocity_upper_limits(vel_upper_lims, dof_names)
        self.robot.set_velocity_lower_limits(-1*vel_upper_lims, dof_names)

        self.robot.set_positions(start_q)

        self.simu = rd.RobotDARTSimu(0.0005)
        
        # Add robot and floor to the simulation
        self.simu.add_robot(self.robot)
        self.simu.add_checkerboard_floor()
        self.robot.set_draw_axis("vx300/ee_gripper_link")

        # Create the real cube that we are trying to grub
        self.box = self.robot.create_box([0.065,0.065,0.065],self.box_pos,mass = 0.01)
        self.simu.add_robot(self.box)
        self.box.set_draw_axis("box")
        # Homogenous tranformation of the real cube
        self.Tbox = self.box.body_pose("box")


        # Create a visual cube that, The cube position that we want to let the real cube
        self.vbox = self.robot.create_box([0.065,0.065,0.065],self.vbox_pos,mass = 0.01,box_name = "vbox")
        self.simu.add_visual_robot(self.vbox)
        # Homogenous tranformation of the visual cube
        self.Tvbox = self.vbox.body_pose("vbox")

        # Create graphics
        g_config = rd.gui.GraphicsConfiguration()
        
        # This configuration lowers the graphics
        g_config.shadow_map_size = 256
        g_config.shadowed = False
        g_config.transparent_shadows = False
        
        graphics = rd.gui.Graphics(g_config)
        
        self.simu.set_graphics(graphics)
        self.simu.set_collision_detector("fcl")

    def __str__(self):
        return f"Simulation Created with box position {self.box_pos}"
