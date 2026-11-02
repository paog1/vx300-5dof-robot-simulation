import controller as ctrl
import simulation
import numpy as np
import inverse_kinematics
import task_level_trajectory as tr


def real_world_simulation(simu_obj,q_list):
    
    # Initializing pid controller at joint space
    joint_control = ctrl.Joint_Space_Controller(kp=250,ki=25,kd=2*np.sqrt(250),err=0,sum_err=0,pre_err=0,step=simu_obj.simu.timestep())
    # print(joint_control)
    
    for q_pos in q_list:
        # print(q_pos)
        joint_control.pre_err = q_pos[:5] # Passing the previous error on joint controller we only need the dof joint Not teh grippers
        while True:
            # we transform the joint positon to a column vector
            rp = simu_obj.robot.positions() 
            rp_cur = np.array([[rp[0]],[rp[1]],[rp[2]],[rp[3]],[rp[4]],[rp[5]],[rp[6]]]) 
            
            joint_control_out = joint_control.controller() # The out of our controler --> return # [The signal that PID created,The current error, The sum error,The previous error]
            joint_control.err = q_pos[:5] - rp_cur[:5]

            # [The signal that PID created,The current error, The sum error,The previous error]
            joint_control.sum_err = joint_control_out[2] # The sum error that joint controller return
            joint_control.pre_err = joint_control_out[3] # The previous error that joint controller return
            # A column vector 5x1
            joint_control_signal = joint_control_out[0] # The signal that pid joint controller created

            # We are exteding the control signal from 5x1 column vector to a 7x1 column vector with the 6th and 7th element to be the signals for gripper (Signals 2= open,-2 = closed)
            # This need to be to set the commands to VX300-5DOF
            joint_control_signal = np.array([[joint_control_signal[0,0]],[joint_control_signal[1,0]],[joint_control_signal[2,0]],[joint_control_signal[3,0]],[joint_control_signal[4,0]],[q_pos[5][0]],[q_pos[6][0]]])
            simu_obj.robot.set_commands(joint_control_signal)
            
            # We are checking the acceptable joint position error
            success = joint_control.good_enough(qdesired=q_pos[:5],qcurrent=rp_cur[:5])
            if success :
                simu_obj.robot.set_commands(joint_control_signal)
                break

            if simu_obj.simu.step_world():
                break


        simu_obj.robot.set_commands(joint_control_signal)
  

def main():
    # The starting position for our robot before task is assing
    start_q = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[1],[1]])

    # =============================================================
    # Below some examples that has solution in task space
    # even if some joints positions of the trajectory of the Robot
    # cant solved well enough from QP solver (Invese Kinematics) 
    # =============================================================
    # The np.pi/2 in  2nd element of the cubes, it is helping the 
    # controller to grub it from above.
    # =============================================================

    # Example 0
    box_pos = np.array([[0.0],[np.pi/2],[0.0],[0.2],[-0.2],[0.07]])
    vbox_pos = np.array([[0.0],[np.pi/2],[0.0],[0.2],[0.45],[0.07]])

    # Example 1
    # box_pos = np.array([[0.0],[np.pi/2],[0.0],[0.3],[0.0],[0.07]])
    # vbox_pos = np.array([[0.0],[np.pi/2],[0.0],[0.55],[0.0],[0.07]])

    # Example 2
    # box_pos = np.array([[0.0],[np.pi/2],[0.0],[0.2],[0.4],[0.07]])
    # vbox_pos = np.array([[0.0],[np.pi/2],[0.0],[0.45],[-0.1],[0.07]])

    # Example 3
    # box_pos = np.array([[0.0],[np.pi/2],[0.0],[0.3],[0.45],[0.07]])
    # vbox_pos = np.array([[0.0],[np.pi/2],[0.0],[0.45],[0.0],[0.07]])

    # Example 4
    # box_pos = np.array([[0.0],[np.pi/2],[0.0],[0.2],[0.4],[0.07]])
    # vbox_pos = np.array([[0.0],[np.pi/2],[0.0],[0.4],[0.4],[0.07]])

    # Example 5
    # box_pos = np.array([[0.0],[np.pi/2],[0.0],[0.2],[0.4],[0.07]])
    # vbox_pos = np.array([[0.0],[np.pi/2],[0.0],[0.55],[0.0],[0.07]])
    

    # Creating the Simulation for RobotDart Visuals
    simu_obj = simulation.Simulation(start_q,box_pos,vbox_pos)

    # Add visual-only target robot
    vrobot = simu_obj.robot.clone_ghost()
    simu_obj.simu.add_visual_robot(vrobot)  

    # Setting a Offset for end effector to stay lower so it can grub the cube
    offset_for_grip = -0.02
    tbox = simu_obj.Tbox
    tvbox = simu_obj.Tvbox
    
    ll = simu_obj.pos_lower_limits
    ul = simu_obj.pos_upper_limits
    
    # Calculating the trajectory in Task Space
    task_tr = tr.Task_Level_Trajectory(start_point=box_pos[3::],end_point=vbox_pos[3::])
    trajectory = task_tr.calculate_rectagle_trajectory(dz = np.array([[0,0,0.23]])) # This is the trajectory from when we grub the cube to when we let it we actually trajectory on the same level

    # ======================================================================================
    # The tb = Tbox and tvb = Tvbox is the homogenous matrixes of box and vbox respectively,
    # transform them in numpy arrays
    # ======================================================================================
    
    it = 0
    q_list = []
    for pos in trajectory:
        # the if else condition is used to specify when we need to calculate the orientation of the visual cube, the last point and the previous of the last
        if it != len(trajectory) or it != len(trajectory)-1:
            tb = np.array([[tbox.rotation()[0,0],tbox.rotation()[0,1],tbox.rotation()[0,2],pos[0][0]],[tbox.rotation()[1,0],tbox.rotation()[1,1],tbox.rotation()[1,2],pos[1][0]],[tbox.rotation()[2,0],tbox.rotation()[2,1],tbox.rotation()[2,2],pos[2][0]+offset_for_grip],[0,0,0,1]])
            # Calculating the Inveser Kinematics using QP 
            ik = inverse_kinematics.IK_QP(robot=vrobot,Tbox=tb,step=simu_obj.simu.timestep(),qmin=ll,qmax=ul)
        else:
            tvb = np.array([[tvbox.rotation()[0,0],tvbox.rotation()[0,1],tvbox.rotation()[0,2],pos[0][0]],[tvbox.rotation()[1,0],tvbox.rotation()[1,1],tvbox.rotation()[1,2],pos[1][0]],[tvbox.rotation()[2,0],tvbox.rotation()[2,1],tvbox.rotation()[2,2],pos[2][0]+offset_for_grip],[0,0,0,1]])
            ik = inverse_kinematics.IK_QP(robot=vrobot,Tbox=tvb,step=simu_obj.simu.timestep(),qmin=ll,qmax=ul)
        # The loop that solve the QP
        t_q = ik.loop()
        # The if else condition is used to specify when the gripper need to close or to stay open
        # A: The point that cube is starting and we are trying to grub it
        # B: The point that cube need to transfer in task space and we are trying to let it
        # UpA_open -> A_closed -> UpA_closed -> UpB_closed -> B_open -> UpB_open
        if it == 0 or it == 1  or it == len(trajectory)-1 or it == len(trajectory):
            q_temp = np.array([[t_q[0]],[t_q[1]],[t_q[2]],[t_q[3]],[t_q[4]],[2],[0]])
            q_list.append(q_temp)
        else:
            q_temp = np.array([[t_q[0]],[t_q[1]],[t_q[2]],[t_q[3]],[t_q[4]],[-2],[0]])
            q_list.append(q_temp)
        it += 1


    real_world_simulation(simu_obj,q_list)

    return 0

if __name__ =="__main__":
    print(main())