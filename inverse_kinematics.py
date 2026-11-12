import numpy as np
from numpy import ndarray
import proxsuite
import pinocchio as pin

class FK:
    """It is calculating the forward kinematics of the VX300-5DOF Robot using the robotdart physics"""
    def __init__(self,robot):
        self.robot = robot

    @property
    def cal(self)->ndarray:
        """Calculate the End Effector Homogenous matrix --> return ndarray 4x4 """
        ee_tf = self.robot.body_pose("vx300/ee_gripper_link")
        # We translated the matrix to numpy cause the homogenous is a dart3.Isometry object
        return np.array([[ee_tf.rotation()[0,0],ee_tf.rotation()[0,1],ee_tf.rotation()[0,2],ee_tf.translation()[0]],
                        [ee_tf.rotation()[1,0],ee_tf.rotation()[1,1],ee_tf.rotation()[1,2],ee_tf.translation()[1]],
                        [ee_tf.rotation()[2,0],ee_tf.rotation()[2,1],ee_tf.rotation()[2,2],ee_tf.translation()[2]],
                        [0,0,0,1]])
    

class IK_QP:
    """It is calculating the Invese Kinematics problem using QP programming"""
    def __init__(self,robot,Tbox,step,qmin,qmax):
        self.robot = robot
        self.T_wd = Tbox # The desired Homogenous matrix (rotation and translation), It is actually the Homogenous of our cube
        self.step = step
        self.q_min = qmin
        self.q_max = qmax 
        self.qp_dim = 5
        self.qp_dim_eq = 0
        self.qp_dim_in = 5
        self.fk = FK(self.robot)


    def loop(self):
        """The Calculation of the QP"""

        #Initializzig the QP problem
        success = False
        qp = proxsuite.proxqp.dense.QP(self.qp_dim,self.qp_dim_eq,self.qp_dim_in)
        qk = np.zeros(self.qp_dim)
        error = np.ones((6,1))

        for _ in range(100000):
            
            # Twb = current End Effector configuration at space
            T_wb = np.copy(self.fk.cal)
            T_wb_pinv = np.copy(np.linalg.inv(T_wb))

            # The translation and rotation error from pin.log6()
            err = pin.log6(self.T_wd@T_wb_pinv)

            # WE tranform the error to a numpy vector
            error = np.array([[err.angular[0]],[err.angular[1]],[err.angular[2]],[err.linear[0]],[err.linear[1]],[err.linear[2]]])

            # the acceptable Error
            if (np.abs(error) <= 1e-3).all():
                success = True
                break

            # The break condition for a unsolve problem
            if _ > 100000:
                success = False
                break


            J = self.robot.jacobian("vx300/ee_gripper_link")[::,:5]

            #QP Parameters
            Q = J.T@J
            q = (-J.T@error).ravel()  # proxsuite needs a 1-D vector, not (5,1)
            C = np.eye(self.qp_dim)*self.step
            d_min = self.q_min - qk
            d_max = self.q_max - qk
            
            if _ == 0:
                qp.init(Q,q,None,None,C,d_min,d_max)
            else:
                qp.update(Q,q,None,None,C,d_min,d_max)
            qp.solve()
            v = np.copy(qp.results.x)
            qk = qk+v*self.step
            t_q = np.array([[qk[0]],[qk[1]],[qk[2]],[qk[3]],[qk[4]],[1],[1]])
            self.robot.set_positions(t_q)
        if success:
            print("Found solution in %d iterations with error: %s" % (_, error.T))
        else:
            print("Could not find solution in %d iterations! Error: %s" % (_, error.T))
        print(qk.T)
        # exit(1)
        return qk
