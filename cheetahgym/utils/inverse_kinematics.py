import numpy as np
import math
class Cheetah_Kinematics(object):
    def __init__(self):
        pass
    
    
    def inverse2D(self, x,y,br):

        l1 = 0.21
        l2 = 0.18
        sol_branch = br
        t1 = (-4*l2*y + np.sqrt(16*l2**2*y**2 - 4*(-l1**2 + l2**2 - 2*l2*x + x**2 + y**2)*(-l1**2 + l2**2 + 2*l2*x + x**2 + y**2)))/(2.*(l1**2 - l2**2 - 2*l2*x - x**2 - y**2))
        t2 = (-4*l2*y - np.sqrt(16*l2**2*y**2 - 4*(-l1**2 + l2**2 - 2*l2*x + x**2 + y**2)*(-l1**2 + l2**2 + 2*l2*x + x**2 + y**2)))/(2.*(l1**2 - l2**2 - 2*l2*x - x**2 - y**2))

        if(sol_branch):
            t = t2
        else:
            t = t1
        th12 = np.arctan2(2*t,(1-t**2))
        th1 = np.arctan2(y - l2*np.sin(th12), x - l2*np.cos(th12))
        th2 = th12 - th1
        return [th1,th2]
        #return [theta_1,theta_2]

    def inverseKinematics(self, x,y,z,br):
        # x = 0
        # y = -0.28
        # z = 0.0
        '''
        inverse kinematics  function
        Args:
            x : end effector position on X-axis in leg frame
            y : end effector position on Y-axis in leg frame
            z : end effector position on Z-axis in leg frame
        Ret:
            [motor_knee, motor_hip, motor_abduction] :  a list of hip, knee, and abduction motor angles to reach a (x, y, z) position
        '''
        theta = np.arctan2(z,-y)
        new_coords = np.array([x,y/np.cos(theta),z])
        motor_hip, motor_knee = self.inverse2D(new_coords[0], new_coords[1], br)
        return theta, motor_hip, motor_knee

    def forwardKinematics(self, q):
        '''
		Forward kinematics of the
		Args:
		-- q : Active joint angles, i.e., [theta1, theta4], angles of the links 1 and 4 (the driven links)
		Return:
		-- valid : Specifies if the result is valid
		-- x : End-effector position
		'''
        l1 = 0.21
        l2 = 0.18
        x = l1 * math.cos(q[0]) + l2 * math.cos(q[0]+q[1])
        y = l1 * math.sin(q[0]) + l2 * math.sin(q[0]+q[1])
        return [x,y]
