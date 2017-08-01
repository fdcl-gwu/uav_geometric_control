from __future__ import absolute_import, division, print_function, unicode_literals

import numpy as np
import scipy.linalg

import kinematics.attitude as attitude


class Dumbbell(object):
    r"""Dumbbell object

    Creates a dumbbell model of a rigid spacecraft around an asteroid.
    Method functions allows for simulation in both the body and inertial frames.
    Also included is the capability to control the trajectory of the dumbbell on SE(3)

    Author
    ------
    Shankar Kulumani		GWU		skulumani@gwu.edu

    References
    ----------
    This derivation is based on the following works:

    .. [1] LEE, Taeyoung, LEOK, Melvin y MCCLAMROCH, N Harris. "Lie Group
    Variational Integrators for the Full Body Problem". Computer Methods in
    Applied Mechanics and Engineering. 2007, vol 196, no. 29, p. 2907--2924.

    """

    def __init__(self, m1=100.0, m2=100.0, l=0.003):
        r"""Initalize dumbbell model

        This will initialize the properties of a dumbbell model of a rigid
        spacecraft around an asteroid.

        Parameters
        ----------
        m1 : float
            Mass in kg of first spherical dumbbell mass
        m2 : float
            Mass in kg of second spherical dumbbell mass
        l : float
            length in meters of the distance between the COM of m1 and m2

        Author
        ------
        Shankar Kulumani		GWU		skulumani@gwu.edu

        """

        self.m1 = m1  # kg first mass
        self.m2 = m2  # kg second mass
        self.l = l  # km rigid link
        self.r1 = 0.001  # km radius of each spherical mass
        self.r2 = 0.001

        self.mratio = self.m2/(self.m1+self.m2)
        self.lcg1 = self.mratio*self.l; # distance from m1 to the CG along the b1hat direction
        self.lcg2 = self.l - self.lcg1

        self.zeta1 = np.array([-self.lcg1,0,0])
        self.zeta2 = np.array([self.lcg2,0,0])

        self.Jm1 = 2.0/5*self.m1*self.r1**2 * np.diag([1,1,1])
        self.Jm2 = 2.0/5*self.m2*self.r2**2 * np.diag([1,1,1])

        self.J = self.Jm1 + self.Jm2 + self.m1 *(np.inner(self.zeta1,self.zeta1)*np.eye(3,3) - np.outer(self.zeta1,self.zeta1)) + self.m2 * (np.inner(self.zeta2,self.zeta2)*np.eye(3,3) - np.outer(self.zeta2,self.zeta2))
        self.Jd = self.m1*np.outer(self.zeta1,self.zeta1) + self.m2*np.outer(self.zeta2,self.zeta2) + self.Jm1/2 + self.Jm2/2

        # controller parameters
        OS_translation = 5/100
        Tp_translation = 5
        Ts_translation = 10 

        OS_rotation = 5/100
        Tp_rotation = 5
        Ts_rotation = 10
        
        self.zeta_translation = - np.log(OS_translation) / np.sqrt(np.pi**2 + np.log(OS_translation)**2)
        self.wn_translation = 4.0 / self.zeta_translation / Ts_translation

        self.zeta_rotation = - np.log(OS_rotation) / np.sqrt(np.pi**2 + np.log(OS_rotation)**2)
        self.wn_rotation = 4 / self.zeta_rotation / Ts_rotation

        self.kR = self.wn_rotation**2 
        self.kW = 2 * self.zeta_rotation * self.wn_rotation 
        
        self.kx =  (self.m1 + self.m2) * self.wn_translation**2
        self.kv = (self.m1 + self.m2) * 2 * self.zeta_translation * self.wn_translation



    def attitude_controller(self, time, state, ext_moment):
        r"""Geometric attitude controller on SO(3)

        This function will determine an attitude control input for a rigid spacecraft around an asteroid.
        The function is setup to work for a vehicle defined in the inertial frame relative to an asteroid.

        Parameters
        ----------
        self : dumbbell instance
            Instance of dumbbell class with all of it's parameters
        time : float
            Current time for simulation which is used in the desired attitude trajectory
        state : array_like (18,)
            numpy array defining the state of the dumbbell
            position - position of the center of mass wrt to the inertial frame
            and defined in the inertial frame (3,)
            velocity - velocity of the center of mass wrt to teh inertial frame
            and defined in the inertial frame (3,)
            R_b2i - rotation matrix which transforms vectors from the body
            frame to the inertial frame (9,)
            angular_velocity - angular velocity of the body frame with respect
            to the inertial frame and defined in the body frame (3,)
        ext_moment : array_like (3,)
            External moment in the body fixed frame

        Returns
        -------
        u_m : array_like (3,)
            Body fixed control moment

        Author
        ------
        Shankar Kulumani		GWU		skulumani@gwu.edu

        References
        ----------
        
        .. [1] LEE, Taeyoung, LEOK, Melvin y MCCLAMROCH, N Harris. "Control of
        Complex Maneuvers for a Quadrotor UAV Using Geometric Methods on Se
        (3)". arXiv preprint arXiv:1003.2005. 2010, 

        Examples
        --------

        """ 
        # extract the state
        pos = state[0:3] # location of the center of mass in the inertial frame
        vel = state[3:6] # vel of com in inertial frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
        ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame
        # compute the desired attitude command
        Rd, Rd_dot, ang_vel_d, ang_vel_d_dot = self.desired_attitude(time)
        # determine error between command and current state
        eR = 1/2 * attitude.vee_map(Rd.T.dot(R) - R.T.dot(Rd))
        eW = ang_vel - R.T.dot(Rd).dot(ang_vel_d)
        # compute attitude input
        u_m = (-self.kR*eR - self.kW*eW + np.cross(ang_vel, self.J.dot(ang_vel)) 
                - self.J.dot( attitude.hat_map(ang_vel).dot(R.T).dot(Rd).dot(ang_vel_d)-
                    R.T.dot(Rd).dot(ang_vel_d_dot)) - ext_moment)
        return u_m

    def translation_controller(self, time, state, ext_force):
        """SE(3) Translational Controller

        Inputs:

        Outputs:
            u_f - force command in the dumbbell frame

        """

        # extract the state
        pos = state[0:3] # location of the center of mass in the inertial frame
        vel = state[3:6] # vel of com in inertial frame
        R = np.reshape(state[6:15],(3,3)) # sc body frame to inertial frame
        ang_vel = state[15:18] # angular velocity of sc wrt inertial frame defined in body frame
      
        m = self.m1 + self.m2

        # figure out the desired trajectory
        x_des, xd_des, xdd_des = self.desired_translation(time)

        # compute the error
        ex = pos - x_des
        ev = vel - xd_des
        # compute the control
        u_f = - self.kx * ex - self.kv * ev - ext_force + m * xdd_des

        return u_f

    def desired_attitude(self, time, alpha=2*np.pi/100, axis=np.array([0, 1, 0])):
        """Desired attitude trajectory

        This function will output a desired attitude trajectory. The controller will use this 
        trajectory in it's computations. The outputs will be the desired attitude matrix and
        the desired angular velocity:

        Outputs:
            Rd_sc2int - 3x3 array defining the transformation from the spacecraft frame to the
                inertial frame
            w_sc2int - 3 array defining the angular velocity of the spacecraft frame with respect
                to the inertial frame and defined in the spacecraft fixed frame

        """
        Rd = scipy.linalg.expm(alpha * time * attitude.hat_map(axis) ) 
        Rd_dot = alpha * attitude.hat_map(axis).dot(
                    scipy.linalg.expm(alpha * time * attitude.hat_map(axis)))

        ang_vel_d = attitude.vee_map(Rd.T.dot(Rd_dot))
        ang_vel_d_dot = np.zeros_like(ang_vel_d)

        return (Rd, Rd_dot, ang_vel_d, ang_vel_d_dot) 

    def desired_translation(self, time, alpha=2*np.pi/100):
        """Desired translational trajectory

        This function will output the desired translational states, namely the desired position and
        velocity. This position and velocity will be defined in the inertial reference frame.

        """
        x_des = np.array([1.5, 0.2*np.cos(alpha * time), 0.5*np.sin(alpha * time)])
        xd_des = np.array([0, - alpha * 0.2 * np.sin(alpha * time), alpha * 0.5 * np.cos(alpha * time)])
        xdd_des = np.array([0, - alpha**2 * 0.2 * np.cos(alpha * time), - alpha**2 * 0.5 * np.sin(alpha * time)])

        return (x_des, xd_des, xdd_des)
