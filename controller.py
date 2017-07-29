from __future__ import absolute_import, division, print_function, unicode_literals

import numpy as np
import scipy.linalg

import kinematics.attitude as attitude

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

