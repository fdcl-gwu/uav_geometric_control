    def attitude_controller(self, time, state, ext_moment):
        """SE(3) Attitude Controller
        
        This function will return the control input to track a desired attitude trajectory


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

