#!/usr/bin/env python
from __future__ import print_function, division, absolute_import
import numpy as np
from scipy.integrate import odeint
from scipy.integrate import ode
import numpy.linalg as la
import pdb
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import sys
# import seaborn as sns
from .. import ukf_uav


class UAV(object):

    def __init__(self, J, e3):
        self.m = 4.34
        self.g = 9.81
        self.J = J
        self.e3 = e3
        self.kR = 8.81 # attitude gains
        self.kW = 2.54 # attitude gains
        self.kx = 16.*self.m # position gains
        self.kv = 5.6*self.m # position gains
        self.xd = None
        self.xd_dot = None
        self.command = None
        print('UAV: initialized')
        # self.ukf = ukf_uav.UnscentedKalmanFilter(12,6,0.01)

    def dydt(self, t, X):
        R = np.reshape(X[6:15],(3,3));  # rotation from body to inertial
        W = X[15:];   # angular rate
        x = X[:3];  # position
        v = X[3:6];    # velocity

        xd = np.array([0, 0, 0])
        xd_dot = np.array([0, 0, 0])
        xd_ddot = np.array([0, 0, 0])
        xd_dddot = np.array([0, 0, 0])
        xd_ddddot = np.array([0, 0, 0])
        b1d = np.array([1., 0., 0.])
        b1d_dot=np.array([0., 0., 0.])
        b1d_ddot=np.array([0., 0., 0.])
        Rd = np.eye(3)
        Wd = np.array([0.,0.,0.])
        Wd_dot = np.array([0.,0.,0.])
        f = np.array([0,0,0])
        M = np.array([0,0,0])

        if t < 4:
            xd_dot = np.array([1.+ 0.5*t, 0.2*np.sin(2*np.pi*t), -0.1])
            b1d = np.array([1., 0.,0.])
            d_in = (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
                    b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot)
            (f, M) = self.velocity_control(t, R, W, x, v, d_in)
        elif t < 6:
            xd = np.array([8.,0.,0.])
            ang_d=2.*np.pi*(t-4)
            ang_d_dot=2.*np.pi
            Rd = np.array([[np.cos(ang_d), 0., np.sin(ang_d)],[0.,1.,0.],
                [-np.sin(ang_d), 0., np.cos(ang_d)]])
            Rd_dot = np.array([[-ang_d_dot*np.sin(ang_d), 0.,
                ang_d_dot*np.cos(ang_d)],[0.,0.,0.],
                [-ang_d_dot*np.cos(ang_d), 0., -ang_d_dot*np.sin(ang_d)]])
            Wdhat=Rd.T.dot(Rd_dot)
            Wd=np.array([-Wdhat[1,2],Wdhat[0,2],-Wdhat[0,1]])
            b1d = Rd[:,0]
            b1d_dot = Rd_dot[:,0]
            d_in = (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
                    b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot)
            (f, M) = self.attitude_control(t, R, W, x, v, d_in)
        elif t < 8:
            xd = np.array([14. - t, 0, 0])
            xd_dot = np.array([-1 , 0, 0])
            b1d = np.array([1., 0,0])
            d_in = (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
                    b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot)
            (f, M) = self.position_control(t, R, W, x, v, d_in)
        elif t < 9:
            xd = np.array([6.,0.,0.])
            ang_d=2.*np.pi*(t-8)
            ang_d_dot=2.*np.pi
            Rd = np.array([[1.,0.,0.],[0, np.cos(ang_d), -np.sin(ang_d)],
                [0,np.sin(ang_d), np.cos(ang_d)]])
            Rd_dot = np.array([[0,0,0],[0,-ang_d_dot*np.sin(ang_d),
                -ang_d_dot*np.cos(ang_d)],
                [0,ang_d_dot*np.cos(ang_d), -ang_d_dot*np.sin(ang_d)]])
            Wdhat=Rd.T.dot(Rd_dot)
            Wd=np.array([-Wdhat[1,2],Wdhat[0,2],-Wdhat[0,1]])
            b1d = Rd[:,0]
            b1d_dot = Rd_dot[:,0]
            d_in = (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
                    b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot)
            (f, M) = self.attitude_control(t, R, W, x, v, d_in)
        elif t < 12:
            xd = np.array([20. - 5./3*t, 0, 0])
            xd_dot = np.array([-5./3 , 0, 0])
            b1d = np.array([0., 1.,0.])
            d_in = (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot,
                    b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot)
            (f, M) = self.position_control(t, R, W, x, v, d_in)

        R_dot = np.dot(R,hat(W))
        W_dot = np.dot(la.inv(self.J), M - np.cross(W, np.dot(self.J, W)))
        x_dot = v
        v_dot = self.g*self.e3 - f*R.dot(self.e3)/self.m
        X_dot = np.concatenate((x_dot, v_dot, R_dot.flatten(), W_dot))
        self.xd = xd
        self.xd_dot = xd_dot
        self.command = np.insert(M,0,f)
        return X_dot

    def position_control(self, t, R, W, x, v, d_in):
        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot, b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot) = d_in
        (ex, ev) = position_errors( x, xd, v, xd_dot)

        f = np.dot(self.kx*ex + self.kv*ev + self.m*self.g*self.e3 - self.m*xd_2dot, R.dot(self.e3) )
        W_hat = hat(W)
        R_dot = R.dot(W_hat)
        x_2dot = self.g*self.e3 - f*R.dot(self.e3)/self.m
        ex_2dot = x_2dot - xd_2dot

        f_dot = ( self.kx*ev + self.kv*ex_2dot - self.m*xd_3dot).dot(R.dot(self.e3)) + ( self.kx*ex + self.kv*ev + self.m*self.g*self.e3 - self.m*xd_3dot).dot(np.dot(R_dot,self.e3))

        x_3dot = -1/self.m*( f_dot*R + f*R_dot ).dot(self.e3)
        ex_3dot = x_3dot - xd_3dot

        A = -self.kx*ex - self.kv*ev - self.m*self.g*self.e3 + self.m*xd_2dot
        A_dot = -self.kx*ev - self.kv*ex_2dot + self.m*xd_3dot
        A_2dot = -self.kx*ex_2dot - self.kv*ex_3dot + self.m*xd_4dot

        (Rd, Wd, Wd_dot) = get_Rc(A, A_dot, A_2dot , b1d, b1d_dot, b1d_ddot)

        (eR, eW) = attitude_errors( R, Rd, W, Wd )
        M= -self.kR*eR - self.kW*eW + np.cross(W, self.J.dot(W)) - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd))) - R.T.dot(Rd.dot(Wd_dot)))
        return (f, M)

    def velocity_control(self, t, R, W, x, v, d_in):
        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot, b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot) = d_in
        (ex, ev) = position_errors( x, xd, v, xd_dot)

        f = (self.kx*ev + self.m*self.g*self.e3 - self.m*xd_2dot).dot(R.dot(self.e3))
        W_hat = hat(W)
        R_dot = R.dot(W_hat)
        x_2dot = self.g*self.e3 - f*R.dot(self.e3)/self.m
        ex_2dot = x_2dot - xd_2dot

        f_dot = ( self.kx*ex_2dot - self.m*xd_3dot).dot(R.dot(self.e3)) + ( self.kx*ev + self.m*self.g*self.e3 - self.m*xd_3dot).dot(np.dot(R_dot,self.e3))

        x_3dot = -1/self.m*( f_dot*R + f*R_dot ).dot(self.e3)
        ex_3dot = x_3dot - xd_3dot

        A = - self.kv*ev - self.m*self.g*e3 + self.m*xd_2dot
        A_dot = - self.kv*ex_2dot + self.m*xd_3dot
        A_2dot = - self.kv*ex_3dot + self.m*xd_4dot

        (Rd, Wd, Wd_dot) = get_Rc(A, A_dot, A_2dot , b1d, b1d_dot, b1d_ddot)
        (eR, eW) = attitude_errors( R, Rd, W, Wd )
        M= -self.kR*eR - self.kW*eW + np.cross(W, self.J.dot(W)) - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd))) - R.T.dot(Rd.dot(Wd_dot)))
        return (f, M)

    def attitude_control(self, t, R, W, x, v, d_in):
        (xd, xd_dot, xd_ddot, xd_dddot, xd_ddddot, b1d, b1d_dot, b1d_ddot, Rd, Wd, Wd_dot) = d_in
        (ex, ev) = position_errors( x, xd, v, xd_dot)
        f = (self.kx*ex + self.kv*v + self.m*self.g*self.e3).dot(R.dot(self.e3))
        W_hat = hat(W)
        (eR, eW) = attitude_errors( R, Rd, W, Wd )
        M= -self.kR*eR - self.kW*eW + np.cross(W, self.J.dot(W)) - self.J.dot(W_hat.dot(R.T.dot(Rd.dot(Wd))) - R.T.dot(Rd.dot(Wd_dot)))
        return (f, M)

def get_Rc(A, A_dot, A_2dot, b1d, b1d_dot, b1d_ddot):
    # move this as a function
    norm_A = la.norm(A)
    b3c = - A/norm_A
    b3c_dot = - A_dot/norm_A + ( np.dot(A, A_dot)*A )/norm_A**3
    b3c_2dot = - A_2dot/norm_A + ( 2*np.dot(A*A_dot,A_dot) )/norm_A**3 + np.dot( A_dot* A_dot + A*A_2dot ,A)/norm_A**3 - 3*np.dot((A*A_dot)**2,A)/norm_A**5

    b_ = np.cross(b3c, b1d)
    b_norm = la.norm(b_)
    b_dot = np.cross(b3c_dot, b1d) + np.cross(b3c, b1d_dot)
    b_2dot = np.cross(b3c_2dot, b1d) + 2*np.cross(b3c_dot, b1d_dot) + np.cross(b3c, b1d_ddot)

    b1c = -np.cross( b3c, b_ )/b_norm
    b1c_dot = -( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )/b_norm + np.cross(b3c, b_)*(b_dot* b_)/b_norm**3

    # intermediate steps to calculate b1c_2dot
    m_1 = ( np.cross(b3c_2dot, b_) + 2*np.cross(b3c_dot, b_dot) + np.cross(b3c, b_2dot) )/b_norm
    m_2 = ( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )*np.dot(b_dot, b_)/b_norm**3
    m_dot = m_1 - m_2
    n_1 = np.cross(b3c, b_)*np.dot(b_dot, b_)
    n_1dot = ( np.cross(b3c_dot, b_) + np.cross(b3c, b_dot) )*np.dot(b_dot, b_) + np.cross(b3c, b_)*( np.dot(b_2dot, b_)+np.dot(b_dot, b_dot) )
    n_dot = n_1dot/b_norm**3 - 3*n_1*np.dot(b_dot, b_)/b_norm**5
    b1c_2dot = -m_dot + n_dot

    Rc = np.reshape([b1c, np.cross(b3c, b1c), b3c],(3,3)).T
    Rc_dot = np.reshape([b1c_dot, ( np.cross(b3c_dot, b1c) + np.cross(b3c, b1c_dot) ), b3c_dot],(3,3)).T
    Rc_2dot = np.reshape( [b1c_2dot, ( np.cross(b3c_2dot, b1c) + np.cross(b3c_dot, b1c_dot) + np.cross(b3c_dot, b1c_dot) + np.cross(b3c, b1c_2dot) ), b3c_2dot],(3,3)).T
    Wc = vee(Rc.T.dot(Rc_dot))
    Wc_dot= vee( Rc_dot.T.dot(Rc_dot) + Rc.T.dot(Rc_2dot))
    return (Rc, Wc, Wc_dot)

def vee(M):
    return np.array([M[2,1], M[0,2], M[1,0]])


def attitude_errors( R, Rd, W, Wd ):
    eR = 0.5*vee(Rd.T.dot(R) - R.T.dot(Rd))
    eW = W - R.T.dot(Rd.dot(Wd))
    return (eR, eW)

def position_errors(x, xd, v, vd):
    ex = x - xd
    ev = v - vd
    return (ex, ev)

def rot_eul(x_in):
    theta_x = np.arctan2(x_in[:,7], x_in[:,8])
    theta_y = np.arctan2(x_in[:,6], (x_in[:,7]**2+x_in[:,8]**2)**(1/2))
    theta_z = np.arctan2(x_in[:,1], x_in[:,0])
    return np.array([theta_x,theta_y,theta_z]).T

def hat(x):
    hat_x = [0, -x[2], x[1],
            x[2], 0, -x[0],
            -x[1], x[0], 0]
    return np.reshape(hat_x,(3,3))


if __name__ == "__main__":
      # execute only if run as a script
    ukf_flag = True
    anim_flag = False
    J = np.diag([0.0820, 0.0845, 0.1377])
    e3 = np.array([0.,0.,1.])
    uav_t = UAV(J, e3)
    t_max = 12
    N = 100*t_max + 1
    t = np.linspace(0,t_max,N)
    xd = np.array([0.,0.,0.])
    # Initial Conditions
    R0 = [[1., 0., 0.],
            [0., -0.9995, -0.0314],
            [0., 0.0314, -0.9995]] # initial rotation
    R0 = np.eye(3)
    W0 = [0.,0.,0.];   # initial angular velocity
    x0 = [0.,0.,0.];  # initial position (altitude?0)
    v0 = [0.,0.,0.];   # initial velocity
    R0v = np.array(R0).flatten().T
    y0 = np.concatenate((x0, v0, R0v, W0))

    # sim = odeint(uav_t.dydt,y0,t)

    solver = ode(uav_t.dydt)
    solver.set_integrator('dopri5').set_initial_value(y0, 0)
    dt = 1./100
    sim = []
    xd = []
    xd_dot = []
    command_val = []
    while solver.successful() and solver.t < t_max:
        solver.integrate(solver.t+dt)
        sim.append(solver.y)
        xd.append(uav_t.xd)
        xd_dot.append(uav_t.xd_dot)
        command_val.append(uav_t.command)

    sim = np.array(sim)
    xd = np.array(xd)
    xd_dot = np.array(xd_dot)
    # def animate(i):
    #fig = plt.figure()
    #ax = fig.gca(projection = '3d')
    #ax.set_aspect('equal')
    #ax.plot(sim[:,0],sim[:,1],sim[:,2])
    #ax.set_xlim(0, 10)
    #ax.set_ylim(-5, 5)
    #ax.set_zlim(-5, 5)
    #plt.show()
    if anim_flag:
        f, ax = plt.subplots(3,2)
        ax[0][0].plot(xd[:,0])
        ax[0][0].plot(sim[:,0])
        ax[0][1].plot(xd_dot[:,0])
        ax[0][1].plot(sim[:,3])

        ax[1][0].plot(xd[:,1])
        ax[1][1].plot(xd_dot[:,1])
        ax[1][0].plot(sim[:,1])
        ax[1][1].plot(sim[:,4])

        ax[2][0].plot(xd[:,2])
        ax[2][1].plot(xd_dot[:,2])
        ax[2][0].plot(sim[:,2])
        ax[2][1].plot(sim[:,5])
        plt.show()
        pass

    if ukf_flag == False:
        sys.exit()

    ukf_test = ukf_uav.UnscentedKalmanFilter(12,6,0.01)
    Ns = 12
    q = 0.1
    r = 0.1
    ukf_test.J = J
    ukf_test.e3 = e3
    ukf_test.Q = q**2*np.eye(Ns)
    ukf_test.R = r**2
    ukf_test.P = np.eye(Ns)*1
    x = np.zeros(Ns)
    P = np.eye(Ns)*1

    x_ukf = []
    x_sensor = []
    for i, k in enumerate(sim):
        Rot = np.reshape(k[6:15],(-1,9))
        Rot_e = rot_eul(Rot)
        noise = np.zeros(Ns)
        noise[:3] = r*(0.5-np.random.random(3))
        x_obs = np.concatenate((k[:6],np.reshape(Rot_e,(3,)),k[-3:])) + noise
        z = ukf_test.sss(x_obs)# + r*(0.5-np.random.random(6))
        x_sensor.append(z)
        ukf_test.Rb = Rot.reshape((3,3))
        x, P = ukf_test.ukf(x,P, z, ukf_test.Q, ukf_test.R, command_val[i])
        # x_obs = ukf_test.dss(x,command_val[i])# + q*(0.5-np.random.random(Ns))
        x_ukf.append(x)
        pass
    # fig, ax = plt.subplots()
    x_estimate = np.array(x_ukf)
    x_sensor = np.array(x_sensor)
    f, (ax0, ax1, ax2) = plt.subplots(3,1)
    ax0.plot(sim[:,0],'b--')
    ax0.plot(x_sensor[:,0],'gx')
    ax0.plot(x_estimate[:,0],'r')
    ax1.plot(sim[:,1],'b--')
    ax1.plot(x_sensor[:,1],'gx')
    ax1.plot(x_estimate[:,1],'r')
    ax2.plot(sim[:,2],'b--')
    ax2.plot(x_sensor[:,2],'gx')
    ax2.plot(x_estimate[:,2],'r')

    plt.show()
    sys.exit()


    fig = plt.figure()
    ax = p3.Axes3D(fig)
    xs = sim[:,-6]
    ys = sim[:,-5]
    zs = sim[:,-4]

    x_estimate = np.array(x_ukf)
    x_sensor = np.array(x_sensor)
    ax.plot(xs,ys,zs,'b')
    ax.plot(x_estimate[:,0],x_estimate[:,1],x_estimate[:,2],'r')
    ax.plot(x_sensor[:,0],x_sensor[:,1],x_sensor[:,2],'g--')
    fig.show()
  #  plt.figure()
  #  plt.subplot(211)
  #  plt.plot(t,sim[:,-6:-3])
  #  plt.grid()
  #  plt.subplot(212)
  #  plt.plot(t,rot_eul(sim))
  #  plt.grid()
  #  plt.show()
  #  plt.close()
  #
  #  anim_on = 1
  #  if anim_on:
  #      mlab.figure(bgcolor=(0.839216, 0.839216, 0.839216))
  #    mlab.roll(45)
  #    pt = mlab.points3d(xs[0], ys[0], zs[0],color =  (1, 0, 0), opacity=0.5, scale_factor = 0.1)
  #    path = mlab.plot3d(xs,ys,zs, color = (0.541176, 0.168627, 0.886275),tube_radius = 0.01, opacity=0.5)
  #
  #    wx = np.linspace(-1,1,5)
  #    wy = wx
  #    [wx,wy] = np.meshgrid(wx,wy)
  #    wz = np.zeros(wx.shape)
  #    ground = mlab.mesh(wx,wy,wz,color=(0,0,0),representation='wireframe')
  #
  #    xaz = mlab.plot3d([xs[0], xs[0]] ,[ys[0],ys[0]],[zs[0],zs[0]],color=(0,0,1),tube_radius = 0.01)
  #    xax = mlab.plot3d([xs[0], xs[0]] ,[ys[0],ys[0]],[zs[0],zs[0]],color=(1,0,0),tube_radius = 0.01)
  #    xay = mlab.plot3d([xs[0], xs[0]] ,[ys[0],ys[0]],[zs[0],zs[0]],color=(0,1,0),tube_radius = 0.01)
  #    # zipped = zip(xx,xy,xz)
  #
  #    eul_ang = rot_eul(sim)
  #    # @mlab.show
  #    @mlab.animate(delay=10)
  #    def anim():
  #        f = mlab.gcf()
  #        while True:
  #            i = 0
  #            for (x, y, z, angle) in zip(xs, ys, zs, sim[:,:9]):
  #                pt.mlab_source.set(x=x, y=y, z=z)
  #                ptx = angle.reshape((3,3)).dot([1,0,0])*0.2
  #                ptx = [x,y,z] + ptx
  #                xax.mlab_source.set(x=[x, ptx[0]] ,y=[y,ptx[1]],z=[z, ptx[2]])
  #                ptx = angle.reshape((3,3)).dot([0,1,0])*0.2
  #                xay.mlab_source.set(x=[x, x + ptx[0]] ,y=[y,y+ptx[1]],z=[z,z+ptx[2]])
  #                ptx = angle.reshape((3,3)).dot([0,0,1])*0.2
  #                xaz.mlab_source.set(x=[x, x + ptx[0]] ,y=[y,y+ptx[1]],z=[z,z+ptx[2]])
  #                f.scene.render()
  #                i += 1
  #                yield
  #
  #    # Run the animation.
  #    anim()
  #    mlab.show()
  #
  #
  #
  #
    # x = np.arange(0, 2*np.pi, 0.01)
    # line, = ax.plot(sim[:,-6], sim[:,-5])
  
    # def animate(i):
    #   line.set_data(np.vstack([sim[:i,-6], sim[:i,-5]]))  # update the data
    #   line.set_3d_properties(sim[:i,-4])
    #   return line,
  
    # Setting the axes properties
    # ax.set_xlim3d([-1.0, 1.0])
    # ax.set_xlabel('X')
    # ax.set_ylim3d([-1.0, 1.0])
    # ax.set_ylabel('Y')
    # ax.set_zlim3d([0.0, 1.0])
    # ax.set_zlabel('Z')
    # Init only required for blitting to give a clean slate.
    # def init():
      # print(np.concatenate((x.T,np.ma.array(x, mask=True).T)).shape)
      # pdb.set_trace()
      # line.set_data(np.concatenate((x,np.ma.array(x, mask=True))))
      # line.set_3d_properties(x)
      # return line,
    # ani = animation.FuncAnimation(fig, animate, np.arange(N),
                                # interval=25, blit=False)
  
