import numpy as np
from controller import Dumbbell
from kinematics import attitude

angle = (2*np.pi- 0) * np.random.rand(1) + 0

# generate a random initial state that is outside of the asteroid
pos = np.random.rand(3)+np.array([2,2,2])
R = attitude.rot1(angle).reshape(9)
vel = np.random.rand(3)
ang_vel = np.random.rand(3)
t = np.random.rand()*100

state = np.hstack((pos,vel, R, ang_vel))

class TestDumbbellInertialDesiredAttitude():
    
    dum = Dumbbell()
    alpha = np.random.rand()
    axis = np.array([1, 0, 0])
    Rd, Rd_dot, ang_vel_d, ang_vel_d_dot = dum.desired_attitude(1, alpha, axis)

    def test_desired_rotation_matrix_determinant(self):
        np.testing.assert_almost_equal(np.linalg.det(self.Rd), 1) 
    
    def test_desired_rotation_matrix_orthogonal(self):
        np.testing.assert_array_almost_equal(self.Rd.T.dot(self.Rd), 
                np.eye(3,3))

    def test_desired_attitude_satifies_kinematics(self):
        np.testing.assert_array_almost_equal(self.Rd_dot,
                self.Rd.dot(attitude.hat_map(self.ang_vel_d)))
 
class TestDumbbellInertialAttitudeController():
    """Test the attitude controller for the inertial eoms
    """
    dum = Dumbbell()
    u_m = dum.attitude_controller(t, state, np.zeros(3))

    def test_control_moment_size(self):
        np.testing.assert_equal(self.u_m.shape, (3,))

class TestDumbbellInertialTranslationalController():
    
    dum = Dumbbell()
    u_f = dum.translation_controller(t, state, np.zeros(3))

    def test_control_force_size(self):
        np.testing.assert_equal(self.u_f.shape, (3,))
