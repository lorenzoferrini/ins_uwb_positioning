import numpy as np
import quaternion
import rospy
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import scipy as sp


def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])


def push(x, y):
    x[:, :-1] = x[:, 1:]
    x[:, -1:] = y  # .reshape(-1,1)
    return x


class shfaf(object):
    def __init__(self, R=None, Q=None, P=None, x=None, q=None, window_width=12, a=0.95):

        if R is None:
            R = np.diag([1, 1, 1, 1 * 5, 1 * 5, 1 * 5]) * 1e-1
        if Q is None:
            Q = np.diag([0.06 ** 2, 0.06 ** 2, 0.06 ** 2, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0])
        if P is None:
            P = np.diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0, 0, 0, 0.01, 0.01, 0.01, 0, 0, 0])
        if x is None:
            x = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         np.float)  # position, velocity, acceleration bias, omega bias
        if q is None:
            q = quaternion.from_rotation_vector(np.array([0, 0, 0.701], np.float))
            q = quaternion.as_float_array(q)

        '''
            Filter mode must be set to:
                0 - for Sage-Husa Fuzzy Adaptive Filter
                1 - for Sage-Husa Adaptive Filter
                2 - for EKF
                3 - for Inertial Navigation
        '''
        self.mode = 0
        self.on = True

        self.lamb = np.block([[np.zeros((3, 12), np.float)],
                              [np.identity(12, np.float)]])
        self.H = np.block([[np.identity(6, np.float), np.zeros((6, 9), np.float)]])
        self.threshold = 4.5
        a_v = np.ones(window_width, np.float) * a
        j = np.arange(0, window_width)
        s = np.power(a_v, j) * (1 - a_v) / (1 - np.power(a_v, window_width))
        self.sigma = np.diag(s)
        self.time = 0
        self.uwbTime = 0
        self.nAnchors = 4
        self.anchorPos = np.zeros((2, self.nAnchors), np.float)
        self.range = np.ones(self.nAnchors, np.float) * 99
        self.uwbCeck = np.zeros(self.nAnchors)  # check if all anchors' signals arrived
        self.uwbInit = 0  # flag for uwb velocity initialization
        self.pred = 0  # flag which states if prediction or correction has been performed last
        beta = 0
        self.G = 1 - np.square(beta)
        self.H_uwb = np.square(1 - beta)

        self.R = R
        self.Q = Q
        self.P = P
        self.P_ = P
        self.x = x[np.newaxis].T
        self.x_ = x[np.newaxis].T
        self.posOld = x[:3]
        self.velOld = x[3:6]
        self.q = q
        self.q_ = q
        self.dx = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], np.float)[
            np.newaxis].T  # position, velocity, orientation, acceleration bias, omega bias
        self.dx_ = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], np.float)[
            np.newaxis].T  # position, velocity, orientation, acceleration bias, omega bias
        self.innovation = np.random.normal(0.0, 0.1, (6, window_width))
        self.error = (np.random.rand(15, window_width) - 1) * 0.1
        # self.innovation_ = np.zeros((6,window_width), np.float)
        self.k = 0  # Iteration counter
        self.lambda_ = 1.0119  # Sage Huza ICW coefficent
        self.b = 0.9976  # Sage Huza ICW coefficent
        self.alpha = 0.48  # Sage Huza ICW coefficent
        H = self.H
        self.K = H.T.dot(np.linalg.inv(H.dot(H.T)))  # Kalman gain

        # Fuzzy inference

        input = ctrl.Antecedent(np.arange(0, 0.9, 0.1), 'r')
        output = ctrl.Consequent(np.arange(0.8, 2.1, 0.1), 's')

        # Define membership functions
        input['less'] = fuzz.trimf(input.universe, [0, 0, 0.3])
        input['equal'] = fuzz.trimf(input.universe, [0.1, 0.4, 0.7])
        input['more'] = fuzz.trimf(input.universe, [0.5, 0.8, 0.8])

        output['less'] = fuzz.trimf(output.universe, [0.8, 0.8, 1.2])
        output['equal'] = fuzz.trimf(output.universe, [1, 1.4, 1.8])
        output['more'] = fuzz.trimf(output.universe, [1.6, 2.0, 2.0])

        # Fuzzy rules
        rule1 = ctrl.Rule(input['equal'], output['equal'])
        rule2 = ctrl.Rule(input['more'], output['more'])
        rule3 = ctrl.Rule(input['less'], output['less'])

        fuzzy_control = ctrl.ControlSystem([rule1, rule2, rule3])
        self.fuzzy = ctrl.ControlSystemSimulation(fuzzy_control)

    def _rotationMatrix(self, q):
        q = quaternion.from_float_array(q)
        C = quaternion.as_rotation_matrix(q)
        return C

    def prediction(self, am, wm, t):

        am = am[np.newaxis].T
        wm = wm[np.newaxis].T
        wm[:2, :] = 0
        if self.uwbInit:
            dt = t - self.time
        else:
            dt = 0.01

        if self.pred:
            x = self.x_
            P = self.P_
            q = self.q_
        else:
            x = self.x
            P = self.P
            q = self.q
        C = self._rotationMatrix(q)
        self.Q = sp.linalg.block_diag(np.eye(3) * ((0.06 * dt) ** 2), np.eye(3) * ((0.0 * dt) ** 2),
                                      np.eye(3) * ((0.001 * dt) ** 2), np.eye(3) * ((0.0 * dt) ** 2))
        Q = self.Q
        w = wm * dt
        g = np.array([0, 0, 9.8])[np.newaxis].T
        x_dot = np.concatenate([x[3:6, :],
                                C.dot((am - x[6:9, :])) - g,
                                np.zeros(6)[np.newaxis].T
                                ])
        x_next = x + x_dot * dt

        if np.linalg.norm(w) != 0:
            qq = quaternion.as_quat_array(q)
            wq = quaternion.from_rotation_vector(w[:, 0] * np.array((0, 0, 1)))
            q_next = quaternion.as_float_array(qq * wq)
        else:
            q_next = q
        F = np.block([[np.identity(3, np.float), np.identity(3, np.float) * dt, np.zeros((3, 9))],
                      [np.zeros((3, 3), np.float), np.identity(3, np.float), -(C.dot(skew(am - x[6:9, :]))) * dt,
                       -C * dt, np.zeros((3, 3), np.float)],
                      [np.zeros((3, 6), np.float), self._rotationMatrix(
                          quaternion.as_float_array(quaternion.from_euler_angles(((wm - x[9:12, :]) * dt)[:, 0]))),
                       np.zeros((3, 3), np.float),
                       -np.identity(3, np.float) * dt],
                      [np.zeros((3, 9), np.float), np.identity(3, np.float), np.zeros((3, 3), np.float)],
                      [np.zeros((3, 12), np.float), np.identity(3, np.float)]])
        self.P_ = (F.dot(P.dot(F.T)) + self.lamb.dot(Q.dot(self.lamb.T)))
        self.dx_ = F.dot(self.dx)
        self.k = self.k + 1
        self.x_ = x_next
        self.q_ = q_next
        self.time = t
        self.pred = 1
        self.uwbInit = 1

        return x_next, q_next

    def fuzzyInference(self, r):

        self.fuzzy.input['r'] = r
        self.fuzzy.compute()

        return self.fuzzy.output['s']

    def correction(self):
        pos = self.posOld
        vel = self.velOld
        measure = np.concatenate((pos, vel))[np.newaxis].T
        H = self.H
        z = measure - self.x_[:6, :]
        check = np.ones(6, int)[np.newaxis].T
        S_teo = H.dot(self.P_.dot(H.T)) + self.R

        # Filter outliers
        while sum(check)[0] != 0:
            inn = z - H.dot(self.dx_)
            ep = push(self.innovation, inn)
            S = ep.dot(self.sigma.dot(ep.T))
            if self.mode == 2:
                break
            D = S_teo + H.dot(self.dx_.dot(self.dx_.T.dot(H.T)))
            G = S + H.dot(self.dx_.dot(self.dx_.T.dot(H.T)))
            m = (np.abs(np.diagonal(G) / np.diagonal(D)))[np.newaxis].T
            check = np.greater(m, self.threshold).astype(int)[np.newaxis].T
            z = (z * (check * (1 / np.sqrt(m) - 1) + 1)).reshape(-1, 1)

        self.innovation = ep
        if self.mode == 0:
            r = np.abs(np.trace(S) / np.trace(S_teo) - 1)
            s = self.fuzzyInference(r)
        elif self.mode == 1:
            s = 1
        elif self.mode == 2:
            s = 0
            self.R = np.diag([1, 1, 1, 1, 1, 1]) * 1e-1

        d = (self.lambda_ - self.b) / (self.lambda_ - np.power(self.b, self.k + 1))

        # Estimate measurement noise covariance
        self.R = (1 - np.power(s, self.alpha) * d) * self.R + np.power(s, self.alpha) * d * (
                S - H.dot(self.P_.dot(H.T)))

        # Calculate Kalman gain
        self.K = self.P_.dot(H.T.dot(np.linalg.inv(H.dot(self.P_.dot(H.T)) + self.R)))

        # Update error state and its covariance
        self.dx = self.K.dot(inn)
        self.error = push(self.error, self.dx)
        self.P = (np.identity(15, float) - self.K.dot(H)).dot(self.P_)

        # Update the nominal state
        dx_pos = np.concatenate((self.dx[:6], self.dx[9:]))
        self.x = self.x_ + dx_pos
        dq = quaternion.from_euler_angles(np.concatenate((np.zeros(2), self.dx[8])))
        self.q = quaternion.as_float_array(quaternion.from_float_array(self.q_) * dq)


        # Reset the error state
        self.dx = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], np.float)[np.newaxis].T
        self.dx_ = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], np.float)[np.newaxis].T
        self.pred = 0
        self.k = 0

    def uwbRange2Pos(self):
        r = self.range
        anc = self.anchorPos
        t = rospy.get_time()
        dt = t - self.uwbTime
        G = anc[0, :] - anc[1:, :]
        b = np.square(r[1:]) - np.square(r[0]) + np.square(anc[0, 0]) - np.square(anc[1:, 0]) + np.square(
            anc[0, 1]) - np.square(anc[1:, 1])  # + np.square(anc[0,2]) - np.square(anc[1:,2])
        pos_ = 0.5 * np.linalg.inv(G.T.dot(G)).dot(G.T.dot(b))
        pos = self.posOld + self.velOld * dt + self.G * (pos_ - (self.posOld + self.velOld * dt))
        vel = self.velOld + self.H_uwb / dt * (pos_ - (self.posOld + self.velOld * dt))
        self.uwbTime = t
        self.posOld = pos
        self.velOld = vel
        return pos, vel
