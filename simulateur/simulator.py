import numpy as np

class Quadcopter:
    def __init__(self, m, I_body, dt=0.01):
        self.m = m
        self.I = np.diag(I_body)
        self.I_inv = np.diag(1/np.array(I_body))
        self.dt = dt
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1., 0., 0., 0.])
        self.omega = np.zeros(3)
        self.g = np.array([0, 0, -9.81])

    def _quat_mul(self, q, r):
        w0, x0, y0, z0 = q
        w1, x1, y1, z1 = r
        return np.array([
            w0*w1 - x0*x1 - y0*y1 - z0*z1,
            w0*x1 + x0*w1 + y0*z1 - z0*y1,
            w0*y1 - x0*z1 + y0*w1 + z0*x1,
            w0*z1 + x0*y1 - y0*x1 + z0*w1
        ])

    def step(self, torque, thrust):
        # Linear acceleration
        tb = np.array([0, 0, 0, thrust])
        q_conj = self.q * np.array([1, -1, -1, -1])
        tmp = self._quat_mul(self.q, tb)
        tb_i = self._quat_mul(tmp, q_conj)[1:]
        a = self.g + tb_i / self.m

        # Angular acceleration
        omega_dot = self.I_inv @ (torque - np.cross(self.omega, self.I @ self.omega))

        # Integrate (Euler)
        self.p     += self.v * self.dt
        self.v     += a * self.dt
        self.omega += omega_dot * self.dt
        omega_q = np.hstack(([0.], self.omega))
        q_dot = 0.5 * self._quat_mul(self.q, omega_q)
        self.q += q_dot * self.dt
        self.q /= np.linalg.norm(self.q)

        return self.p.copy(), self.v.copy(), self.q.copy(), self.omega.copy()