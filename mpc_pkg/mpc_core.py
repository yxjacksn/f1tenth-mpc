"""
MPC solver using CasADi + IPOPT.

Pure math — no ROS imports. Testable standalone.

Formulation:
  State:   z = [X, Y, theta, v]
  Control: u = [delta, a]
  Dynamics: kinematic bicycle (forward Euler)
  Cost: quadratic tracking + input + input-rate penalties
  Constraints: box constraints on states and inputs
"""

import casadi as ca
import numpy as np


class MPCCore:
    def __init__(self, params: dict):
        self.N = params['horizon']
        self.dt = params['dt']
        self.L = params['wheelbase']

        # Cost weights
        self.Q = np.diag([
            params['q_x'],
            params['q_y'],
            params['q_theta'],
            params['q_v'],
        ])
        self.R = np.diag([
            params['r_delta'],
            params['r_a'],
        ])
        self.R_d = np.diag([
            params['r_d_delta'],
            params['r_d_a'],
        ])

        # Bounds
        self.delta_max = params['delta_max']
        self.v_min = params['v_min']
        self.v_max = params['v_max']
        self.a_max = params['a_max']

        # Build the NLP once at init (CasADi symbolic → compiled)
        self._build_solver()

        # Warm start storage
        self._prev_u = np.zeros((self.N, 2))

    def _build_solver(self):
        """Construct the CasADi NLP. Called once."""
        N, dt, L = self.N, self.dt, self.L
        nx, nu = 4, 2

        # Decision variables: all states + all controls flattened
        # Layout: [z_0, z_1, ..., z_N, u_0, u_1, ..., u_{N-1}]
        n_vars = (N + 1) * nx + N * nu
        w = ca.MX.sym('w', n_vars)

        # Parameters: initial state (4) + reference trajectory ((N+1)*4) + previous control (2)
        n_params = nx + (N + 1) * nx + nu
        p = ca.MX.sym('p', n_params)

        # Helpers to index into w
        def get_z(k):
            start = k * nx
            return w[start:start + nx]

        def get_u(k):
            start = (N + 1) * nx + k * nu
            return w[start:start + nu]

        # Extract parameters
        z0 = p[0:nx]
        ref = ca.reshape(p[nx:nx + (N + 1) * nx], nx, N + 1).T  # (N+1) x 4
        u_prev = p[nx + (N + 1) * nx: nx + (N + 1) * nx + nu]

        # Build cost and constraints
        J = 0.0
        g = []       # equality constraints (dynamics)
        lbg = []
        ubg = []

        # Initial state constraint
        g.append(get_z(0) - z0)
        lbg += [0.0] * nx
        ubg += [0.0] * nx

        Q = ca.DM(self.Q)
        R = ca.DM(self.R)
        R_d = ca.DM(self.R_d)

        for k in range(N):
            zk = get_z(k)
            uk = get_u(k)
            zk1 = get_z(k + 1)

            X, Y, theta, v = zk[0], zk[1], zk[2], zk[3]
            delta, a = uk[0], uk[1]

            # Kinematic bicycle (forward Euler)
            z_next = ca.vertcat(
                X + v * ca.cos(theta) * dt,
                Y + v * ca.sin(theta) * dt,
                theta + v * ca.tan(delta) / L * dt,
                v + a * dt,
            )

            # Dynamics constraint
            g.append(zk1 - z_next)
            lbg += [0.0] * nx
            ubg += [0.0] * nx

            # Tracking cost
            e = zk - ref[k, :].T
            # Wrap theta error to [-pi, pi]
            e_theta = e[2]
            # CasADi doesn't have a clean modulo; use atan2(sin, cos) trick
            e[2] = ca.atan2(ca.sin(e_theta), ca.cos(e_theta))
            J += ca.mtimes([e.T, Q, e])

            # Input cost
            J += ca.mtimes([uk.T, R, uk])

            # Input rate cost
            if k == 0:
                du = uk - u_prev
            else:
                du = uk - get_u(k - 1)
            J += ca.mtimes([du.T, R_d, du])

        # Terminal tracking cost
        e_N = get_z(N) - ref[N, :].T
        e_N[2] = ca.atan2(ca.sin(e_N[2]), ca.cos(e_N[2]))
        J += ca.mtimes([e_N.T, Q, e_N])

        # Variable bounds
        lbw = []
        ubw = []
        for k in range(N + 1):
            lbw += [-1e9, -1e9, -1e9, self.v_min]    # X, Y, theta, v
            ubw += [1e9, 1e9, 1e9, self.v_max]
        for k in range(N):
            lbw += [-self.delta_max, -self.a_max]
            ubw += [self.delta_max, self.a_max]

        # NLP
        nlp = {
            'x': w,
            'f': J,
            'g': ca.vertcat(*g),
            'p': p,
        }

        opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0,
            'ipopt.max_iter': 50,
            'ipopt.warm_start_init_point': 'yes',
        }

        self._solver = ca.nlpsol('mpc', 'ipopt', nlp, opts)
        self._n_vars = n_vars
        self._n_params = n_params
        self._lbw = np.array(lbw)
        self._ubw = np.array(ubw)
        self._lbg = np.array(lbg)
        self._ubg = np.array(ubg)

        # Store dimensions
        self._nx = nx
        self._nu = nu

    def solve(self, z0: np.ndarray, ref: np.ndarray, u_prev: np.ndarray):
        """
        Solve the MPC problem.

        Args:
            z0: current state [X, Y, theta, v], shape (4,)
            ref: reference trajectory, shape (N+1, 4) — [X, Y, theta, v] per step
            u_prev: previous applied control [delta, a], shape (2,)

        Returns:
            u_opt: optimal first control [delta, a], shape (2,)
            pred_z: predicted state trajectory, shape (N+1, 4)
            solve_success: bool
        """
        N, nx, nu = self.N, self._nx, self._nu

        # Pack parameters
        p = np.concatenate([z0, ref.flatten(), u_prev])

        # Warm start: shift previous solution
        w0 = np.zeros(self._n_vars)
        # States: shift left, repeat last
        for k in range(N):
            w0[k * nx:(k + 1) * nx] = z0  # fallback
        w0[N * nx:(N + 1) * nx] = z0
        # Controls: use previous solution shifted
        ctrl_start = (N + 1) * nx
        for k in range(N - 1):
            w0[ctrl_start + k * nu:ctrl_start + (k + 1) * nu] = self._prev_u[min(k + 1, N - 1)]
        w0[ctrl_start + (N - 1) * nu:ctrl_start + N * nu] = self._prev_u[-1]

        try:
            sol = self._solver(
                x0=w0,
                lbx=self._lbw,
                ubx=self._ubw,
                lbg=self._lbg,
                ubg=self._ubg,
                p=p,
            )
            w_opt = np.array(sol['x']).flatten()
            stats = self._solver.stats()
            success = stats['return_status'] == 'Solve_Succeeded'
        except Exception:
            return u_prev.copy(), np.tile(z0, (N + 1, 1)), False

        # Extract solution
        pred_z = np.zeros((N + 1, nx))
        for k in range(N + 1):
            pred_z[k] = w_opt[k * nx:(k + 1) * nx]

        u_all = np.zeros((N, nu))
        for k in range(N):
            u_all[k] = w_opt[ctrl_start + k * nu:ctrl_start + (k + 1) * nu]

        # Save for warm start
        self._prev_u = u_all.copy()

        u_opt = u_all[0]
        return u_opt, pred_z, success
