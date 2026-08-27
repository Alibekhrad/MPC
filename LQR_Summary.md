# Summary: Dynamic Programming and Optimal Control

## 1. Optimal Control and Dynamic Programming
* The fundamental goal of optimal control is to find a sequence of control inputs $\mathbf{u} = \{u_t\}_{t=0,1,\dots}$ that minimizes a specific cost function $J(\mathbf{u})$ for a dynamical system.
* Because this problem is typically high-dimensional and non-convex, it is simplified using **Bellman's Optimality Principle**, which breaks the sequence down into smaller, solvable stage problems.
* Assuming a Markovian state transition $x_{t+1} = f_t(x_t, u_t)$ and an additive cost structure, the problem is solved via backward induction using the Bellman equation:
$$V_t(x_t) = \min_{u_t} \{g_t(x_t, u_t) + V_{t+1}(f_t(x_t, u_t))\}$$

## 2. The Linear-Quadratic Regulator (LQR)
* The optimal control problem becomes practically solvable when the system dynamics are strictly linear and the cost function is quadratic.
* The finite-horizon LQR objective balances the penalty for state deviations against the cost of control effort:
$$\min_{\mathbf{u}} \sum_{t=0}^{T-1} (\|x_t\|_Q^2 + \|u_t\|_R^2) + \|x_T\|_S^2$$
* The weighting matrices must strictly follow $Q, S \succeq 0$ (positive semi-definite) and $R \succ 0$ (strictly positive definite), ensuring that actuating the system always incurs a realistic cost.
* The optimal control law is a linear state feedback $u_t = -K_t x_t$, where the gain $K_t$ and the cost-to-go matrix $P_t$ are computed backward using the discrete-time Riccati equation:
$$P_t = Q + A^\top P_{t+1} A - A^\top P_{t+1} B (R + B^\top P_{t+1} B)^{-1} B^\top P_{t+1} A$$

## 3. Infinite Horizon LQR and System Stability
* When the goal is to regulate a system to a stationary state indefinitely, the backward recursion converges to a time-invariant, constant matrix $P_\infty$ and a fixed feedback gain $K_\infty$.
* This constant matrix $P_\infty$ is derived by dropping the time indices and solving the Algebraic Riccati Equation (ARE).
* **Theorem 1:** The calculated feedback gain $K_\infty$ successfully stabilizes the system if and only if the system pair $(A, B)$ is stabilizable and the state cost pair $(Q^{1/2}, A)$ is observable.

## 4. Computing the Optimal Stabilizing Controller
* A major pitfall occurs if the system has unstable modes that are unobservable in the cost matrix $Q$. To minimize the energy cost $R$, the mathematical solver will ignore these modes, resulting in an "optimal" but physically unstable system.
* To prevent this and guarantee physical stability, we initialize the backward iteration of the Riccati equation with a positive definite matrix $P_0 \succ 0$ rather than zero.
* This mathematical trick effectively simulates an infinite terminal penalty for any divergent state, forcing the solver to expand control effort to constrain all unstable modes, yielding the optimal stabilizing solution $P_S$.
