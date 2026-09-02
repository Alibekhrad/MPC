# Summary: Model Predictive Control (MPC) - Fundamentals

## 1. Introduction to Model Predictive Control
* **Model Predictive Control (MPC)**, also known as receding horizon control, is a process control technique that optimizes a cost function over a future horizon using a model of the system.
* Unlike standard LQR, MPC explicitly handles **state and input constraints** ($x_k \in \mathcal{X}_k$, $u_k \in \mathcal{U}_k$).
* The finite-horizon constrained optimization problem solved at each time step is:
  $$\min_{\mathbf{u}} \sum_{k=0}^{K-1} g_k(x_k, u_k) + g_K(x_K)$$
  subject to the system dynamics $x_{k+1} = f(x_k, u_k)$ and boundary/path constraints.
* **Key Characterization:** MPC acts as a **static, nonlinear, time-invariant feedback control law**. It is static because it depends only on the current state (no memory), nonlinear due to the presence of inequality constraints (even if the system is linear), and time-invariant because the same optimization problem is shifted and solved at each step.

## 2. The Receding Horizon Strategy (Closed-Loop vs. Open-Loop)
* **Open-Loop:** Solving the optimization once yields an optimal sequence $\mathbf{u}^* = \{u_0^*, u_1^*, \dots, u_{K-1}^*\}$. Applying this entire sequence blindly leaves the system vulnerable to disturbances and model mismatches.
* **Closed-Loop (MPC Approach):** 
  1. Measure the current state $x_i$.
  2. Solve the optimization problem over a horizon $K$ to find the optimal sequence $\mathbf{u}_i^*$.
  3. **Apply only the first element** ($u_{i,0}^*$) to the system and discard the rest.
  4. Step forward in time, measure the new state $x_{i+1}$, and repeat the process.

## 3. Does MPC Realize the Optimal Control Solution? (Pathological Case)
* **No, not necessarily.** Because MPC constantly truncates the optimal sequence and only applies the first step, the actual trajectory of the system might not match the theoretically optimal trajectory.
* **Pathological Example:** Consider the cost function $\sum_{k=0}^{K-1} \frac{1}{k} ||u_k||^2 + ||x_K||^2$. 
  * To avoid the infinite cost at $k=0$ (since $1/0 \to \infty$), the optimizer dictates $u_0^* = 0$ and delays the control effort to future steps ($k \ge 1$).
  * However, because MPC *always* executes only $u_0^*$, it will repeatedly apply $u=0$ at every time step.
  * Consequently, the system remains completely stuck in its initial state and never reaches the target, proving that MPC is not inherently a robust optimal controller.

## 4. MPC vs. Finite-Time LQR
* Given the identical unconstrained finite-time problem, LQR and MPC compute the exact same sequence of cost-to-go matrices $\{P_K, P_{K-1}, \dots, P_1\}$ and gain matrices $\{\Gamma_0, \Gamma_1, \dots\}$.
* **LQR Behavior:** Applies a **time-varying** feedback law: $u_0 = -\Gamma_0 x_0$, $u_1 = -\Gamma_1 x_1$, etc.
* **MPC Behavior:** Because the receding horizon constantly resets the problem to "step 0", MPC applies a **time-invariant** feedback law, always using the first gain matrix: $u_0 = -\Gamma_0 x_0$, $u_1 = -\Gamma_0 x_1$, etc.
* **Exception:** In the case of an *infinite horizon* ($K \to \infty$), the LQR gain converges to a constant $\Gamma_\infty$. In this specific scenario, both LQR and MPC apply the exact same time-invariant feedback law.
