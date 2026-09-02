# Summary: Model Predictive Control (MPC) - Fundamentals

## 1. Introduction to Model Predictive Control
* **Model Predictive Control (MPC)**, also known as receding horizon control, is a process control technique that optimizes a cost function over a future horizon using a model of the system.
* Unlike standard LQR, MPC explicitly handles **state and input constraints** ($x_k \in \mathcal{X}_k$, $u_k \in \mathcal{U}_k$).
* The finite-horizon constrained optimization problem solved at each time step is:
  $$\min_{\mathbf{u}} \sum_{k=0}^{K-1} g_k(x_k, u_k) + g_K(x_K)$$
  subject to the system dynamics $x_{k+1} = f(x_k, u_k)$ and boundary/path constraints.
* **Key Characterization:** MPC acts as a **static, nonlinear, time-invariant feedback control law**. It is static because it depends only on the current state (no memory), nonlinear due to the presence of inequality constraints (even if the system is linear), and time-invariant because the same optimization problem is shifted and solved at each step.

## 2. The Receding Horizon Strategy (Closed-Loop vs. Open-Loop)
* **Open-Loop:** Solving the optimization once yields an optimal sequence $\mathbf{u}^\ast = \{u_0^\ast, u_1^\ast, \dots, u_{K-1}^\ast\}$. Applying this entire sequence blindly leaves the system vulnerable to disturbances and model mismatches.
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

## 5. Stability Analysis (Lyapunov Theorem)
* Because MPC inherently acts as a **nonlinear** control law (due to the presence of constraints), classical linear analysis tools (like Nyquist or Bode plots) are insufficient for proving global stability.
* **Lyapunov's Direct Method:** To prove asymptotic stability for a discrete system $x_{k+1} = f(x_k)$ around the equilibrium $x=0$, one must find a real-valued scalar function $W(x)$ (the Lyapunov function) such that:
  1. **Positive Definite:** $W(0) = 0$ and $W(x) > 0$ for all $x \neq 0$.
  2. **Strictly Decreasing:** $W(f(x)) < W(x)$ along the trajectories of the system.

## 6. Stability of Infinite Horizon MPC
* For an infinite horizon problem, the optimal cost-to-go function $V_i^\infty(\mathbf{x}, \mathbf{u})$ acts as a perfect candidate for the Lyapunov function $W(x)$.
* According to **Bellman's principle of optimality**, the remaining "tail" of an optimal trajectory is also optimal. Therefore, as the system moves one step forward:
  $$W(x_{i+1}) = W(x_i) - g_i(x_i, u_0^*(x_i))$$
* As long as the stage cost $g_i$ is **strictly positive** (i.e., all states are observable/penalized in the cost function), the equation guarantees $W(x_{i+1}) < W(x_i)$. This proves that the energy continuously decreases, guaranteeing asymptotic stability.

## 7. Stability of Finite Horizon MPC
* In finite-horizon MPC (length $K$), the simple proof above fails. When shifting the window forward from $i$ to $i+1$, the cost of the first step is removed, but a new, unknown cost is added at the end of the horizon ($g(x_{i+K}, u_{i+K})$). Therefore, a strict decrease in $W(x)$ is no longer guaranteed.
* **The Solution (Terminal Constraint):** To recover stability, a strict boundary constraint is added to the optimization problem, forcing the system to reach the origin exactly at the end of the horizon: **$x_K = 0$**.
* **Proof Intuition:** By forcing $x_K = 0$, the controller can construct a feasible (suboptimal) sequence for step $i+1$ by shifting the previous inputs and appending $u=0$ at the end. Since the system was already at $x=0$, applying $u=0$ incurs zero additional cost at the end of the new horizon. Thus, the total cost strictly decreases by the amount of the first step's cost, satisfying the Lyapunov condition.

## 8. Warning: Consistency of Cost and Equilibrium
* The stability proofs fundamentally rely on the target equilibrium point being the absolute minimum of the cost function.
* **Example 2.1.1 takeaway:** If a system requires a non-zero input (e.g., $u=1$) to maintain the state at $x=0$, standard quadratic costs ($g = x^2 + u^2$) will create a conflict, causing the optimizer to settle at a wrong, non-zero state. The cost function must be shifted (e.g., $g = x^2 + (u-1)^2$) so that its mathematical minimum perfectly aligns with the physical equilibrium point $(x=0, u=1)$.

  ## 9. Implementation of MPC: Offline vs. Online Computation
* **Offline Computation (Explicit MPC):** This method computes the control law $u_0^*(x)$ as a function of the state space prior to operation.
  * **KKT Conditions:** The offline solution relies on the Karush-Kuhn-Tucker conditions. The solution space is divided into regions where constraints are either active (on the boundary) or inactive (inside the domain).
  * **The Curse of Dimensionality:** For a system with $n$ constraints, the state space can shatter into up to $2^n$ distinct regions. This exponential growth makes offline MPC intractable for systems with many constraints, as storing and searching through these regions becomes impossible.
* **Online Computation:** Instead of pre-computing all regions, the online method solves a numerical optimization problem at every single time step to find just the *value* of $u_0^*(x)$. 
  * Because the reachable state space in real-time operation is vastly smaller than the theoretical full state space, online computation drastically reduces storage requirements, making it the standard approach for slow dynamics systems.

## 10. Managing the Infinite Horizon (Practical Approximations)
* Real-world problems require infinite horizon stability, but calculating it is computationally impossible for constrained systems. MPC uses a finite horizon $K$ as a tractable approximation.
* To ensure the truncated "tail" (from $K$ to $\infty$) doesn't destroy stability, one of the following methods must be applied:
  1. **Zero Terminal Constraint:** Forcing the final state to reach the origin ($x_K = 0$).
  2. **Terminal Set:** Forcing the final state into a small, easily controllable invariant region ($\mathcal{X}_K$).
  3. **Terminal Cost:** Adding a heavy penalty ($g_K(x_K)$) on the final state to approximate the infinite cost-to-go.

## 11. Steady-State Selection and Incremental MPC
* **Non-Zero Equilibria:** If the goal is not the origin $(0,0)$ but a specific working point $(x_s, u_s)$, the MPC cost function must be shifted to $g_k(x - x_s, u - u_s)$. If the desired specification $(x_{spec}, u_{spec})$ is not physically feasible, an offline optimization is first run to find the closest valid steady-state $(x_s, u_s)$.
* **Incremental Formulation:** To reject unmeasured disturbances and ensure zero steady-state error (offset-free tracking), an integrator must be embedded in the controller.
  * Instead of optimizing the absolute control input $u_k$, the controller optimizes the **change in input** $\Delta u_k$.
  * The state space is augmented to include the previous input: $ egin{bmatrix} x & u \end{bmatrix}^	op$.
  * This allows the controller to continuously adjust the input (like a gas pedal adjusting for wind resistance) until the setpoint is perfectly reached, while also making it easy to penalize aggressive input *changes* in the cost function.
