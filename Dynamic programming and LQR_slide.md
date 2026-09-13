# Dynamic Programming and LQR
**Source:** Computational Control Course (Saverio Bolognani, ETH Zurich)[cite: 5]

This document provides a comprehensive summary of Dynamic Programming (DP) and the Linear-Quadratic Regulator (LQR) problem.[cite: 5]

## 1. Optimal Control Problem Formulation

A standard optimal control problem involves finding a sequence of control inputs $u_{0}, u_{1}, \dots, u_{T}$ to minimize a cost function[cite: 5]. 

The general optimization problem is formulated as:

$$
\min_{u \in \mathcal{U}} J(u)
$$

Where $\mathcal{U}$ represents the set of feasible inputs[cite: 5]. Solving this directly as a single massive optimization problem is generally intractable because:
*   The high dimensionality of the decision variable $u$[cite: 5].
*   The requirement for an extremely accurate system model or simulator[cite: 5].
*   The problem is often non-convex[cite: 5].
*   The resulting open-loop sequence is vulnerable to disturbances and uncertainty[cite: 5].

---

## 2. Problem Decomposition and Dynamic Programming

To make the problem tractable, we rely on the **Markovian assumption**[cite: 5]:
1.  The system state evolves according to a discrete-time dynamical system: $x_{t+1} = f_{t}(x_{t}, u_{t})$[cite: 5].
2.  The initial condition $x_{0} = X_{0}$ is known[cite: 5].
3.  The cost function is additive over time: $J = \sum_{t} g_{t}(x_{t}, u_{t})$[cite: 5].

### Bellman's Principle of Optimality
Bellman's principle states that an optimal policy has the property that, regardless of the initial state and initial decisions, the remaining decisions must constitute an optimal policy for the state resulting from the first decision[cite: 5].

This allows us to break the large problem into nested subproblems, moving backwards in time (Backward Induction)[cite: 5]. At each stage $t$, we define a **Value Function** $V_{t}(x_{t})$, which represents the minimum cost-to-go from state $x_{t}$[cite: 5].

The dynamic programming backward induction step is:

$$
V_{t}(x_{t}) = \min_{u_{t}} \{ g_{t}(x_{t}, u_{t}) + V_{t+1}(f_{t}(x_{t}, u_{t})) \}
$$

At the final stage $T$, the value function is simply the terminal cost:

$$
V_{T}(x_{T}) = g_{T}(x_{T})
$$

This stage problem is simple to solve if the decision space is convex, $g_{t}$ is convex in $u_{t}$, and $V_{t+1} \circ f_{t}$ is convex in $u_{t}$[cite: 5]. This naturally leads to systems with linear dynamics and quadratic costs[cite: 5].

---

## 3. Optimal Linear-Quadratic Regulation (LQR)

The LQR problem is a specific case of optimal control where the system is linear and the cost is quadratic[cite: 5].

*   **Dynamics (Markovian update):** 
    $$ x_{t+1} = A x_{t} + B u_{t} $$
*   **Cost Function:**
    $$ \sum_{t=0}^{T-1} (x_{t}^{\top} Q x_{t} + u_{t}^{\top} R u_{t}) + x_{T}^{\top} S x_{T} $$
    Where $Q \ge 0$, $S \ge 0$ (positive semi-definite) and $R > 0$ (positive definite)[cite: 5].

### Solving LQR via Backward Induction
The goal is to prove that if the terminal cost is quadratic ($V_{T}(x) = x^{\top} S x$), then the value function at any time step $t$ is also quadratic ($V_{t}(x) = x^{\top} P_{t} x$)[cite: 5].

Assuming $V_{t+1}(x) = x^{\top} P_{t+1} x$, the optimization at stage $t$ becomes an unconstrained quadratic minimization[cite: 5]. Setting the gradient with respect to $u$ to zero yields the **optimal control input**:

$$
u^{\star}_{t} = - (R + B^{\top} P_{t+1} B)^{-1} B^{\top} P_{t+1} A x_{t}
$$

By substituting $u^{\star}_{t}$ back into the value function equation, we can confirm that $V_{t}(x) = x^{\top} P_{t} x$, where the matrix $P_{t}$ is updated recursively[cite: 5]:

$$
P_{t} = Q + A^{\top} P_{t+1} A - A^{\top} P_{t+1} B (R + B^{\top} P_{t+1} B)^{-1} B^{\top} P_{t+1} A
$$

This recursive formula is known as the **Dynamic Riccati Equation**[cite: 5].

### Optimal Feedback Control vs. Open-Loop
Instead of merely computing an open-loop sequence of actions offline, LQR provides an **optimal feedback control law**[cite: 5]:

$$
u_{t} = r_{t} x_{t}
$$

Where the feedback gain $r_{t}$ is:

$$
r_{t} = - (R + B^{\top} P_{t+1} B)^{-1} B^{\top} P_{t+1} A
$$

*   **Offline Computation:** Involves matrix multiplications, matrix inversion, and $T$ iterations to compute all $P_{t}$ and $r_{t}$ matrices[cite: 5].
*   **Online Computation:** Requires storing $T$ matrices of size $m \times n$ and performing a simple matrix multiplication ($r_{t} x_{t}$) at each time step[cite: 5].

---

## 4. Infinite Horizon LQR ($T \to \infty$)

For regulation and persistent tracking problems, extending LQR to an infinite horizon ($T \to \infty$) is highly beneficial[cite: 5]. 

*   **Cost Function:**
    $$ \min \sum_{t=0}^{\infty} (x_{t}^{\top} Q x_{t} + u_{t}^{\top} R u_{t}) $$

*   **Feasibility:** If the system is stabilizable (meaning there exists a linear feedback $K$ such that $A+BK$ has eigenvalues inside the unit circle), then there is an input sequence that yields a finite total cost[cite: 5].

### Algebraic Riccati Equation (ARE)
As $T \to \infty$ (or equivalently, iterating backward to $t \to -\infty$), the sequence of matrices $P_{t}$ converges to a constant positive definite matrix $P_{\infty}$[cite: 5]. 

$P_{\infty}$ is the solution to the **Algebraic Riccati Equation (ARE)**:

$$
P_{\infty} = Q + A^{\top} P_{\infty} A - A^{\top} P_{\infty} B (R + B^{\top} P_{\infty} B)^{-1} B^{\top} P_{\infty} A
$$

### Infinite Horizon Optimal Feedback
The optimal input becomes a **time-invariant** feedback law:

$$
u_{t} = \Gamma_{\infty} x_{t}
$$

Where the static gain $\Gamma_{\infty}$ is:

$$
\Gamma_{\infty} = - (R + B^{\top} P_{\infty} B)^{-1} B^{\top} P_{\infty} A
$$

*   **Advantages:** This approach requires much less memory for online implementation (storing only one $m \times n$ matrix instead of $T$ matrices) and allows the use of stability analysis tools from LTI system theory[cite: 5].

### Stability of the Optimal Controller
Does the optimal infinite-horizon feedback actually stabilize the system?[cite: 5]
*   **Theorem:** Let $Q = C^{\top} C$. The optimal infinite-horizon feedback stabilizes the system if and only if the pair $(A, C)$ does not have unobservable unstable modes[cite: 5]. 
*   **Practical takeaway:** If the system has unstable dynamics, those unstable states must be weighted (penalized) in the cost function $Q$ to ensure the controller actively stabilizes them[cite: 5].
