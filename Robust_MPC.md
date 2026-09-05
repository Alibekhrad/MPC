# Summary: Robust Model Predictive Control (Robust MPC)

## 1. The Need for Robustness
Standard MPC algorithms rely on the assumption of a perfect system model. In reality, models fail to capture true dynamics due to several factors:
*   **Unknown Exogenous Disturbances ($w_k$):** Unpredictable external forces acting on the system.
*   **Model Mismatch:** The mathematical model is only an approximation of the true physical plant.
*   **Missing Dynamics & Linearization:** Unmodeled hidden states ($z_k$) or errors introduced by linearizing a non-linear system.

To design a robust controller, disturbances are typically modeled either as a **finite discrete set** ($w_k \in \{w^0, \dots, w^p\}$), a **convex hull/polytope** ($w_k \in \text{co}\{w^0, \dots, w^p\}$), or a probability distribution.

## 2. Cost and Constraint Strategies under Uncertainty
When uncertainty is introduced, the optimization problem must explicitly define how to handle costs and boundaries:
*   **Cost Function:** Can be formulated to minimize the *nominal cost* (assuming zero disturbance), the *expected cost* (weighted by probabilities), or the *worst-case cost* (Min-Max approach).
*   **Constraints Handling:** 
    *   *Guaranteed Satisfaction (Hard Constraints):* Constraints must be satisfied for *all* possible disturbances (critical for safety).
    *   *Chance Constraints:* Constraints must be satisfied with a high probability (e.g., 95% of cases).

## 3. The Min-Max Formulation and Open-Loop Infeasibility
*   **Adversarial Game:** Robust MPC is often formulated as a Min-Max problem: $\min_{\mathbf{u}} \max_{\mathbf{w}}$. The controller tries to minimize the cost while an "adversary" (disturbance) tries to maximize it.
*   **The Infeasibility Trap:** Standard MPC attempts to solve this in an **open-loop** fashion, searching for a static sequence of control inputs $\mathbf{u}^\ast = \{u_0^\ast, \dots, u_{K-1}^\ast\}$ that satisfies constraints for *all* disturbance scenarios simultaneously. 
*   Because uncertainties accumulate over the horizon (the "trumpet effect"), finding a single sequence of fixed numbers that satisfies conflicting worst-case scenarios becomes mathematically **infeasible**.

## 4. The Closed-Loop Paradigm (Feedback MPC)
*   To achieve true robustness, the optimizer cannot search for static numbers; it must search for a **Control Policy** (a function of the state): $u_k = \pi_k(x_k)$.
*   **Soft-Constrained LQR ($H_\infty$ Control):** For linear systems with quadratic costs and no hard constraints, a closed-form robust policy exists. It is solved via the **Isaacs Equation** (Dynamic Programming). A negative penalty term $-\gamma^2 w_k^\top w_k$ is added to the cost function to bound the disturbance energy, making the maximization concave and solvable.

## 5. Affine Control Laws and Tube MPC Concept
To handle hard constraints while keeping the search space manageable, the control policy is parametrized. The most effective parametrization is the **Affine Control Law**:
$$u_k = v_k + Lx_k$$
*   **$v_k$ (Nominal Input):** The open-loop decision variable computed by the optimizer to steer the system along an ideal nominal trajectory.
*   **$Lx_k$ (Robust Feedback):** A fixed feedback gain (like LQR) that acts as a spring, actively rejecting disturbances and pulling the system back to the nominal trajectory, thus preventing the uncertainties from compounding over time.

## 6. Convexification via Disturbance Feedback
*   If we try to optimize both the nominal input ($v_k$) and the feedback gain ($L_k$) simultaneously, the optimization problem becomes **non-convex** (due to the multiplication of decision variables) and practically impossible to solve in real-time.
*   **The Mathematical Trick:** Because the current state $x_k$ can be perfectly reconstructed using the initial state and past disturbances, we can re-parametrize the policy as an **Affine Disturbance Feedback**:
    $$u_k = \sum_{i=0}^{k-1} M_{ki}w_i + v_k$$
*   By optimizing over the matrices $\mathbf{M}$ and vectors $\mathbf{v}$ instead of the state feedback gain $L$, the optimization problem remains completely **convex** and computationally tractable (scaling by $n \times K \times W$).
