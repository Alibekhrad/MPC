# Convex Optimization and KKT Conditions
**Source:** Computational Control Course (Saverio Bolognani, ETH Zurich)

This document provides a detailed summary of the concepts starting from page 19 of the introductory slides.

## 1. Convexity Basics (Pages 19-21)

### Convex Sets
A set $\mathcal{X}$ is convex if the line segment connecting any two points within the set also lies entirely within the set. Mathematically, if $x', x'' \in \mathcal{X}$, then for all $0 \le t \le 1$:

$$
tx' + (1-t)x'' \in \mathcal{X}
$$

### Convex Functions
A function is convex if the line segment between any two points on its graph lies above or on the graph itself. For all $0 \le t \le 1$:

$$
\phi(tx' + (1-t)x'') \le t\phi(x') + (1-t)\phi(x'')
$$

### Relationship Between Convex Functions and Sets
If a function $g(x)$ is convex, then its sublevel set $\{x \mid g(x) \le c\}$ is also a convex set. 

**Proof:** If $x', x'' \in \{x \mid g(x) \le c\}$, we must prove $g(tx' + (1-t)x'') \le c$. Because of the convexity of $g(\cdot)$:

$$
g(tx' + (1-t)x'') \le t g(x') + (1-t) g(x'') \le tc + (1-t)c = c
$$

**Important Note:** The converse is not true. A set being convex does not mean the function generating it is convex. Counterexamples include $g(x) = -e^{-x}$ or $g(x) = \sin(x) + x$.

---

## 2. Common Convex Sets and Functions (Pages 22-25)

### Common Convex Sets
* **Halfspaces:** $\{x \mid a^{\top} x \le b\}$
* **Balls:** $\{x \mid \Vert x - c \Vert < r\}$
* **Polytopes:** Defined as the intersection of multiple hyperplanes. Represented as $\{x \mid Ax \le b\}$ or $\{x \mid a_{i}^{\top} x \le b \quad \forall i\}$.

### Supporting Hyperplanes
**Theorem:** If a set is convex, there exists a supporting hyperplane at every point $z$ on its boundary. If the set is defined as $\{x \mid g(x) \le 0\}$, the supporting hyperplane at $z$ is:

$$
\nabla g(z)^{\top} (x - z) \le 0
$$

Which can also be written as:

$$
\nabla g(z)^{\top} x \le \nabla g(z)^{\top} z
$$

### Common Convex Functions
* **Affine functions:** $f(x) = a^{\top} x + b$
* **Norms:** $f(x) = \Vert x - c \Vert$
* **Quadratic functions:** $f(x) = x^{\top} Ax + b^{\top} x$ (requires $A \succeq 0$, meaning $A$ must be positive semi-definite).

---

## 3. Optimality Conditions (Pages 26-29)

### Derivatives and Convexity
**First-order condition:** If $f$ is differentiable on a convex domain, $f$ is convex if and only if:

$$
f(y) \ge f(x) + \nabla f(x)^{\top} (y - x) \quad \forall x, y
$$

**Second-order condition:** If $f$ is twice differentiable on a convex domain, $f$ is convex if and only if its Hessian is positive semi-definite:

$$
\nabla^{2} f(x) \succeq 0 \quad \forall x
$$

### Convex Optimization Problem Formulation
The core decision problem is formulated as:

$$
\begin{aligned}
\min_{x \in \mathcal{X}} \quad & f(x) \\
\text{subject to} \quad & g(x) \le 0 \\
& h(x) = 0
\end{aligned}
$$

Where $\mathcal{X}$ is a convex set, $f$ and $g$ are convex functions, and $h$ is an affine function. The resulting convex feasible set is $\mathcal{X} \cap \{x \mid g(x) \le 0\} \cap \{x \mid h(x) = 0\}$.

### Local vs. Global Optimality
**Proposition:** Any local optimum of a convex problem is a global optimum.

**Proof:** Suppose $x^{\star}$ is locally optimal, but there exists a $y$ such that $f(y) < f(x^{\star})$. Let $z = ty + (1-t)x^{\star}$ with $t = \frac{R}{2 \Vert y - x^{\star} \Vert}$. By convexity, $z$ is feasible and inside a ball of radius $R$. However:

$$
f(z) \le t f(y) + (1-t) f(x^{\star}) < t f(x^{\star}) + (1-t) f(x^{\star}) = f(x^{\star})
$$

Which contradicts the local optimality assumption.

### Local Optimality Conditions (Differentiable Case)
**Proposition:** $x^{\star}$ is a local optimum for a differentiable problem if and only if:

$$
\nabla f(x^{\star})^{\top} (y - x^{\star}) \ge 0 \quad \text{for all feasible } y
$$

If the problem is **unconstrained**, this simply reduces to:

$$
\nabla f(x^{\star}) = 0
$$

---

## 4. Constrained Case and KKT Conditions (Pages 30-32)

When constraints are present (e.g., $g(x) \le 0$), the unconstrained optimum $\nabla f(x^{\star}) = 0$ might lie outside the feasible set. To find the optimal solution on the boundary, we use the **Karush-Kuhn-Tucker (KKT)** conditions.

There exist multipliers (dual variables) $\mu$ such that the following system of equations and inequalities holds:

$$
\begin{cases} 
\nabla f(x) + \mu^{\top} \nabla g(x) = 0 \\ 
\mu \ge 0 \\ 
g(x) \le 0 \\ 
\mu_{i} g_{i}(x) = 0 \quad \forall i 
\end{cases}
$$

### Breakdown of the KKT Conditions:

**1. Stationarity:** 

$$
\nabla f(x) + \mu^{\top} \nabla g(x) = 0 
$$

**2. Dual Feasibility:** 

$$
\mu \ge 0 
$$

**3. Primal Feasibility:** 

$$
g(x) \le 0 
$$

**4. Complementary Slackness:** 

$$
\mu_{i} g_{i}(x) = 0 \quad \forall i 
$$

### Solving Convex Optimization Problems
Because these problems are structured, they are much simpler to tackle computationally. They can be solved using:
* Iterative gradient-based algorithms
* Interior point algorithms
* Finding the saddle-point of the **Lagrangian** function (which directly solves the KKT conditions).

Ultimately, solving these comes down to linear algebra and zero-finding, making them computationally efficient.
