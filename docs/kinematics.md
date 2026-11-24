# Kinematic model of the SCARA arm

This note derives every equation used in the simulator. The code that implements each part is named at the end of its section.

## 1. Robot and joint variables

The SCARA (Selective Compliance Assembly Robot Arm) is a 4-DOF **RRPR** manipulator:

| Joint | Type | Variable | Motion |
|:-----:|:----:|:--------:|--------|
| 1 | Revolute  | θ₁ | shoulder rotation about the vertical base axis |
| 2 | Revolute  | θ₂ | elbow rotation about a parallel vertical axis |
| 3 | Prismatic | d₃ | vertical travel of the quill (positive = down) |
| 4 | Revolute  | θ₄ | roll of the tool about the quill axis |

Geometric constants: column height **d₁**, link lengths **a₁** and **a₂**, and tool offset **d₄**.

## 2. Denavit–Hartenberg convention

The simulator uses the *standard* (distal) DH convention. Each link transform is

$$
A_i = \mathrm{Rot}_z(\theta_i)\,\mathrm{Trans}_z(d_i)\,\mathrm{Trans}_x(a_i)\,\mathrm{Rot}_x(\alpha_i)
=
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Code: `src/dh_transform.m`

## 3. DH parameter table

| Link | θᵢ | dᵢ | aᵢ | αᵢ |
|:----:|:--:|:--:|:--:|:--:|
| 1 | θ₁\* | d₁ | a₁ | 0 |
| 2 | θ₂\* | 0 | a₂ | π |
| 3 | 0 | d₃\* | 0 | 0 |
| 4 | θ₄\* | d₄ | 0 | 0 |

\* joint variable

The twist α₂ = π flips z₂ so that it points **down**. A positive d₃ therefore lowers the quill, as on an industrial SCARA.

Code: `src/scara_dh_table.m`

## 4. Forward kinematics

Using the shorthand cᵢ = cos θᵢ, sᵢ = sin θᵢ and c₁₂ = cos(θ₁ + θ₂):

$$
A_1 = \begin{bmatrix} c_1 & -s_1 & 0 & a_1c_1 \\ s_1 & c_1 & 0 & a_1s_1 \\ 0&0&1&d_1 \\ 0&0&0&1 \end{bmatrix},\quad
A_2 = \begin{bmatrix} c_2 & s_2 & 0 & a_2c_2 \\ s_2 & -c_2 & 0 & a_2s_2 \\ 0&0&-1&0 \\ 0&0&0&1 \end{bmatrix},
$$

$$
A_3 = \begin{bmatrix} 1&0&0&0 \\ 0&1&0&0 \\ 0&0&1&d_3 \\ 0&0&0&1 \end{bmatrix},\quad
A_4 = \begin{bmatrix} c_4 & -s_4 & 0 & 0 \\ s_4 & c_4 & 0 & 0 \\ 0&0&1&d_4 \\ 0&0&0&1 \end{bmatrix}
$$

Their product is the base-to-tool transform

$$
{}^0T_4 = A_1A_2A_3A_4 =
\begin{bmatrix}
\cos\varphi & \sin\varphi & 0 & a_1c_1 + a_2c_{12} \\
\sin\varphi & -\cos\varphi & 0 & a_1s_1 + a_2s_{12} \\
0 & 0 & -1 & d_1 - d_3 - d_4 \\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad \varphi = \theta_1 + \theta_2 - \theta_4
$$

So the tool tip is at

$$
x = a_1\cos\theta_1 + a_2\cos(\theta_1+\theta_2),\quad
y = a_1\sin\theta_1 + a_2\sin(\theta_1+\theta_2),\quad
z = d_1 - d_3 - d_4
$$

The tool z-axis always points straight down, and the tool yaw is φ = θ₁ + θ₂ − θ₄. The minus sign on θ₄ appears because z₃ points down.

Code: `src/scara_fk.m`. The unit tests check this closed form against the numeric DH product for 200 random configurations.

## 5. Inverse kinematics

Given a target pose (x, y, z, φ):

$$
\cos\theta_2 = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2},\qquad
\theta_2 = \operatorname{atan2}\!\left(\pm\sqrt{1-\cos^2\theta_2},\ \cos\theta_2\right)
$$

$$
\theta_1 = \operatorname{atan2}(y, x) - \operatorname{atan2}(a_2\sin\theta_2,\ a_1 + a_2\cos\theta_2)
$$

$$
d_3 = d_1 - d_4 - z,\qquad \theta_4 = \theta_1 + \theta_2 - \varphi
$$

The ± sign picks one of the two arm configurations (θ₂ > 0 or θ₂ < 0). The target is out of reach when |cos θ₂| > 1.

Code: `src/scara_ik.m`

## 6. Jacobian, manipulability and singularities

Column *i* of the geometric Jacobian comes from the DH frames:

- Revolute joint: $J_i = \begin{bmatrix} z_{i-1}\times(o_4 - o_{i-1}) \\ z_{i-1}\end{bmatrix}$
- Prismatic joint: $J_i = \begin{bmatrix} z_{i-1} \\ 0\end{bmatrix}$

For the SCARA this gives

$$
J = \begin{bmatrix}
-a_1s_1 - a_2s_{12} & -a_2s_{12} & 0 & 0 \\
a_1c_1 + a_2c_{12} & a_2c_{12} & 0 & 0 \\
0 & 0 & -1 & 0 \\
0&0&0&0 \\ 0&0&0&0 \\
1 & 1 & 0 & -1
\end{bmatrix}
$$

Yoshikawa's manipulability of the translational part is

$$
w = \sqrt{\det(J_vJ_v^{\mathsf T})} = a_1a_2\,|\sin\theta_2|
$$

It is zero at the singular configurations θ₂ = 0 (arm fully stretched, on the outer workspace boundary) and θ₂ = ±π (arm folded back). It is largest at θ₂ = ±90°.

Code: `src/scara_jacobian.m`

## 7. Workspace analysis

### Reach

The horizontal distance from the base axis to the quill axis depends on θ₂ only:

$$
r(\theta_2) = \sqrt{a_1^2 + a_2^2 + 2a_1a_2\cos\theta_2}
$$

This decreases monotonically in |θ₂|. So r_max is reached at the allowed θ₂ closest to 0, and r_min at the largest allowed |θ₂|.

Code: `src/scara_reach.m`

### Planar area

In polar coordinates, with t = |θ₂|, we have $r\,dr = a_1a_2\sin t\,dt$. At radius r(t), each allowed elbow branch (+t and/or −t) covers an arc of length L = θ₁,max − θ₁,min. The two arcs are rotated against each other by 2ψ(t), where ψ = atan2(a₂ sin t, a₁ + a₂ cos t). With U(t) the angular length of the union of the allowed arcs:

$$
A = \int_0^{\pi} U(t)\; a_1a_2\sin t \; dt
$$

Special cases:

- **Full rotation of both joints:** U = 2π, so A = 4π a₁a₂ = π[(a₁+a₂)² − (a₁−a₂)²], the annulus area.
- **One elbow branch, θ₂ ∈ [α, β] ⊂ (0, π):** A = L·a₁a₂(cos α − cos β).

The area is also computed independently with a dense occupancy grid (`method = 'grid'`). The unit tests require the two methods to agree within 2 %. In practice they agree within about 0.3 %.

Code: `src/scara_workspace_area.m`

### Volume

The vertical axis is decoupled from the planar motion. The reachable volume is therefore a prism:

$$
V = A \cdot (d_{3,\max} - d_{3,\min}),\qquad z \in [\,d_1 - d_4 - d_{3,\max},\ d_1 - d_4 - d_{3,\min}\,]
$$

Code: `src/scara_workspace.m`, `src/scara_plot_workspace.m`

### Envelope curves

The black curves drawn on the workspace plots are the images of the four edges of the joint-space rectangle [θ₁ limits] × [θ₂ limits], plus the singular arc θ₂ = 0. The workspace boundary is made of pieces of these curves.
