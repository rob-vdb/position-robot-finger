# Positioning-Robotic-Finger

Using Java to code an iterative inverse kinematics solver that can be used to position a robotic finger. 

## Background

This simple project considers a robotic hand that is identcal to a human hand. The hand excludes a thumb, and thus has only 4 fingers. Each finger has 3 phalanges ("finger bones"): a proximal, intermediate and distal phalanx.
1. The proximal phalanx is attached to the the metacarpal in the palm of the hand. 
2. The intermediate phalanx is the finger bone situated in the middle of the finger. 
3. The distal phalanx is the finger bone furthest from the palm of the hand.

Furthermore, each finger has three 1 degree of freedom joints: a metacarpo-phalangeal, proximal inter- phalangeal and distal inter-phalangeal joint.
1. The metacarpo-phalangeal joint is the joint that connects the proximal phalanx to the metacarpal in the palm of the hand. We assume that this joint is situated at the origin of the coordinate frame.
2. The proximal inter-phalangeal joint connects the proximal phalanx to the intermediate phalanx.
3. The distal inter-phalangeal joint connects the intermediate phalanx to the distal phalanx.

This human hand exists in a space where there is an object O given by:

$O = \\{ \\{ x, y, z \\} ∈ R^3 | y + 18 ≤ 0 \\}$

Which suggests that O is a wall like surface situated at y = −18mm. The goal of the current study is to place a fingertip of the hand in contact with the object O. In order to do is, we need to consider the length of the phalanges (links), as well as the joint angles necessary to place a fingertip on O. As a result, we will define 3 joint angles for each finger:

1. $θ_M$ : the metacarpo-phalangeal angle is the counterclockwise angle between the proximal phalanx and the x-axis .
2. $θ_P$ : the proximal inter-phalangeal angle is the counterclockwise angle between the intermediate phalanx and the proximal phalanx.
3. $θ_D$ : the distal inter-phalangeal angle is the counterclockwise angle between the distal phalanx and the intermediate phalanx.

In addition, we will also define 3 phalanx lengths. To simplify, we will only consider the index finger in this report. The average lengths of the phalanges of a human index finger are:

1. Proximal phalanx: $l_P$ = 39.8mm
2. Intermediate phalanx: $l_I$ = 22.4mm 
3. Distal phalanx: $l_D$ = 15.8mm

Furthermore, it should be noted that there are some restrictions on the range of possible joint angles for a human finger. These restrictions are: 

1. $−π/3 ≤ θ_M ≤ π/3$
2.  $−2π/3 ≤ θ_P ≤ 0$
3.  $−2π/3 ≤ θ_D ≤ 0$

In order to use this inofrmation to place the finger tip of the robottic hand on O, we first need to simplify the problem. We first begin by assuming that all joint axes are parallel with the z axis, which simplifies the problem to 2D space. Furthermore, in reality, the distal inter-phalangeal angle ( $\theta_D$ ) actually exhibits the following relationship with the proximal inter-phalangeal angle ( $\theta_P$ ):

$$ \theta_{D} = \frac{2}{3} \theta_{P} $$

Using some simple linear algebra, the following expression for the final x- and y-coordinates of the finger tip can be derived:

$$ x_{final} = l_{P} Cos ( \theta_{M} ) + l_{I} Cos(\theta_{M} + \theta_{P}) + l_{D} Cos(\theta_{M} + \frac{5}{3} \theta_{P}  ) $$

$$ y_{final} = l_{P} Sin ( \theta_{M} ) + l_{I} Sin(\theta_{M} + \theta_{P}) + l_{D} Sin(\theta_{M} + \frac{5}{3} \theta_{P}  ) $$

Since we are given the desired x- and y-coordinates, as well as the lengths of the phalanges, we now have 2 equations in 2 unkowns: $\theta_{M}$ and $\theta_{P}$. One common way that you can solve these equations for $\theta_{M}$ and $\theta_{P}$ is to use the equations for an iterative inverse kinematics solver:

$$ q^{i+1} = q^{i} + J(q^{i})^{-1} * (p - p^{i}) $$

where $q^{i}$ is a vector of initial approximations for the angles; $q^{i+1}$ is a vector of updated approximations based on $q^{i}$; $J(q^{i})^{-1}$ is the inverse of the Jacobian matrix - which is based on the initial approximations $q^{i}$; and $p^{i}$ is a vector of the $x$ and $y$ coordinates produced by the initial approximations $q^{i}$. The term $(p - p^{i})$ is defined as the error in the initial approximation. The above equation can be expanded as:

$$ \left(\begin{array}{cc} 
\theta_{M} \\ 
\theta_{P} 
\end{array}\right) ^{i+1} =
\left(\begin{array}{cc} 
\theta_{M} \\
\theta_{P}
\end{array}\right) ^{i} +
\left(\begin{array}{cc} 
\frac{dx}{d\theta_{M}} & \frac{dx}{d\theta_{P}} \\
\frac{dy}{d\theta_{M}} & \frac{dy}{d\theta_{P}} 
\end{array}\right) ^{-1} *
\left(
\left(\begin{array}{cc} 
x \\
y
\end{array}\right) -
\left(\begin{array}{cc} 
x \\
y
\end{array}\right) ^{i}
\right)
$$

It is now possible to code this iterative inverse kinematics solver. In order to do so, we need to specify the desired coordinates along the object O given by $p$. The y coordinate will be given by -18, and so we arbitrarily pick an x coordinate of 70 (the x coordinate simply needs to be within the reach of the finger - so not too close or too far from the finger). Furthermore, we also need some arbitrary initial guess for $q^{0}$. Setting $\theta_{M} = 0$ and $\theta_{P} = -1$ we have:

$$
p = 
\left(\begin{array}{cc} 
70 \\
-18
\end{array}\right)
$$

$$
q^0 = 
\left(\begin{array}{cc} 
0 \\
-1
\end{array}\right)
$$

These are the values used in the solverTest.java file. 
