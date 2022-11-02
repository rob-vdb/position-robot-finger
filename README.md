# Positioning-Robotic-Finger

Using an inverse kinematics solver to position a robotic finger. 

## Introduction

This simple project considers a robotic hand that is identcal to a human hand. The hand excludes a thumb, and thus has only 4 fingers. Each finger has 3 phalanges ("finger bones"): a proximal, intermediate and distal phalanx.
1. The proximal phalanx is attached to the the metacarpal in the palm of the hand. 
2. The intermediate phalanx is the finger bone situated in the middle of the finger. 
3. The distal phalanx is the finger bone furthest from the palm of the hand.

Furthermore, each finger has three 1 degree of freedom joints: a metacarpo-phalangeal, proximal inter- phalangeal and distal inter-phalangeal joint.
1. The metacarpo-phalangeal joint is the joint that connects the proximal phalanx to the metacarpal in the palm of the hand. We assume that this joint is situated at the origin of the coordinate frame.
2. The proximal inter-phalangeal joint connects the proximal phalanx to the intermediate phalanx.
3. The distal inter-phalangeal joint connects the intermediate phalanx to the distal phalanx.

This human hand exists in a space where there is an object O given by:
O = {{x, y, z} ∈ $R^3$|y + 18 ≤ 0}

Which suggests that O is a wall like surface situated at y = −18mm. The goal of the current study is to place a fingertip of the hand in contact with the object O. In order to do is, we need to consider the length of the phalanges (links), as well as the joint angles necessary to place a fingertip on O. As a result, we will define 3 joint angles for each finger:
1. θM : the metacarpo-phalangeal angle is the counterclockwise angle between the proximal phalanx and the x-axis .
2. θP : the proximal inter-phalangeal angle is the counterclockwise angle between the intermediate phalanx and the proximal phalanx.
3. θD : the distal inter-phalangeal angle is the counterclockwise angle between the distal phalanx and the intermediate phalanx.

In addition, we will also define 3 phalanx lengths. To simplify, we will only consider the index finger in this report. The average lengths of the phalanges of a human index finger are:

1. Proximal phalanx: lP = 39.8mm
2. Intermediate phalanx: lI = 22.4mm 3. Distal phalanx: lD = 15.8mm
We consider 2 scenarios, 1) When Joint Angles Are Unconstrained, and 2) When Joint Angles Are Con- strained.
