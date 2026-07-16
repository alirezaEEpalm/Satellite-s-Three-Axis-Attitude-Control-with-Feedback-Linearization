\# Satellite Attitude Control using Feedback Linearization



This repository contains a MATLAB implementation and supporting documentation for the design of a nonlinear feedback linearization controller for satellite attitude stabilization using quaternion kinematics.



\## Project Overview

The objective of this project is to design a robust attitude controller for a rigid-body satellite using feedback linearization. The method cancels the nonlinear rotational dynamics and introduces a linear closed-loop error system with second-order behavior.



The controller is developed for:

\- Nonlinear rigid-body rotational dynamics

\- Quaternion-based attitude representation

\- Disturbance rejection

\- Actuator saturation effects



\## Mathematical Model



\### 1. Rigid-Body Dynamics

The satellite rotational motion is modeled by Euler’s equation:



$$J \\dot{\\omega} = -\\omega^\\times J \\omega + u + \\tau\_{dist}$$



where:

\- $J$ is the inertia matrix

\- $\\omega$ is the body angular velocity vector

\- u is the control torque

\- $\\tau\_{dist}$ is the disturbance torque



\### 2. Quaternion Kinematics

The attitude is represented using a unit quaternion:



$$q = \[q\_0 \\; q\_v]^T$$



with quaternion error defined as:



$$q\_e = q\_d^{-1} \\otimes q$$



where $q\_d$ is the desired quaternion.



The attitude control objective is:



$$q\_{ev} \\to 0, \\qquad \\omega \\to 0$$



\## Feedback Linearization Design



The output is chosen as the vector part of the attitude error quaternion:



$$y = q\_{ev}$$



The nonlinear dynamics are transformed into a linear input-output form:



$$\\ddot{y} = \\alpha + \\beta u + d$$



where:

\- $\\alpha$ contains the nonlinear terms

\- $\\beta$ is the input gain matrix

\- $d$ represents the disturbance term



The virtual control law is selected as a PD-like second-order law:



$$v = -K\_d \\dot{q}\_{ev} - K\_p q\_{ev}$$



with controller gains defined from the desired natural frequency and damping ratio:



$$K\_p = \\omega\_n^2 I$$



$$K\_d = 2 \\zeta \\omega\_n$$ I



Then the actual torque command is computed as:



$$u = \\beta^{-1}(v - \\alpha - d)$$



\## Simulation Features

The MATLAB script includes:

\- quaternion normalization

\- rigid-body attitude propagation

\- disturbance torque modeling

\- control torque saturation

\- time-domain simulation of attitude stabilization

\- plotting of attitude error, angular velocity, control torque, and disturbance torque



\## Files Included

\- `mfile\_FBLin.m`  

&#x20; Main MATLAB simulation script for feedback linearization control design.

\- `README.md`  

&#x20; Project README structure and formatting example.



\## Simulation Parameters

Typical simulation settings used in the code include:

\- Simulation time: 200 s

\- Integration step: 0.1 s

\- Torque saturation: \\pm 5 N.m

\- Disturbance torques: constant plus sinusoidal components



\## Figures

If you include the generated plots in the repository, use the following format:



\## Simulation Results



\### Feedback linearization results with saturation

!\[Feedback linearization results with saturation](Figures/FBLin\_results\_example2.png)



\### Feedback linearization results with additional example

!\[Feedback linearization results with additional example](Figures/FBLin\_results\_example3.png)



\### Feedback linearization results without saturation

!\[Feedback linearization results without saturation](Figures/FBLin\_results\_no\_saturation.png)



\## Usage

1\. Open `mfile\_FBLin.m` in MATLAB.

2\. Modify controller gains if needed:

&#x20;  - natural frequency \\omega\_n

&#x20;  - damping ratio \\zeta

3\. Run the script to generate the simulation results.

4\. Review the output plots for attitude stabilization and control effort.



\## Notes

\- The attitude is represented using quaternions to avoid singularities.

\- The controller uses feedback linearization to cancel nonlinear rotational coupling.

\- Torque saturation may affect tracking performance but improves practical implementability.

\- Disturbance rejection is evaluated under constant and sinusoidal perturbations.



\## Requirements

\- MATLAB

\- Basic familiarity with quaternion kinematics and nonlinear control



\---

Prepared for a satellite attitude control project using nonlinear feedback linearization.



