# Robotic Manipulation Project Course 2021/22
> This repository is a work in progress, where Python and Julia will 
> be tested to see if they are good candidates to replace 
> MATLAB/Simulink

Project of Robotic's Manipulation course of 2021/22 in Instituto 
Superior Técnico - Universidade de Lisboa by using Julia/Python as 
programming language instead of MATLAB/Simulink.

## Project plan
This project is going to follow the following plan for each section 
of it:
1. Kinematics - *Part K*
   1. Robot's DHC table
   2. Direct Kinematics
   3. Geometric Jacobian
   4. *CLIK*
   5. Inverse Kinematics
2. Dynamic - *Part D*
   1. Add $I_m$, $m$ and $\bar{c}_m$ in the DHC table
   2. Dynamics Model - *Lagrange VS Newton*
   3. Decentralized *PID*
   4. Centralized Inverse Dynamics
   5. Project a task and trajectory for it

In each step, it is necessary to make this code the most adapted to 
the signal as a time series' concept, needed to make therefore a way
to express this series in either a vector or as list of tuple of 
$(time, value)$.
Besides that, it is necessary to test it, which will be necessary to 
develop scripts for testing and, big surprise, a CI client.

## Current status
1. Kinematics - *Part K*
   - [ ] Robot's DHC table
   - [ ] Direct Kinematics
   - [ ] Geometric Jacobian
   - [ ] *CLIK*
   - [ ] Inverse Kinematics
2. Dynamic - *Part D*
   - [ ] Add $I_m$, $m$ and $\bar{c}_m$ in the DHC table
   - [ ] Dynamics Model - *Lagrange VS Newton*
   - [ ] Decentralized *PID*
   - [ ] Centralized Inverse Dynamics
   - [ ] Project a task and trajectory for it
