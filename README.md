# pid-goal-controller
A PID controller to navigate a robot to some goal coordinates using simiam. Implemented as part of a Georgia Tech Mobile Robotics course project.

Simiam is a MATLAB-based educational bridge between theory and practice in robotics created at Georgia Tech GRITS Lab.

<h4>1. Goal Heading</h4>
Goal heading, theta_g, should increase as the robot travels straight at its current heading in the absence of a goal controller.
<br />
<img src="https://github.com/Salman-H/pid-goal-controller/blob/master/figures/Inked1_theta_g_test_sim_LI.jpg" alt="" width="500">
<br />
<img src="https://github.com/Salman-H/pid-goal-controller/blob/master/figures/Inked1_theta_g_test_data_LI.jpg" alt="" width="500">

<h4>2. Goal Controller</h4>
A PID controller steers the robot to the goal heading, theta_g, and the robot stops at the designated goal coordinates.
<br />
<img src="https://github.com/Salman-H/pid-goal-controller/blob/master/figures/Inked3_pid_arrived_at_goal_sim_LI.jpg" alt="" width="500">
<br />
The PID gains are tuned to allow a faster settle time for the system's natural response which is marginally stable.
<br />
<img src="https://github.com/Salman-H/pid-goal-controller/blob/master/figures/Inked3_pid_settle_time_graph_LI.jpg" alt="" width="500">
<br />
The robot stops within +/- 0.05 m of the goal coordinates.
<br />
<img src="https://https://github.com/Salman-H/pid-goal-controller/blob/master/figures/Inked3_goal_final_pose_data_LI.jpg" alt="" width="500">
<br />

<h4>3. Ensuring Angular Velocity</h4>
In the case where the controller computes a linear velocity, v, and an angular velocity, w, large enough such that the robot's right and left-wheel
angular velocities exceed their maximum physical limits, then v is scaled back to ensure that w is achieved. This is because, for now, w 
is given more priority for the behavoirs of steering to goal locations and to avoid obstacles with agility. So, in situations where w cannot 
be achieved by the motors, v is sacrificed until sufficient headroom is acquired to achieve w.
<br />
To test this functionality, v is set to a large value such that the right and left-wheel velocities exceed their physical limits, and then 
these are scaled back as described above and then w recomputed as w_limited. This w_limited is then compared to w to ensure that the two are equal
and that w is indeed being guaranteed.
<br />
<img src="https://github.com/Salman-H/pid-goal-controller/blob/master/figures/Inked4_ensure_w_data_LI.jpg" alt="" width="500">

