# Motion control for unicycle model
 This is a repo containing controller for motion control of unicycle model. The motion control is consisted of path following and trajectory tracking, whereas the former could be viewed as a subproblem of the latter. The task of path following is to follow a path parameterized either by arclength or time, while it is not required to track the path with a  timing law as specified in trajectory tracking problem.
## how to use
copy the code and run `path_following_test.m` or `trajectory_tracking_test.m`.

You probably don't have the `rk45.m` file which exists in my local machine. Either substitute it with MATLAB built-in command `ode45` or contact the user directly.

## Credit
### path following
* Hung, Nguyen, et al. "A review of path following control strategies for autonomous robotic vehicles: Theory, simulations, and experiments." Journal of Field Robotics 40.3 (2023): 747-779.
* Micaelli, Alain, and Claude Samson. Trajectory tracking for unicycle-type and two-steering-wheels mobile robots. Diss. INRIA, 1993.
### trajectory tracking
* Yu, Xiao, Lu Liu, and Gang Feng. "Trajectory tracking for nonholonomic vehicles with velocity constraints." IFAC-PapersOnLine 48.11 (2015): 918-923.
