function u = YLF15(idx, qx, qy, ud, psi_p, omega_r, x, c1, c2, c3)
% A trajectory tracking controller with velocity constraints. For more
% details, please refer to,
% Xiao Yu et al. "Trajectory Tracking for Nonholonomic Vehicles
% with Velocity Constraints", IFAC 2015.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author       :    Muhan Zhao
% Institute    :    Mechanical and Aerospace Engineering, UC San Diego
% Date         :    Sep. 6, 2023
u = zeros(2, 1);
if nargin < 8
    c1 = 100;
    c2 = 50;
    c3 = 100;
end
vr = ud(idx);
% compute the error dynamics in the Frenet frame of vehicle
xe = error_dynamics_matrix(x(3)) * [qx(idx)-x(1); qy(idx)-x(2); psi_p(idx)-x(3)];
s = sqrt(1 + xe(1)^2 + xe(2)^2);
% eqn (10)
u(1) = vr + c1*xe(1)/s;
% eqn (11)
u(2) = omega_r(idx) + c2*vr*(xe(2)*cos(xe(3)/2)-xe(1)*sin(xe(3)/2))/s + c3*sin(xe(3)/2);
end

function M = error_dynamics_matrix(theta)
M = [ cos(theta), sin(theta), 0;
     -sin(theta), cos(theta), 0;
               0,          0, 1];
end
