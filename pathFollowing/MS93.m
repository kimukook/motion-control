function u = MS93(qx, qy, x, pd_dot, pd_ddot, psi_p, ud, uddot, k1, k2)
% find the reference point, i.e. the nearest point on the path to the vehicle
[pd, idx] = findReferencePoint(qx, qy, x);

% compute the curvature of the path at the reference point
kappa = curvatureComp(pd_dot(idx, :), pd_ddot(idx, :)); 

% rotation matrix, eqn (15)
R = rotationMatrix(psi_p(idx));

% compute y1 via, eqn (14)
e = errorComp(x, pd, R);

% compute psi_e via eqn (20)
psi_e = x(3) - psi_p(idx);

[delta, delta_dot] = delta_func(e(2), ud(idx), uddot(idx), psi_e);
% compute psi_tilde, eqn(26)
psi_tilde = psi_e - delta;

% second, computed by eqn(28)
upk = ud(idx) * cos(psi_e) / (1-kappa*e(2));

% compute r via (30)
r = kappa*upk + delta_dot - k1*psi_e - k2*e(2)*ud*(sin(psi_e)-sin(delta))/psi_tilde;

u = [ud(idx); r];
end

function [pd, idx] = findReferencePoint(qx, qy, x)
% find the reference point, defined as the nearest point on the path (qx,
% qy) to the vehicle's current position x;
dis = sum((cat(2, qx, qy) - x(1:2)').^2, 2);
[~, idx] = min(dis);
pd = [qx(idx); qy(idx)];
end

function kappa = curvatureComp(pd_dot_idx, pd_ddot_idx)
kappa = (pd_dot_idx(1) * pd_ddot_idx(2) - pd_ddot_idx(1) * pd_dot_idx(2)) / norm(pd_dot_idx).^3;
end

function R = rotationMatrix(psi_p)
R = [cos(psi_p), sin(psi_p);
    -sin(psi_p), cos(psi_p)];
end

function e = errorComp(xk, pd, R)
e = R * (xk(1:2) - pd);
end

function [delta, delta_dot] = delta_func(y1, u, udot, psi_e)
% design function used to shape the manner in which the vehicle approaches
% the path; eqn(31) of the reference:
% Hung, Nguyen, et al. "A review of path following control strategies for autonomous robotic vehicles: Theory, simulations, and experiments." Journal of Field Robotics 40.3 (2023): 747-779.
theta = pi/4; % theta \in (0, pi);
k_delta = 1; % k_delta > 0;
delta = -theta * tanh(k_delta * y1 * u);

% d tanh(x) / dx = 1 - tanh^2(x);
y1dot = u*sin(psi_e);
delta_dot = -theta * (1 - delta^2) * k_delta * (u*y1dot+y1*udot);
end
