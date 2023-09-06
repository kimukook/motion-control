function [r, idx, upk] = compute_turning_rate(qx, qy, x, pd_dot, pd_ddot, psi_p, ud, uddot, k1, k2)
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

% how to compute up? up is the linear velocity of P w.r.t. {I}
% first one, computed from xdot and ydot
%     upk = ud(idx); % ?

% second, computed by eqn(28)
upk = ud(idx) * cos(psi_e) / (1-kappa*e(2));

% compute r via (30)
r = kappa*upk + delta_dot - k1*psi_e - k2*e(2)*ud*(sin(psi_e)-sin(delta))/psi_tilde;
    
end
