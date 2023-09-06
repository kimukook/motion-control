function u = YLF15(idx, qx, qy, ud, psi_p, omega_r, x, c1, c2, c3)
u = zeros(2, 1);
if nargin < 8
    c1 = 100;
    c2 = 50;
    c3 = 100;
end
vr = ud(idx);
xe = error_dynamics_matrix(x(3)) * [qx(idx)-x(1); qy(idx)-x(2); psi_p(idx)-x(3)];
s = sqrt(1 + xe(1)^2 + xe(2)^2);
u(1) = vr + c1*xe(1)/s;
u(2) = omega_r(idx) + c2*vr*(xe(2)*cos(xe(3)/2)-xe(1)*sin(xe(3)/2))/s + c3*sin(xe(3)/2);
end
