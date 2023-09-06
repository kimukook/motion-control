function u = JN97(idx, qx, qy, ud, psi_p, omega_r, x, )
% under construction
u = zeros(2, 1);
vr = ud(idx);
xe = error_dynamics_matrix(x(3)) * [qx(idx)-x(1); qy(idx)-x(2); psi_p(idx)-x(3)];
u(1) = vr * cos(xe(3)) + c1*xe(1);
u(2) = (1+)
end
