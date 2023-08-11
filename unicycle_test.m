% unicycle_test
clear, clc, close all;

Nt = 200; T = 2; 
Nx = 51; Ny = Nx; L = 10; 
% To modify the configuration, change lines above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = T/Nt; t = 0:dt:T-dt;
mu = [5; 5];

% reference trajectory, cassini ovals
M = 1;
[qx, qy, psi_p] = initial_orbits(M, T, Nt, dt, mu);

x = [qx(1); qy(1); pi/2];

% up: desired longitudinal speed profile
[pd_dot, pd_ddot, ud, uddot] = derivativeComp(qx, qy, t);

k1 = 1;
k2 = 1;

fig = figure(1);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);

%%%%%%%%%%%%
Nsteps = 435;
%%%%%%%%%%%%

for k = 1 : Nsteps
    % find the reference point, i.e. the nearest point on the path to the vehicle
    [pd, idx] = findReferencePoint(qx, qy, x(:, k));

    % compute the curvature of the path at the reference point
    kappa = curvatureComp(pd_dot(idx, :), pd_ddot(idx, :));

    % rotation matrix, eqn (15)
    R = rotationMatrix(psi_p(idx));

    % compute y1 via, eqn (14)
    e = errorComp(x(:, k), pd, R);

    % compute psi_e via eqn (20)
    psi_e = x(3, k) - psi_p(idx);

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
    
    % time marching the trajectory with solved control inputs
    u(:, k) = [upk; r];
    func = @(x) RHS(x, u(:, k));
    x(:, k+1) = rk45(func, x(:, k), dt);

    % save figures
%     subplot(2, 3, [1 2 4 5])
%     plot(qx, qy, 'r'), hold on; grid on;
%     plot(x(1, 1:k), x(2, 1:k), '--b');
%     xlabel('$x$', 'Interpreter', 'latex')
%     ylabel('$y$', 'Interpreter', 'latex')
%     legend('reference', 'unicycle trajectory')
% 
%     subplot(2, 3, 3)
%     plot(u(1, 1:k)), grid on;
%     xlabel('$t$', 'Interpreter', 'latex')
%     ylabel('$v$', 'Interpreter', 'latex')
%     legend('linear velocity')
% 
%     subplot(2, 3, 6)
%     plot(u(2, 1:k)), grid on;
%     xlabel('$t$', 'Interpreter', 'latex')
%     ylabel('$r$', 'Interpreter', 'latex')
%     legend('heading rate')
% 
%     drawnow
% 
%     file_name = sprintf('images/pf%d.png', k);
%     saveas(fig, file_name, 'png');
%     clf(1)
end

function dx = RHS(x, u)
dx = zeros(3, 1);
dx(1) = u(1) * cos(x(3));
dx(2) = u(1) * sin(x(3));
dx(3) = u(2);
end
