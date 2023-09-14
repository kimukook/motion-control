% unicycle_test
clear, clc, close all;

Nt = 200; T = 2; 
Nx = 51; Ny = Nx; L = 10; 
% To modify the configuration, change lines above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = T/Nt; t = 0:dt:T-dt;
mu = [Lx/2; Lx/2];

% initialize the reference trajectory, cassini ovals
M = 1;
[qx, qy, psi_p] = initial_orbits(M, T, Nt, dt, mu);

% initialize the starting point for the sensor vehicle
x = [qx(1); qy(1); pi/2];

% ud: desired longitudinal speed profile
[pd_dot, pd_ddot, ud, uddot] = derivativeComp(qx, qy, t);

k1 = 1;
k2 = 1;

fig = figure(1);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);

%%%%%%%%%%%%
Nsteps = 425; % a user-tuning parameter, change it such that the end of the trajectory is closer to the full path
%%%%%%%%%%%%

for k = 1 : Nsteps
    u(:, k) = MS93(qx, qy, x(:, k), pd_dot, pd_ddot, psi_p, ud, uddot, k1, k2);
    
    % time marching the trajectory with solved control inputs
    func = @(x) RHS(x, u(:, k));
    x(:, k+1) = rk45(func, x(:, k), dt);

    % save figures
    subplot(2, 3, [1 2 4 5])
    plot(qx, qy, 'r'), hold on; grid on;
    plot(x(1, 1:k), x(2, 1:k), '--b');
    xlabel('$x$', 'Interpreter', 'latex')
    ylabel('$y$', 'Interpreter', 'latex')
    legend('reference', 'unicycle trajectory')

    subplot(2, 3, 3)
    plot(u(1, 1:k), 'b'), grid on; hold on;
    xlabel('$t$', 'Interpreter', 'latex')
    ylabel('$v$', 'Interpreter', 'latex')
    legend('linear velocity')

    subplot(2, 3, 6)
    plot(u(2, 1:k)), grid on;
    xlabel('$t$', 'Interpreter', 'latex')
    ylabel('$r$', 'Interpreter', 'latex')
    legend('heading rate')

    drawnow

%     file_name = sprintf('images/pf%d.png', k);
%     saveas(fig, file_name, 'png');
%     clf(1)
    pause(.5)
end

