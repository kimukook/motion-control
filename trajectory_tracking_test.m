% an attempt towards trajectory tracking
% clear, clc, close all;

Nt = 128; T = 2; 
Nx = 51; Ny = Nx; L = 10; 
% To modify the configuration, change lines above
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = T/Nt; t = 0:dt:T-dt;
mu = [5/2; 5/2];

% initialize the reference trajectory, cassini ovals
M = 1;
[qx, qy, psi_p] = initial_orbits(M, T, Nt, dt, mu);

% initialize the starting point for the sensor vehicle
x = [qx(1); qy(1); psi_p(1)];

% compute the turning rate of reference point
omega_r = gradient(psi_p, t);

% ud: desired longitudinal speed profile
[pd_dot, pd_ddot, ud, uddot] = derivativeComp(qx, qy, t);


fig = figure(1);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);

for k = 1 : Nt-1
    u(:, k) = YLF15(k, qx, qy, ud, psi_p, omega_r, x(:, k));
%     u(:, k) = JN97(k, qx, qy, ud, psi_p, omega_r, x(:, k));
%     keyboard
    

    func = @(x) RHS(x, u(:, k));
    x(:, k+1) = rk45(func, x(:, k), dt);

    % error in inertial frame
    error(:, k) = [qx(k)-x(1, k); qy(k)-x(2, k); psi_p(k)-x(3, k)];

    figure(1);
    % save figures
    subplot(2, 3, [1 2 4 5])
    plot(qx, qy, 'r'), hold on; grid on;
    plot(x(1, 1:k), x(2, 1:k), '--b');
    s = scatter(qx(k), qy(k), 10, 'r', 'filled');

    xlabel('$x$', 'Interpreter', 'latex')
    ylabel('$y$', 'Interpreter', 'latex')
    legend('reference', 'unicycle trajectory')

    subplot(2, 3, 3)
    plot(u(1, 1:k), 'b'); grid on; hold on;
    
    xlabel('$t$', 'Interpreter', 'latex')
    ylabel('$v$', 'Interpreter', 'latex')
    legend('linear velocity')

    subplot(2, 3, 6)
    plot(u(2, 1:k)); grid on; hold on;
    xlabel('$t$', 'Interpreter', 'latex')
    ylabel('$r$', 'Interpreter', 'latex')
    legend('heading rate')

    drawnow

    file_name = sprintf('images/pf%d.png', k);
    saveas(fig, file_name, 'png');
    clf(1)

    figure(2);
    % error in inertial frame
    plot(error(1, 1:k), 'b'), hold on;
    plot(error(2, 1:k), 'r')
    plot(error(3, 1:k), 'g')
    legend('$x_e$', '$y_e$', '$\theta_e$', 'interpreter', 'latex');

    
%     keyboard
    pause(.5)
    delete(s);
end



