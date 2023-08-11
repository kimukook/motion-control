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
