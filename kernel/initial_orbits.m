function [qx, qy, psi_p] = initial_orbits(M, T, Nt, dt, mu)
if M > 2
    error('I am not interested in the case which has agents more than 2. ')
end
% initialize qx and qy as concentric cassini ovals
M = 1;
qx = zeros(Nt, M); qy = zeros(Nt, M);
a_cassini = ones(1, 2); b_cassini = [1.005, 1.5];

% x0 = solveGeoHeuInitCond('cassini', mu, 2, a_cassini, b_cassini);
omega = 2*pi/(dt*Nt); theta = (0:dt:T-dt)' * omega;
for kk = 1 : M
    b1 = -2 * a_cassini(kk)^2 * cos(2*theta);
    c1 = a_cassini(kk)^4 - b_cassini(kk)^4;
    r1 = sqrt((-b1+sqrt(b1.^2-4*c1))/2);
    qx(:, kk) = r1 .* cos(theta) + mu(1);
    qy(:, kk) = r1 .* sin(theta) + mu(2);
end
t = 0:dt:T-dt;
gx = gradient(qx, t);
gy = gradient(qy, t);

psi_p = atan2(gy, gx);
psi_p = angle_convert(psi_p);
end
