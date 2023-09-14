function dx = RHS(x, u)
dx = zeros(3, 1);
dx(1) = u(1) * cos(x(3));
dx(2) = u(1) * sin(x(3));
dx(3) = u(2);
end
