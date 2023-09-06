function M = error_dynamics_matrix(theta)
M = [ cos(theta), sin(theta), 0;
     -sin(theta), cos(theta), 0;
               0,          0, 1];
end
