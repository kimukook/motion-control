function [pd_dot, pd_ddot, ud, uddot] = derivativeComp(qx, qy, t)
pd_dot = [gradient(qx, t), gradient(qy, t)];
pd_ddot = [gradient(pd_dot(:, 1), t), gradient(pd_dot(:, 2), t)];

ud = sqrt(sum(pd_dot.^2, 2));
uddot = (pd_dot(:, 1).*pd_ddot(:, 1) + pd_dot(:, 2).*pd_ddot(:, 2)) ./ ud;
end
