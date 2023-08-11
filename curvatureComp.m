function kappa = curvatureComp(pd_dot_idx, pd_ddot_idx)
kappa = (pd_dot_idx(1) * pd_ddot_idx(2) - pd_ddot_idx(1) * pd_dot_idx(2)) / norm(pd_dot_idx).^3;
end
