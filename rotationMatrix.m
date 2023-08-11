function R = rotationMatrix(psi_p)
R = [cos(psi_p), sin(psi_p);
    -sin(psi_p), cos(psi_p)];
end
