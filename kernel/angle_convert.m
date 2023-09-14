function psi_new = angle_convert(psi)
% input: psi 
% output: psi_new
% Jul. 31, 2023
psi_new = zeros(size(psi, 1), 1);
psi_new(1) = psi(1);
for i = 2 : length(psi)
    delta_psi = psi(i) - psi(i-1);
    if delta_psi < -pi
        psi_new(i) = psi_new(i-1) + 2*pi + delta_psi;
    elseif delta_psi > pi
        psi_new(i) = psi_new(i-1) + delta_psi - 2*pi;
    else
        psi_new(i) = psi_new(i-1) + delta_psi;
    end
end
end
