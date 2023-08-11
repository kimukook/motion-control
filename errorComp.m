function e = errorComp(xk, pd, R)
e = R * (xk(1:2) - pd);
end
