function [pd, idx] = findReferencePoint(qx, qy, x)
% find the reference point, defined as the nearest point on the path (qx,
% qy) to the vehicle's current position x;
dis = sum((cat(2, qx, qy) - x(1:2)').^2, 2);
[~, idx] = min(dis);
pd = [qx(idx); qy(idx)];
end
