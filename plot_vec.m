function plot_vec(vec, pt)
% plot the vector at the reference point
% example:

for i = 1 : size(vec, 1)
    vec_i = vec(i, :) / norm(vec(i, :));
    x = [pt(i, 1), pt(i, 1)+vec_i(1)];
    y = [pt(i, 2), pt(i, 2)+vec_i(2)];
    s = scatter(pt(i, 1), pt(i, 2), 5, 'r', 'filled');
    p = plot(x, y, 'b'); 
    pause(.5)
    delete(s); delete(p);
end

end
