function V = mx_vCross(v)

w = v(1:3);
v0 = v(4:6);

V = [mx_skew(w), zeros(3);
    mx_skew(v0), mx_skew(w)];

end