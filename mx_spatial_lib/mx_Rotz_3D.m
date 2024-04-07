function R = mx_Rotz_3D(q)

c = cos(q);
s = sin(q);

R = [c, -s, 0;
     s,  c, 0;
     0,  0, 1];

end