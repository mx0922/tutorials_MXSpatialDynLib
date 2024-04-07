function R = mx_Rotx_3D(q)

c = cos(q);
s = sin(q);

R = [1, 0,  0;
     0, c, -s;
     0, s,  c];

end