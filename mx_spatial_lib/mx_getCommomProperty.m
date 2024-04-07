function [S, XBase, VBody, Xup] = mx_getCommomProperty(tree, q, qd)

NB = tree.NB;
Xup = cell(1, NB);    
S = cell(1, NB);   
XBase = cell(1, NB);
VBody = cell(1, NB);

for i = 1:NB
    [XJ, S{i}] = jcalc(tree.pitch(i), q(i));
    vJ = S{i} * qd(i);
    Xup{i} = XJ * tree.Xtree{i};
    ip = tree.parent(i);
    if ip == 0
        XBase{i} = Xup{i};
        VBody{i} = vJ;
    else
        XBase{i} = Xup{i} * XBase{ip};
        VBody{i} = Xup{i} * VBody{ip} + vJ;
    end        
end

end