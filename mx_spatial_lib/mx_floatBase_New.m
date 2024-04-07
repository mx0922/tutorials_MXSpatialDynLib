function  fbmodel = mx_floatBase_New(model)

if any( model.parent(2:model.NB) == 0 )
  error('Only one connection to a fixed base allowed!!!');
end

if ~isequal( model.Xtree{1}, Xtrans([0,0,0]) )
  warning('Xtree{1} not identity');
end

% ========== fbmodel field: NB, pitch, parent, Xtree, I ==========

% body +5
fbmodel.NB = model.NB + 5;

% pitch
% floating base order: x, y, z, roll, pitch, yaw
fbmodel.pitch(1:6) = [inf,inf,inf,0,0,0];
fbmodel.pitch(7:fbmodel.NB) = model.pitch(2:model.NB);

% parent
fbmodel.parent(1:6) = [0 1 2 3 4 5];
fbmodel.parent(7:fbmodel.NB) = model.parent(2:model.NB) + 5;

%% Xtree
fbmodel.Xtree{1} = Xroty(pi/2);                   
fbmodel.Xtree{2} = Xrotx(-pi/2) * Xroty(-pi/2);   
fbmodel.Xtree{3} = Xrotx(pi/2);                   

fbmodel.Xtree{4} = Xroty(pi/2);
fbmodel.Xtree{5} = Xrotx(-pi/2) * Xroty(-pi/2);
fbmodel.Xtree{6} = Xrotx(pi/2);

for i = 7:fbmodel.NB  
  fbmodel.Xtree{i} = model.Xtree{i-5};
end

%% I
for i = 1:fbmodel.NB
  if i < 6
      % virtual dofs
      fbmodel.I{i} = mcI( 0, [0,0,0], zeros(3) );
  else      
      fbmodel.I{i} = model.I{i-5};
  end
end

disp('-->The robot model have been transferred to spatial floating base model.');

end