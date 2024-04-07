function model = setupRobotSpatialModel_myRobot_fixedBase()
% You can modify 'myRobot' with your robot name.

param = setupIntuitiveRobotModel();    % Users only have to update this function!

model = transferToSpatialModel(param); % trasfer the intuitive model to spatial model

end

% Please setup your robot model here!
function p = setupIntuitiveRobotModel()
% fcn: Create an intuitive and easily adjustable robot model 
%     -- different from the non-intuitive models found in URDF or SD files.

% 0.number of body -- nb
p.nb = 11; % torso + 5 bodies in each leg (one joint drives one body)
p.pitch = zeros(1, p.nb); % 0--rotate; inf--translate  --> all rotate
%           1,   2, 3, 4, 5, 6,    7, 8, 9, 10, 11   --> body number
p.parent = [0,   1, 2, 3, 4, 5,    1, 7, 8,  9, 10];
p.axis_i = [3,   3, 1, 2, 2, 2,    3, 1, 2,  2,  2]; % 1->X, 2->Y, 3->Z

% 1.joint relative postion [j] -- i-th joint frame w.r.t. p(i)-th body frame
% Note: p(i) is the parent of i.
p.jnt_i_pi = zeros(p.nb, 3);
p.jnt_i_pi(1, :) = [    0,       0,      0]; % torso -- null
% right leg
p.jnt_i_pi(2, :) = [    0,   -0.10,   0.05]; % torso frame   --> rHipYaw joint
p.jnt_i_pi(3, :) = [-0.05,       0,  -0.15]; % rHipYaw frame --> rHipRol joint
p.jnt_i_pi(4, :) = [ 0.05,       0,      0]; % rHipRol frame --> rHipPit joint
p.jnt_i_pi(5, :) = [    0,       0, -0.375]; % rHipPit frame --> rKnePit joint
p.jnt_i_pi(6, :) = [    0,       0,  -0.35]; % rKnePit frame --> rAnkPit joint
% left leg
p.jnt_i_pi(7, :) = [    0,    0.10,   0.05]; % torso frame   --> lHipYaw joint
p.jnt_i_pi(8, :) = [-0.05,       0,  -0.15]; % lHipYaw frame --> lHipRol joint
p.jnt_i_pi(9, :) = [ 0.05,       0,      0]; % lHipRol frame --> lHipPit joint
p.jnt_i_pi(10, :) = [   0,       0, -0.375]; % lHipPit frame --> lKnePit joint
p.jnt_i_pi(11, :) = [   0,       0,  -0.35]; % lKnePit frame --> lAnkPit joint

% 2.com relative postion [c] -- CoM position w.r.t. body frame
p.com_i = zeros(p.nb, 3);
p.com_i(1, :) = [     0,       0,   0.22]; % torso com
% right leg
p.com_i(2, :) = [-0.078,       0, -0.118]; %  body com driven by rHipYaw 
p.com_i(3, :) = [  0.05,       0,      0]; %  body com driven by rHipRol
p.com_i(4, :) = [  0.04,       0, -0.159]; % thigh com driven by rHipPit
p.com_i(5, :) = [  0.03,       0, -0.138]; % shank com driven by rKnePit
p.com_i(6, :) = [     0,       0,  -0.04]; %  foot com driven by rAnkPit
% left leg
p.com_i(7, :) = [-0.078,       0, -0.118]; %  body com driven by lHipYaw 
p.com_i(8, :) = [  0.05,       0,      0]; %  body com driven by lHipRol
p.com_i(9, :) = [  0.04,       0, -0.159]; % thigh com driven by lHipPit
p.com_i(10, :) = [ 0.03,       0, -0.138]; % shank com driven by lKnePit
p.com_i(11, :) = [    0,       0,  -0.04]; %  foot com driven by lAnkPit

% 3.mass [m]
p.m_i = zeros(p.nb, 1);
p.m_i(1) = 23.038; % kg, torso mass (including hips yaw motor and arm loads)
% right leg : hipRolMotor, hipPitMotor,   thigh,   shank,   foot
p.m_i(2:6) = [      1.389,       2.507,   3.096,   1.914,    0.6]; % kg
% left leg
p.m_i(7:11) = p.m_i(2:6);

% 4.inertia [I] -- I w.r.t CoM of local body frame -- only principal inertia
p.I_i = zeros(p.nb, 3);
p.I_i(1, :) = [   0.68,    0.64,    0.18]; % kg*m^2, torso 
% right leg
p.I_i(2, :) = [  0.003,   0.003,   0.003]; % hipRolMotor
p.I_i(3, :) = [  0.004,   0.003,   0.004]; % hipPitMotor
p.I_i(4, :) = [  0.039,   0.039,   0.008]; % thigh
p.I_i(5, :) = [  0.016,   0.016,   0.004]; % shank
p.I_i(6, :) = [ 0.0012,  0.0025,  0.0025]; % foot
% left leg
p.I_i(7, :) = [  0.003,   0.003,   0.003]; % hipRolMotor
p.I_i(8, :) = [  0.004,   0.003,   0.004]; % hipPitMotor
p.I_i(9, :) = [  0.039,   0.039,   0.008]; % thigh
p.I_i(10, :) = [ 0.016,   0.016,   0.004]; % shank
p.I_i(11, :) = [0.0012,  0.0025,  0.0025]; % foot

% 5.actuated joint position bounds [q_{lb, ub}]
p.q_lb = [-inf, deg2rad([-20, -30, -80,   5, -65]), deg2rad([-20, -20, -80,   5, -65])]';
p.q_ub = [ inf, deg2rad([ 20,  20,  30, 135,  20]), deg2rad([ 20,  30,  30, 135,  20])]';

% 6.actuated joint velocity bounds[dq_{lb, ub}]
p.dq_lb = [-inf, -10, -10, -20, -20, -20, -10, -10, -20, -20, -20]';
p.dq_ub = [ inf,  10,  10,  20,  20,  20,  10,  10,  20,  20,  20]';

end

% Please don't modify this function!
% function: Transfer the intuitive model to spatial model.
function model = transferToSpatialModel(p)
% fcn: Transfer the intuitive robot model (struct p) to a spatial model, 
%      to facilitate the use of spatial dynamics.

% model fields:
% model.NB       -> number of body
% model.pitch    -> joint type
% model.parent   -> body parent
% model.Xtree    -> p(i) body frame spatialTransform to i-th body coincided joint frame
% model.I        -> spatial inertia w.r.t. i-th body frame origin

% NB, pitch, parent
model.NB = p.nb;
model.pitch = p.pitch;
model.parent = p.parent;

% Xtree
model.Xtree = cell(1, model.NB);
model.Xtree{1} =  Xtrans(p.jnt_i_pi(1, :));
for i = 2:model.NB
    % left X
    switch p.axis_i(i)
        case 1
            leftX = Xroty(pi/2);
        case 2
            leftX = Xrotx(-pi/2);
        case 3
            leftX = Xrotz(0);
        otherwise
            error('Invalid rotAxis!!!');
    end
    % right X
    switch p.axis_i(model.parent(i))
        case 1
            rightX = Xroty(-pi/2);
        case 2
            rightX = Xrotx(pi/2);
        case 3
            rightX = Xrotz(0);
        otherwise
            error('Invalid rotAxis!!!');
    end
    % Xtree
    model.Xtree{i} = leftX * Xtrans(p.jnt_i_pi(i, :)) * rightX;     
end

% I
model.I = cell(1, model.NB);
for i = 1:model.NB
    % rotR
    switch p.axis_i(i)
        case 1
            rotR = mx_Roty_3D( pi/2)';
        case 2
            rotR = mx_Rotx_3D(-pi/2)';
        case 3
            rotR = mx_Rotz_3D(0);
        otherwise
            error('Invalid rotAxis!!!');
    end
    
    % CoM
    CoM = (rotR * p.com_i(i, :)')';
    % Icm
    Icm = diag(abs(rotR * p.I_i(i, :)'));
    % I
    model.I{i} = mcI(p.m_i(i), CoM, Icm);
end

end