%% Example test -- mx spatial dynamics library
% A lightweighted and user-friendly dyanmics library using matlab.
% This library is written and developed by Xiang Meng (PhD candicate 
% in legged robotics) in Beijing Institute of Technology (BIT) on 2024/04.
% Referring to Roy Featherstone's spatial dynamics algorithm.
% If you have any question, you can contact me by emails
% (3120215100@bit.edu.cn).
% Copyrights Reserved 2024.
clear; close all; clc

%% add path to your matlab
dyn_folder = './mx_spatial_lib';
addpath( genpath(dyn_folder) );

%% setup robot spatial model and transfer to floating base model
% Users setup your model in this function.
model = setupRobotSpatialModel_myRobot_fixedBase(); 

myrobot = MX_RobotSpatialDyn(model);  % setup spatial model
myrobot.transFloatBase();             % transfer to floating base model

%% q and dq
ndof = myrobot.tree.NB;

leg_config = deg2rad([0 0 -30 62.4 -32.4]);

% q_ref, dq_ref
q_ref = [0 0 0.80027 0 0 0 leg_config leg_config]';
dq_ref = zeros(ndof, 1);
% dq_ref = rand(ndof, 1); % random dq

myrobot.updateRobState(q_ref, dq_ref);

%% get centroidal property
[cG, hG, AG, mass, vG, X_G_0, IG] = myrobot.getCentroidalProperty();
dAG = myrobot.getAGDot();

%% get dynamics matrices
[H, C] = myrobot.getDynamicsHandC();

%% get kinematics property
pos_ft_local = [0; 0; -0.08]; % pos_w

rfoot_idx = 11;
[rft_P, rft_R] = myrobot.getPointWorldPosRotm(rfoot_idx, 'y', pos_ft_local);
rft_Vel = myrobot.getPointWorldVel(rfoot_idx, 'y', pos_ft_local);
rft_Jac  = myrobot.getPointWorldJac(rfoot_idx, 'y', pos_ft_local);
rft_dJac = myrobot.getPointWorldJacDot(rfoot_idx, 'y', pos_ft_local);

lfoot_idx = 16;
[lft_P, lft_R] = myrobot.getPointWorldPosRotm(lfoot_idx, 'y', pos_ft_local);
lft_Vel = myrobot.getPointWorldVel(lfoot_idx, 'y', pos_ft_local);
lft_Jac  = myrobot.getPointWorldJac(lfoot_idx, 'y', pos_ft_local);
lft_dJac = myrobot.getPointWorldJacDot(lfoot_idx, 'y', pos_ft_local);
