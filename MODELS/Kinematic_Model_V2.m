%------------------------------------------%
% SPM Dynamic Control 
  
% "Autogenerating microsecond solvers for nonlinear MPC: A tutorial
% using ACADO integrators", Quirynen, 2015

% typical configuration: 1) N=80, Ts=Ts_st=0.025, no shifting 2) N=40,
% Ts=Ts_st=0.05, shifting

%------------------------------------------%


%% Dimensions

nx  = 6;  % No. of differential states
nu  = 3;  % No. of controls
nz  = 0;  % No. of algebraic states
ny  = 6; % No. of outputs
nyN = 3; % No. of outputs at the terminal point
np  = 6; % No. of model parameters
nc  = 4; % No. of general constraints
ncN = 4; % No. of general constraints at the terminal point
nbx = 3; % No. of bounds on states
nbu = 3; % No. of bounds on controls

% state and control bounds
nbx_idx = 1:3; % indexs of states which are bounded
nbu_idx = 1:3; % indexs of controls which are bounded

%% create variables

import casadi.*

states   = SX.sym('states',nx,1);   % differential states
controls = SX.sym('controls',nu,1); % control input
alg      = SX.sym('alg',nz,1);      % algebraic states
params   = SX.sym('paras',np,1);    % parameters
refs     = SX.sym('refs',ny,1);     % references of the first N stages
refN     = SX.sym('refs',nyN,1);    % reference of the last stage
Q        = SX.sym('Q',ny,1);        % weighting matrix of the first N stages
QN       = SX.sym('QN',nyN,1);      % weighting matrix of the last stage
aux      = SX.sym('aux',ny,1);      % auxilary variable
auxN     = SX.sym('auxN',nyN,1);    % auxilary variable




%% Kinematic

x1 = states(1);x2 = states(2);x3 = states(3);
e1 = states(4);e2 = states(5);e3 = states(6);


p1 = params(1);p2 = params(2);p3 = params(3);
p4 = params(4);p5 = params(5);p6 = params(6);


E = [e1;e2;e3];
X = [x1;x2;x3];

qr   = [p1;p2;p3];
dqr  = [p4;p5;p6];



u1 = controls(1);
u2 = controls(2);
u3 = controls(3);



% explicit ODE RHS
RHS = Kinematic_ODE_CT(states,controls);
x_dot =  [RHS; 
         RHS - dqr];  

% algebraic function
z_fun = [];                   

% implicit ODE: impl_f = 0
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;        
     
%% Objectives and constraints

% inner objectives

h = [states(4:6);controls];
hN = h(1:nyN);




% outer objectives
obji = 0.5*(h-refs)'*diag(Q)*(h-refs);
objN = 0.5*(hN-refN)'*diag(QN)*(hN-refN);

obji_GGN = 0.5*(aux-refs)'*(aux-refs);
objN_GGN = 0.5*(auxN-refN)'*(auxN-refN);



% general inequality constraints

h_expr  = [Discriminants(X);ConditionIndex(X)];
hN_expr = [Discriminants(X);ConditionIndex(X)];

% [h_expr,hN_expr] = h_expr_Fcn([X1;X2],controls,params);
general_con = h_expr;
general_con_N = hN_expr;



%% NMPC discretizing time length [s]

% Ts_st = dt; % shooting interval time
Ts_st = 1e-3; % shooting interval time