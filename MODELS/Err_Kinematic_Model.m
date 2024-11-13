%------------------------------------------%
% erros spm kinematic model control 
%------------------------------------------%

%% ------------------------------------------
% Dimensions
%% ------------------------------------------

nx               = 3; % No. of differential states
nu               = 3; % No. of controls
nz               = 0;  % No. of algebraic states
ny               = 6; % No. of outputs
nyN              = 3; % No. of outputs at the terminal point
np               = 6; % No. of model parameters
nc               = 1; % No. of general constraints
ncN              = 1; % No. of general constraints at the terminal point
nbx              = 0; % No. of bounds on states
nbu              = 3; % No. of bounds on controls

%% ------------------------------------------
% state and control bounds
%% ------------------------------------------
nbx_idx = 0; % indexs of states which are bounded
nbu_idx = 1:3; % indexs of controls which are bounded


%% ------------------------------------------
%% create variables
%% ------------------------------------------

import casadi.*

states         = SX.sym('states',nx,1);   % differential states
controls       = SX.sym('controls',nu,1); % control input
alg            = SX.sym('alg',nz,1);      % algebraic states
params         = SX.sym('paras',np,1);    % parameters
refs           = SX.sym('refs',ny,1);     % references of the first N stages
refN           = SX.sym('refs',nyN,1);    % reference of the last stage
Q              = SX.sym('Q',ny,1);        % weighting matrix of the first N stages
QN             = SX.sym('QN',nyN,1);      % weighting matrix of the last stage
aux            = SX.sym('aux',ny,1);      % auxilary variable
auxN           = SX.sym('auxN',nyN,1);    % auxilary variable



%% ------------------------------------------
% Differential Kinematic Model
%% ------------------------------------------
e1 = states(1);
e2 = states(2);
e3 = states(3);

u1 = controls(1);
u2 = controls(2);
u3 = controls(3);

p1 = params(1);
p2 = params(2);
p3 = params(3);
p4 = params(4);
p5 = params(5);
p6 = params(6);

x1 = p1 - e1;
x2 = p2 - e2;
x3 = p3 - e3;


E = [e1;e2;e3];
P = [p1;p2;p3;p4;p5;p6];
u = [u1;u2;u3];
X = [x1;x2;x3];

%% ------------------------------------------
% Explicit(expl_f = f(x,u)) and Implicit(impl_f = 0) ODE RHS
%% ------------------------------------------
x_dot=Err_Kinematic_ODE_CT(E,u,P);  
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;  

% algebraic function
z_fun = [];                   

      
%% ------------------------------------------
% Objectives and constraints
%% ------------------------------------------

% inner objectives
h = [e1;e2;e3;u1;u2;u3];
hN = [e1;e2;e3];


% outer objectives
obji = 0.5*(h-refs)'*diag(Q)*(h-refs);
objN = 0.5*(hN-refN)'*diag(QN)*(hN-refN);

obji_GGN = 0.5*(aux-refs)'*(aux-refs);
objN_GGN = 0.5*(auxN-refN)'*(auxN-refN);



% general inequality constraints
general_con = ConditionIndex(X);
general_con_N = ConditionIndex(X);

%% NMPC discretizing time length [s]

Ts_st = 1e-3; % shooting interval time
