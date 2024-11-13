clear all; clear mex; close all;clc;

disp( ' ' );
disp( 'MATMPC -- A (MAT)LAB based Model(M) Predictive(P) Control(C) Package.' );
disp( 'Copyright (C) 2016-2019 by Yutao Chen, University of Padova' );
disp( 'All rights reserved.' );
disp( ' ' );
disp( 'MATMPC is distributed under the terms of the' );
disp( 'GNU General Public License 3.0 in the hope that it will be' );
disp( 'useful, but WITHOUT ANY WARRANTY; without even the implied warranty' );
disp( 'of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.' );
disp( 'See the GNU General Public License for more details.' );
disp( ' ' );
disp( ' ' );
disp('---------------------------------------------------------------------------------');

%% Configuration (complete your configuration here...)

if ismac
    addpath(genpath([pwd,'/solver/mac']));
elseif isunix
    addpath(genpath([pwd,'/solver/linux']));
elseif ispc
    addpath(genpath([pwd,'/solver/win64']));
else
    disp('Platform not supported')
end

cd data;
if exist('settings','file')==2
    load settings
    cd ..
else 
    cd ..
    error('No setting data is detected!');
end

Ts = settings.Ts_st;     % Closed-loop sampling time (usually = shooting interval)

Ts_st = settings.Ts_st;  % Shooting interval
nx = settings.nx;    % No. of states
nu = settings.nu;    % No. of controls
ny = settings.ny;    % No. of outputs (references)    
nyN= settings.nyN;   % No. of outputs at terminal stage 
np = settings.np;    % No. of parameters (on-line data)
nc = settings.nc;    % No. of constraints
ncN = settings.ncN;  % No. of constraints at terminal stage
nbx = settings.nbx;  % No. of state bounds

%% solver configurations

N  = 40;             % No. of shooting points
settings.N = N;

N2 = N/5;
settings.N2 = N2;    % No. of horizon length after partial condensing (N2=1 means full condensing)

r = 10;
settings.r = r;      % No. of input blocks (go to InitMemory.m, line 441 to configure)

opt.hessian         = 'Gauss_Newton';  % 'Gauss_Newton', 'Generalized_Gauss_Newton'
opt.integrator      = 'ERK4'; % 'ERK4','IRK3','IRK3-DAE'
opt.condensing      = 'default_full';  %'default_full','no','blasfeo_full(require blasfeo installed)','partial_condensing'
opt.qpsolver        = 'qpoases';    %'qpoases';  quadprog_dense qpoases_sequence
opt.hotstart        = 'no'; %'yes','no' (only for qpoases, use 'no' for nonlinear systems)
opt.shifting        = 'no'; % 'yes','no'
opt.ref_type        = 0; % 0-time invariant, 1-time varying(no preview), 2-time varying (preview)
opt.nonuniform_grid = 0; % if use non-uniform grid discretization (go to InitMemory.m, line 459 to configure)
opt.RTI             = 'no'; % if use Real-time Iteration 'yes','no
%% available qpsolver

%'qpoases' (condensing is needed)
%'qpoases_mb' (move blocking strategy)
%'quadprog_dense' (for full condensing)
%'hpipm_sparse' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'hpipm_pcond' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'ipopt_dense' (install OPTI Toolbox first; for full condensing)
%'ipopt_sparse' (install OPTI Toolbox first; set opt.condensing='no')
%'ipopt_partial_sparse'(set opt.condensing='partial_condensing'; only for state and control bounded problems)
%'osqp_sparse' (set opt.condensing='no')
%'osqp_partial_sparse' (set opt.condensing='partial_condensing')
%'qpalm_cond' (condensing is needed)
%'qpalm_sparse'(set opt.condensing='no')

%% Initialize Data (all users have to do this)

D2R = pi/180;
R2D = 180/pi;

[r,dr,ddr,ur] = RefGenerator(0,'dynamic');
X0 = [0;20;0;0;0;0] * D2R;



xk = X0;
Ek = xk - [r;dr];
uk = [0;0;0];
state_k = [xk;Ek];
settings.x0 = state_k;
[input, data , Ref] = InitData(settings,0,0);

%% Initialize Solvers (only for advanced users)
mem = InitMemory(settings, opt, input);

%% Simulation (start your simulation...)

mem.iter = 1; time = 0.0;
Tf = 4;  % simulation time
state_sim      = xk';
controls_MPC   = uk';
error_sim      = (r-xk(1:3))';
y_sim          = [];
constraints    = [];
CPT            = [];
ref_traj       = [];
KKT            = [];
OBJ            = [];
numIT          = [];
CPT_           = [];

while time(end) < Tf
     
    waitbar(time(end)/Tf);

    %% --------------------------------
    % Generate Refrence 
    %% --------------------------------
   Time = time(end);
   % [r,dr,ddr,ur] = RefGenerator(Time,'dynamic');
   % Ek = xk - [r;dr];
   % state_k = [xk;Ek];
   % settings.x0 = state_k;
   % [input, data , Ref] = InitData(settings, Time,mem);
   input.y = repmat(data.REF',1,N);
   input.yN = data.REF(1:nyN)';
   
     
   
              
    % obtain the state measurement
    % input.x0 = state_sim(end,:)';
    % [r,dr,ddr,ur] = RefGenerator(Time,'dynamic');
    
    % input.x0 = state_k;
    % call the NMPC solver 
    tic;
    [output, mem] = mpc_nmpcsolver(input, settings, mem, opt);
    CPT_k = toc;
    CPT_ = [CPT_;CPT_k];
        
    % obtain the solution and update the data
    input.x=output.x;
    input.u=output.u;
    input.z=output.z;
    input.lambda=output.lambda;
    input.mu=output.mu;
    input.mu_x=output.mu_x;
    input.mu_u=output.mu_u;

    
    % collect data
    cpt=output.info.cpuTime;
    tshooting=output.info.shootTime;
    tcond=output.info.condTime;
    tqp=output.info.qpTime;
    OptCrit=output.info.OptCrit;
    
    % Simulate system dynamics
    Utraj = output.u;
    uk = Utraj(:,1);
    x_dot = Dynamic_ODE_CT(xk,uk);
    [t,y] = ode45(@(t,y) x_dot, [0 Ts], xk);
    xkp1  = y(end,:)';
    xk = xkp1;
    % [r,dr,ddr,ur] = RefGenerator(Time+Ts,'dynamic');
    % display((r - xk(1:3))*180/pi);
    Dis = Discriminants(xk);
    CI = ConditionIndex(xk);
    display([xk(1:6)*180/pi;Dis;CI]');  
    % display([xk(1:6)*180/pi;Dis;CI;(r - xk(1:3))*180/pi]');

    
 
    % store the optimal solution and states
    controls_MPC = [controls_MPC; uk'];
    state_sim = [state_sim; xk'];
    error_sim = [error_sim; (r-xk(1:3))'];
    KKT= [KKT;OptCrit];
    OBJ= [OBJ;output.info.objValue];
    CPT = [CPT; cpt, tshooting, tcond, tqp];
    numIT = [numIT; output.info.iteration_num];
    
    % go to the next sampling instant
    nextTime = mem.iter*Ts; 
    mem.iter = mem.iter+1;
    % disp(['current time:' num2str(nextTime) '  CPT:' num2str(cpt) 'ms  SHOOTING:' num2str(tshooting) 'ms  COND:' num2str(tcond) 'ms  QP:' num2str(tqp) 'ms  Opt:' num2str(OptCrit) '   OBJ:' num2str(OBJ(end)) '  SQP_IT:' num2str(output.info.iteration_num)]);
    time = [time nextTime];   
   [r,dr,ddr,ur] = RefGenerator(Time,'dynamic');
   Ek = xk - [r;dr];
   state_k = [xk;Ek];
   settings.x0 = state_k;
   [input, data , Ref] = InitData(settings, Time,mem);
   % display((r - xk(1:3))*180/pi);
    
end

%%
if strcmp(opt.qpsolver, 'qpoases')
    qpOASES_sequence( 'c', mem.warm_start);
end
% if strcmp(opt.qpsolver, 'qpalm')
%     mem.qpalm_solver.delete();
% end
% clear mex;

%% draw pictures (optional)
disp(['Average CPT: ', num2str(mean(CPT(2:end,:),1)) ]);
disp(['Maximum CPT: ', num2str(max(CPT(2:end,:))) ]);

Draw;
