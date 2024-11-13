% ------------------------------------------------------
%  initializing spm kinematic model simulink simulation
%  ------------------------------------------------------

%%
clear mex; close all; clear; clc;

CTR_PATH = 'SIMSCAPE_SIMULATION/Kinematic_Model/CTR_PARAMETERS';
CTR_MASK = Simulink.Mask.get(CTR_PATH);

Nx = CTR_MASK.Parameters(1, 1).Value;
Nu = CTR_MASK.Parameters(1, 2).Value;
Ny = CTR_MASK.Parameters(1, 3).Value;
dt = CTR_MASK.Parameters(1, 4).Value;
N_mpc = CTR_MASK.Parameters(1, 5).Value;
Qv = CTR_MASK.Parameters(1, 6).Value;
Rv = CTR_MASK.Parameters(1, 7).Value;
GenMex = CTR_MASK.Parameters(1, 8).Value;
x0 = CTR_MASK.Parameters(1, 9).Value;
ref0 = CTR_MASK.Parameters(1, 10).Value;


Nx = str2num(Nx); Nu = str2num(Nu); Ny = str2num(Ny);
dt = str2num(dt); N_mpc = str2num(N_mpc); Qv = str2num(Qv);
Rv = str2num(Rv); x0 = str2num(x0); ref0 = str2num(ref0);


if (strcmp(GenMex,'on'))

    settings.model='Err_Kinematic_Model'; 
    run('Model_Generation.m');
else
    cd DATA;
    load('settings');
    cd ..
    run('SPM_ASSEMBLY_DataFile');
end


Ts  = settings.Ts_st;    % Sampling time
Ts_st = settings.Ts_st;  % Shooting interval
nx = settings.nx;        % No. of differential states
nu = settings.nu;        % No. of controls
nz = settings.nz;        % No. of algebraic states
ny = settings.ny;        % No. of outputs (references)    
nyN= settings.nyN;       % No. of outputs at terminal stage 
np = settings.np;        % No. of parameters (on-line data)
nc = settings.nc;        % No. of constraints
ncN = settings.ncN;      % No. of constraints at terminal stage
nbu = settings.nbu;      % No. of control bounds
nbx = settings.nbx;      % No. of state bounds
nbu_idx = settings.nbu_idx; % Index of control bounds
nbx_idx = settings.nbx_idx; % Index of state bounds

%% add more to Settings

N  = N_mpc;
N2 = N/5;
r  = 10;

settings.N = N;
settings.N2 = N2;
settings.r = r;

THETAv = SPMIK_Fcn(x0 * (pi/180),'---');
thetaZ0 = -1.140243740887195;

q0_1 = (THETAv(1) - thetaZ0) * (180/pi);
q0_2 = (THETAv(2) - thetaZ0) * (180/pi);
q0_3 = (THETAv(3) - thetaZ0) * (180/pi);


%% options
opt.hessian='Gauss_Newton';  % 'Gauss_Newton', 'Generalized_Gauss_Newton'
opt.integrator='ERK4'; % 'ERK4','IRK3, 'IRK3-DAE'
opt.condensing='default_full';  %'default_full','no','blasfeo_full(require blasfeo installed)','partial_condensing'
opt.qpsolver='qpoases';  %ipopt_dense , qpoases 
opt.hotstart='no'; %'yes','no' (only for qpoases)
opt.shifting='no'; % 'yes','no'
opt.ref_type=0; % 0-time invariant, 1-time varying(no preview), 2-time varying (preview)
opt.nonuniform_grid=0; % currently not supported 
opt.RTI             = 'yes';

%% available qpsolver
%'qpoases' (for full condensing)
%'qpoases_mb' (for full condensing+moving block, please use ERK4 as the integrator)
%'hpipm_sparse' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'hpipm_pcond' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
 
%% Initialization

x0 = (ref0 - x0)*(pi/180);     
u0 = zeros(nu,1); 
z0 = zeros(nz,1);

% [r,dr,ddr,ur] = RefGenerator(0,'kinematic');
para0 = [ref0;zeros(3,1)] * (pi/180);  


Qx = Qv;
Qu = Rv;
W=repmat([Qx Qu]',1,N);
WN=W(1:nyN,1);

% upper and lower bounds for states (=nbx)
lb_x = [];
ub_x =[];
% upper and lower bounds for controls (=nbu)           
lb_u = -500 * (pi/180) * ones(3,1);
ub_u = 500 * (pi/180) * ones(3,1);
                       
% upper and lower bounds for general constraints (=nc)
lb_g = 0.1;
ub_g = 1e5;            
lb_gN = 0.1;
ub_gN = 1e5;




%%
lb = repmat(lb_g,N,1);
ub = repmat(ub_g,N,1);
lb = [lb;lb_gN];
ub = [ub;ub_gN];
if isempty(lb)
    lb=0;
    ub=0;
end
        
lbu = -inf(nu,1);
ubu = inf(nu,1);
for i=1:nbu
    lbu(nbu_idx(i)) = lb_u(i);
    ubu(nbu_idx(i)) = ub_u(i);
end
        
lbu = repmat(lbu,1,N);
ubu = repmat(ubu,1,N);

lbx = repmat(lb_x,1,N+1);
ubx = repmat(ub_x,1,N+1);
if isempty(lbx)
    lbx=0;
    ubx=0;
end

x = repmat(x0,1,N+1);   
u = repmat(u0,1,N);  
z = repmat(z0,1,N);
para = repmat(para0,1,N+1);  

if isempty(z)
    z0=0;
    z=0;
end
