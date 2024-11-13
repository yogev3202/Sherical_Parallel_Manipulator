% ------------------------------------------------------
%  This is an example of initializing simulink simulation
%  ------------------------------------------------------

%%
clear mex; close all; clear; clc;

%% Parametri Simulazione
% cd DATA;
% load('settings');
% cd ..
% run('SPM_ASSEMBLY_DataFile');

% path = getfullname(gcb) 
CTR_PATH = 'Dynamic_Simulation/Aug_Dynamic_Model/CTR_PARAMETERS';
CTR_MASK = Simulink.Mask.get(CTR_PATH);

dt = CTR_MASK.Parameters(1, 1).Value;
Nmpc = CTR_MASK.Parameters(1, 2).Value;
Qv1 = CTR_MASK.Parameters(1, 3).Value;
Qv2 = CTR_MASK.Parameters(1, 4).Value;
Qz1 = CTR_MASK.Parameters(1, 5).Value;
Rv = CTR_MASK.Parameters(1, 6).Value;
GenMex = CTR_MASK.Parameters(1, 7).Value;
X1_0 = CTR_MASK.Parameters(1, 8).Value;
Wp_0 = CTR_MASK.Parameters(1, 9).Value;
X1r_0 = CTR_MASK.Parameters(1, 10).Value;
Wr_0 = CTR_MASK.Parameters(1, 11).Value;
Kp = CTR_MASK.Parameters(1, 12).Value;
Kv = CTR_MASK.Parameters(1, 13).Value;
Ki = CTR_MASK.Parameters(1, 14).Value;




dt = str2num(dt); Nmpc = str2num(Nmpc); Qv1 = str2num(Qv1);
Qv2 = str2num(Qv2); Rv = str2num(Rv); X1_0 = str2num(X1_0);
Wp_0 = str2num(Wp_0); X1r_0 = str2num(X1r_0); Wr_0 = str2num(Wr_0);
Kp = str2num(Kp); Kv = str2num(Kv); Ki = str2num(Ki);
Qz1 = str2num(Qz1); 




if (strcmp(GenMex,'on'))

    settings.model='Aug_Dynamic_Model'; 
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

N  = Nmpc;
N2 = N/5;
r  = 10;

settings.N = N;
settings.N2 = N2;
settings.r = r;




%% options
opt.hessian='Gauss_Newton';  % 'Gauss_Newton', 'Generalized_Gauss_Newton'
opt.integrator='ERK4'; % 'ERK4','IRK3, 'IRK3-DAE'
opt.condensing='default_full';  %'default_full','no','blasfeo_full(require blasfeo installed)','partial_condensing'
opt.qpsolver='qpoases';  %ipopt_dense , qpoases , qpoases_mb
opt.hotstart='yes'; %'yes','no' (only for qpoases)
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

X1_0 = X1_0 * (pi/180);
Wp_0 = Wp_0 * (pi/180);
X1r_0 = X1r_0 * (pi/180);
Wr_0 = Wr_0 * (pi/180);

THETAv = SPMIK_Fcn(X1_0,'---');
thetaZ0 = -1.140243740887195;

M1Z_A = (THETAv(1) - thetaZ0) * (180/pi);
M2Z_A = (THETAv(2) - thetaZ0) * (180/pi);
M3Z_A = (THETAv(3) - thetaZ0) * (180/pi);

X2_0 = W2dEA(X1_0,Wp_0);
X2r_0 = W2dEA(X1r_0,Wr_0);
Um = SPMID_Fcn(X1_0,X2_0,zeros(3,1));

X0 = [X1_0;X2_0];
Xr0 = [X1r_0;X2r_0];
Z0 = X1r_0 - X1_0 ;



x0 = [X0;Z0];  
u0 = zeros(nu,1); 
z0 = zeros(nz,1);
para0 = zeros(max(1,np),1);  



W=repmat([Qv1,Qv2,Qz1,Rv]',1,N);
WN=W(1:nyN,1);

% upper and lower bounds for states (=nbx)
lb_x = -800 * (pi/180) * ones(3,1);
ub_x =  800 * (pi/180) * ones(3,1);
% upper and lower bounds for controls (=nbu)           
lb_u = -100*1e-3 * ones(3,1);
ub_u = 100*1e-3 * ones(3,1);
                       
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
