function X = mpc_nmpcsolver_simulink_V3(Input)

load('settings');

time = Input(1);
[r,dr,ddr,ur] = RefGenerator(time,'dynamic');
Ts_st = settings.Ts_st;  % Shooting interval
nx = settings.nx;    % No. of states
nu = settings.nu;    % No. of controls
ny = settings.ny;    % No. of outputs (references)    
nyN= settings.nyN;   % No. of outputs at terminal stage 
np = settings.np;    % No. of parameters (on-line data)
nc = settings.nc;    % No. of constraints
ncN = settings.ncN;  % No. of constraints at terminal stage
nbx = settings.nbx;  % No. of state bounds
% Nmpc = settings.N;

[input, data , Ref] = InitData(settings, time);

persistent mem
if isempty(mem)
    mem = InitMemory(settings, opt, input);
end

X = 1*time;



end