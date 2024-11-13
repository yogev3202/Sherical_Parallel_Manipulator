clc;
clear;
close all

%% --------------------------------------
%% Declare parameters
%% --------------------------------------
import casadi.*
Alpha1 = casadi.MX.sym('Alpha1');
Alpha2 = casadi.MX.sym('Alpha2');
Xa1 = casadi.MX.sym('Xa1',3,1);
Xa2 = casadi.MX.sym('Xa2',3,1);
Xb1 = casadi.MX.sym('Xb1',3,1);
Xb2 = casadi.MX.sym('Xb2',3,1);

%% --------------------------------------
% Build objective and constraints expression
%
% min  f(x,p)
%  x
%
% s.t.   lbg <= g(x,p) <= ubg
%        lbx <= x <= ubx
%% --------------------------------------

S1 = Alpha1*Xa1 + (1-Alpha1)*Xb1;
S2 = Alpha2*Xa2 + (1-Alpha2)*Xb2;

Vars = [Alpha1;Alpha2];
lbx = [0;0];
ubx = [1;1];
Params = [Xa1;Xb1;Xa2;Xb2];


J = (S1-S2)'*(S1-S2);
g = [];


%% --------------------------------------
% Construct QP solver
%% --------------------------------------

qp_struct = struct;
qp_struct.f = J;
qp_struct.g = g;
qp_struct.x = veccat(Vars{:});
qp_struct.p = veccat(Params{:});


max_iter = 1000;
constr_viol_tol = 1e-8;
dual_inf_tol = 1e-8;
verbose = true;
options = struct;
options.max_iter = max_iter;
options.constr_viol_tol = constr_viol_tol;
options.dual_inf_tol = dual_inf_tol;
options.print_iter = verbose;
options.print_time = verbose;
options.print_info = verbose;
options.print_header = verbose;
solver = qpsol('solver','qrqp',qp_struct,options);
% solver = qpsol('solver','osqp',qp_struct);


%% --------------------------------------
% Call QP solver symbolically
%% --------------------------------------

x0 = MX.sym('x_initial',size(Vars,1),1);
lam_x0 = MX.sym('lam_x_initial',size(Vars,1),1);
lam_g0 = MX.sym('lam_g_initial',size(g,1),1);

arg_struct = struct;
arg_struct.x0 = x0;
arg_struct.lam_x0 = lam_x0;
arg_struct.lam_g0 = [];
arg_struct.lbx = lbx;
arg_struct.ubx = ubx;
arg_struct.lbg = [];
arg_struct.ubg = [];
arg_struct.p = qp_struct.p;
res_struct = solver.call(arg_struct);

sol = res_struct.x;


%% --------------------------------------
% Create a Function of QP Result
%% --------------------------------------
% Input = {x0,lam_x0,lam_g0,veccat(Params{:})};
Input = {x0,lam_x0,veccat(Params{:})};
Output = {sol};
Collition_Fcn = Function('Collition_Fcn',Input,Output);

%% --------------------------------------
% Generate code
%% --------------------------------------

cg_options = struct;
cg_options.casadi_real = 'real_T';
cg_options.casadi_int = 'int_T';
cg_options.with_header = true;
cg_options.mex = true;
cg_options.real_min = num2str(realmin('double'));
if verbose
    cg_options.verbose_runtime = true;
end

cg = CodeGenerator('Collition_Detector.c',cg_options);
cg.add(Collition_Fcn);
cg.generate();
mex Collition_Detector.c -largeArrayDims  % Matlab
