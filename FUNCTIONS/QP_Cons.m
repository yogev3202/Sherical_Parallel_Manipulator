function [ubxk,lbxk,ubgk,lbgk,gk] = QP_Cons(epsilon,rhok,x0,CI_Fcn,CIx_Fcn)


import casadi.*

% XXX = SX.sym('df')
% ubxk = ones(3,1)*20*1e-3;
% lbxk = -ones(3,1)*20*1e-3;



% CI = ConditionIndex(x0);
% Grad_CI = jacobian(CI,x0);


% F_Grad_CI = casadi.Function('F_Grad_CI',{x0},{Grad_CI});
% d = CI - Grad_CI*x0;
% F_d = casadi.Function('F_d',{x0},{d});
% Ak_cons = [F_Grad_CI(rhok)];
% A_hat(Ahat_range,Rho_range) = Ak_cons;
% 
% dk = F_d(rhok);
% ubgk = [inf];
% lbgk = [epsilon-dk];
% 
% gk = F_Grad_CI(rhok)*x0;


    

CI_fun = CI_Fcn;
Grad_CI_fun = CIx_Fcn;
dk = CI_fun(rhok) - Grad_CI_fun(rhok)*rhok;
gk = Grad_CI_fun(rhok)*x0;
ubgk = [inf];
lbgk = [epsilon-dk];
ubxk = ones(3,1)*20*1e-3;
lbxk = -ones(3,1)*20*1e-3;






end

