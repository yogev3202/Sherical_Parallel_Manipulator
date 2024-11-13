function [J,Hessian_symetric,F] = Cost_Fcn_qlmpc(qlmpc_info,N,Q)


    H = qlmpc_info.H;
    S = qlmpc_info.S;
    W_hat = qlmpc_info.W_hat;
    Xr = qlmpc_info.Xr;
    Ur = qlmpc_info.Ur;
    Uk = qlmpc_info.Uk;
    x0 = qlmpc_info.x0;
    
    
    
        
        
    Q1v = Q(1:3);
    Q2v = Q(4:6);
    Rv  = Q(7:9);
    
    Q = diag([Q1v;Q2v]);
    R = diag(Rv);
    
    Q_hat = kron(eye(N+1), Q);
    R_hat = kron(eye(N),   R);
    
    Xk = H*x0 + S*Uk + W_hat;
    J = (Xk-Xr)'*Q_hat*(Xk-Xr) + (Uk-Ur)'*R_hat*(Uk-Ur);
    
    F = (2*x0'*H'*Q_hat*S+2*(W_hat)'*Q_hat*S-2*Xr'*Q_hat*S...
        -2*Ur'*R_hat);
    
    Hessian = 2*(S' * Q_hat * S + R_hat);
    Hessian_symetric = (Hessian+Hessian')*0.5;



end

