function [cpt_qp, mem] = mpc_qp_solve_qpoases(sizes,mem)
    
    nu=sizes.nu;
    N=sizes.N;  
    nbx=sizes.nbx;
                   
    [sol,fval,exitflag,iterations,multiplier,auxOutput] = qpOASES(mem.Hc,mem.gc,[mem.Ccx;mem.Ccg],...
            mem.lb_du,mem.ub_du,[mem.lxc;mem.lcc],[mem.uxc;mem.ucc],mem.qpoases_opt);

    mem.du(:) = reshape(sol, [nu,N]);
    mem.mu_u_new(:)  = - multiplier(1:N*nu); 
    mem.mu_x_new(:) = -multiplier(N*nu+1:N*nu+N*nbx);
    mem.mu_new(:)   = - multiplier(N*nu+N*nbx+1:end);
    cpt_qp   = auxOutput.cpuTime*1e3;
    mem.QP_Flag = exitflag;
    % disp(exitflag);
                
    Recover(mem, sizes);
end
