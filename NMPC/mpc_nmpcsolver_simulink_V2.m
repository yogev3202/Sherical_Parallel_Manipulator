function [input, mem, cpt,KKT,OBJ] = mpc_nmpcsolver_simulink_V2(input, settings, mem, opt)




    mem.sqp_it=0;
    mem.alpha =1;
    mem.obj=0;
    StopCrit = 2*mem.kkt_lim;
    
    CPT.SHOOT=0;
    CPT.COND=0;
    CPT.QP=0;

    tic;
  
    while(mem.sqp_it < mem.sqp_maxit  &&  StopCrit > mem.kkt_lim && mem.alpha>1E-8 ) % RTI or multiple call
        
        %% ----------- QP Preparation
        
        tshoot = tic;
        if opt.nonuniform_grid
            qp_generation_ngrid(input, settings, mem);
        else
            if strcmp(opt.qpsolver, 'qpoases_mb')
                qp_generation_mb(input, settings, mem);
            else
                qp_generation(input, settings, mem);
%                 qp_generation_tac(input, settings, mem);
            end
        end
      
        tSHOOT = toc(tshoot)*1e3; 
        
        switch opt.condensing
            case 'default_full'              
                tcond=tic;
                if ~strcmp(opt.qpsolver, 'qpoases_mb')
                    Condensing(mem, settings);
                else
                    Condensing_mb(mem, settings);
                end
                tCOND=toc(tcond)*1e3;
                
            case 'hpipm_full'
                tcond=tic;
                condensing_hpipm(mem, settings);
                tCOND=toc(tcond)*1e3;
            case 'blasfeo_full'
                tcond=tic;
                Condensing_Blasfeo(mem, settings);
                tCOND=toc(tcond)*1e3;
            case 'partial_condensing'
                tcond=tic;
                mem.mem2 = Pcond(mem, settings, mem.mem2, mem.settings2);
                tCOND=toc(tcond)*1e3;                
            case 'no'
                tCOND = 0;
        end
                                            
        %% ----------  Solving QP
        switch opt.qpsolver
            case 'qpoases'              
                [tQP,mem] = mpc_qp_solve_qpoases(settings,mem);
                
            case 'qpoases_mb'              
                [tQP,mem] = mpc_qp_solve_qpoases_mb(settings,mem, opt);
                
            case 'quadprog_dense'
                [tQP,mem] = mpc_qp_solve_quadprog(settings,mem);
                
            case 'hpipm_sparse'               
                tqp=tic;
                hpipm_sparse(mem, settings);
                tQP = toc(tqp)*1e3;
                
            case 'hpipm_pcond'               
                tqp=tic;
                hpipm_pcond(mem,settings);
                tQP = toc(tqp)*1e3;
                
            case 'ipopt_dense'               
                [tQP,mem] = mpc_qp_solve_ipopt_dense(settings,mem);
                
            case 'ipopt_sparse'               
                [tQP,mem] = mpc_qp_solve_ipopt_sparse(settings,mem);
                
            case 'ipopt_partial_sparse'
                [tQP, mem] = mpc_qp_solve_ipopt_partial_sparse(settings,mem.settings2,mem, mem.mem2);               
                                
            case 'osqp_sparse'
                [tQP, mem] = mpc_qp_solve_osqp(settings,mem);
                
            case 'osqp_partial_sparse'
                [tQP, mem] = mpc_qp_solve_osqp_partial(settings,mem.settings2,mem,mem.mem2);
                
            case 'qpalm_cond'
                [tQP, mem] = mpc_qp_solve_qpalm_cond(settings,mem);
                
            case 'qpalm_sparse'
                [tQP, mem] = mpc_qp_solve_qpalm_sparse(settings,mem);
        end
        

        %% ---------- Line search

        Line_search(mem, input, settings);
        
        %% ---------- CMoN
%         if mem.iter==1 && mem.sqp_it==0
%             [mem.rho_cmon, mem.gamma] = CMoN_Init(settings, mem, input);                                   
%         end  
%         adaptive_eta(mem,settings);
                
        %% ---------- KKT calculation 
        
        [eq_res, ineq_res, KKT, OBJ] = solution_info(input, settings, mem);
                
        StopCrit = max([eq_res, ineq_res, KKT]);
        
        %% ---------- Multiple call management and convergence check
                        
        CPT.SHOOT=CPT.SHOOT+tSHOOT;
        CPT.COND=CPT.COND+tCOND;
        CPT.QP=CPT.QP+tQP;
        
        mem.sqp_it=mem.sqp_it+1;
                      
    end

     cpt = toc*1e3;   % Total CPU time for the current sampling instant
    

     % X_traj = input.x;
     % U_traj = input.u;
     % Z_traj = input.z;
     % P_traj = input.od;
     % Lambda_traj = input.lambda;
     % Mu_traj = input.mu;
     % Mu_x_traj = input.mu_x;
     % Mu_i_traj = input.mu_u;
 


    

  
   



end

